#include <rclcpp/rclcpp.hpp>

#include <stack>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "ns3/core-module.h"
#include "ns3/buildings-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/batmand-module.h"

#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/physics_update.pb.h"
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include <yaml-cpp/yaml.h>

#include <wifi-application.h>
#include <flocking-application.h>

#include <udp_tcp_socket.hpp>

#include <time_probe.hpp>

using namespace ns3;

void printNeighborhood(std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> neighbors)
{
    for (auto &node : neighbors)
    {
        std::cout << "Node " << node.first << " has " << node.second.size() << " neighbors" << std::endl;
        for (auto &neighbor : node.second)
        {
            std::cout << "Neighbor " << neighbor.first << " with distance " << neighbor.second.first << " and time " << neighbor.second.second << std::endl;
        }
        std::cout << std::endl;
    }
}

/**
 * \brief Compresses a string with the zip protocol.
 *
 * \param data The string to compress.
 * \return The compressed string.
 */
static std::string
gzip_compress(const std::string &data)
{
    std::stringstream compressed;
    std::stringstream origin(data);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_compressor());
    in.push(origin);
    boost::iostreams::copy(in, compressed);

    return compressed.str();
}

/**
 * \brief Decompress a string with the zip protocol.
 *
 * \param data The compressed string to decompress.
 * \return The decompressed string.
 */
static std::string
gzip_decompress(const std::string &data)
{
    std::stringstream compressed(data);
    std::stringstream decompressed;

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(compressed);
    boost::iostreams::copy(in, decompressed);

    return decompressed.str();
}


class AdhocChainFlocking : public rclcpp::Node
{
public:
    AdhocChainFlocking() : Node("adhoc_chain_flocking")
    {
        RCLCPP_INFO(this->get_logger(), "Adhoc Chain Flocking Node Created");

        // ========================= PARAMETER READING AND LOG FILE INITIALIZATION =========================
        // Declare two parameters for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        param_desc.description = "Executes in co-simulation or ns-3 only.";
        this->declare_parameter("cosim_mode", true, param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using a ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        bool cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();
        bool verbose = this->get_parameter("verbose").get_parameter_value().get<bool>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // get the name of the folder containing the config file
        // boost::filesystem::path config_file(config_file_path);
        // boost::filesystem::path config_folder = config_file.parent_path();

        // Get the path to the ROS_WS, it is mandatory to run
        if (getenv("ROS_WS") == NULL)
        {
            RCLCPP_FATAL(this->get_logger(), "ROS_WS environment variable not set, aborting.");
            exit(EXIT_FAILURE);
        }
        else
        {
            this->m_ros_ws_path = getenv("ROS_WS");
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        // ========================= COMPUTATION TIME SAVING =========================
        this->save_compute_time = config["save_compute_time"].as<bool>();

        if (this->save_compute_time)
        {

            // Create a folder based on the experience name, if not existant already
            std::string experience_name = config["experience_name"].as<std::string>();
            if (boost::filesystem::create_directories(this->m_ros_ws_path + "/data/" + experience_name))
            {
                RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", experience_name.c_str());
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Using existing data folder for this experiment");
            }

            // Define the output file name, based on the existing files in the experience folder (incremental)
            std::string temp_path;
            int i = 1;
            while (this->m_computation_time_file.empty())
            {
                temp_path = this->m_ros_ws_path + "/data/" + experience_name + "/network" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    this->m_computation_time_file = temp_path;
                }
            }            

            // initialize the output file with headers
            this->probe = WallTimeProbe(this->m_computation_time_file);
        }

        // ========================= NS3 =========================

        // Set the seed for the random number generator
        SeedManager::SetRun(config["seed"].as<int>());

        Time step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        Time currTime = MicroSeconds(0);                                       // us
        Time simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)
        Time neighbor_timeout_value = MicroSeconds(config["neighbor_timeout_value"].as<uint32_t>());
        int numNodes = config["robots_number"].as<int>();

        std::string wifiType = config["wifi_type"].as<std::string>();
        std::string errorModelType = config["error_model_type"].as<std::string>();
        std::string propagation_loss_model = config["propagation_loss_model"].as<std::string>();
        std::string phyMode(config["phy_mode"].as<std::string>()); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
        double frequency = 5.2e9;                                  // operating frequency in Hz
        this->max_neighbors = config["max_neighbors"].as<uint32_t>();
        std::string routingAlgorithm = config["routing_algorithm"].as<std::string>();

        // Create the nodes
        this->nodes.Create(numNodes);

        // **************** MOBILITY MODULE ****************

        // For all the nodes create a ConstantPositionMobilityModel to initialize their position.
        // Assign different positions for each robot as these values will be overwritten at first step of simulation anyway
        for (int i = 0; i < numNodes; i++)
        {
            Ptr<MobilityModel> nodeMob;
            nodeMob = CreateObject<ConstantPositionMobilityModel>();
            nodeMob->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(i, i, 0.5));
            this->nodes.Get(i)->AggregateObject(nodeMob);
        }
        this->targets_reached = false;
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of MOBILITY module");

        // **************** BUILDINGS MODULE ****************

        // [Buildings] -- Define buildings with their center (x, y), and their size in the three dimensions : (size_x, size_y, height)
        std::vector<Ptr<Building>> buildings;
        for (auto building : config["buildings"])
        {
            double x_min = building["x"].as<float>() - (building["size_x"].as<float>() / 2);
            double x_max = building["x"].as<float>() + (building["size_x"].as<float>() / 2);
            double y_min = building["y"].as<float>() - (building["size_y"].as<float>() / 2);
            double y_max = building["y"].as<float>() + (building["size_y"].as<float>() / 2);
            double z_min = 0.0;
            double z_max = building["height"].as<float>();
            Ptr<Building> build = CreateObject<Building>();
            build->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
            build->SetBuildingType(Building::Office);
            build->SetExtWallsType(Building::ConcreteWithWindows);
            build->SetNFloors(1);
            build->SetNRoomsX(1);
            build->SetNRoomsY(1);
            buildings.push_back(build);
            RCLCPP_DEBUG(this->get_logger(), "Created a building with corners (%f, %f, %f, %f)", x_min, x_max, y_min, y_max);
        }

        // [Buildings] -- Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
        BuildingsHelper::Install(this->nodes);

        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of BUILDINGS module");

        // **************** WIFI MODULE ****************

        // The NetDevices on which we will install WiFi
        NetDeviceContainer devices;

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211g);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode));

        // MAC layer
        WifiMacHelper mac;
        mac.SetType("ns3::AdhocWifiMac");

        // PHY layer (we support two WifiPhy types: YansWifiPhy and SpectrumWifiPhy)
        SpectrumWifiPhyHelper spectrumWifiPhy;
        YansWifiPhyHelper yansWifiPhy;

        if (wifiType == "YansWifiPhy")
        {
            YansWifiChannelHelper yansChannel;
            if (propagation_loss_model == "LogDistancePropagationLossModel")
            {
                yansChannel = YansWifiChannelHelper::Default();
                this->m_propagationLossModel = CreateObject<LogDistancePropagationLossModel>();
            }
            else if (propagation_loss_model == "HybridBuildingsPropagationLossModel")
            {
                this->m_propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
                this->m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));     // Default 2.4e9
                this->m_propagationLossModel->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0)); // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                this->m_propagationLossModel->SetAttribute("ShadowSigmaIndoor", DoubleValue(8.0));   // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                this->m_propagationLossModel->SetAttribute("ShadowSigmaOutdoor", DoubleValue(7.0));  // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                yansChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                           "Frequency",            // Additional loss for each internal wall [dB]
                                           DoubleValue(frequency), // Default 2.4e9
                                           "ShadowSigmaExtWalls",  // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                                           DoubleValue(5.0),       // Default 5
                                           "ShadowSigmaIndoor",    // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                                           DoubleValue(8.0),       // Default 8
                                           "ShadowSigmaOutdoor",   // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                                           DoubleValue(7.0));
                yansChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
            }

            yansWifiPhy.SetChannel(yansChannel.Create());
            yansWifiPhy.Set("TxPowerStart", DoubleValue(18));
            yansWifiPhy.Set("TxPowerEnd", DoubleValue(18));
            yansWifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);


            devices = wifi.Install(yansWifiPhy, mac, this->nodes);

            yansWifiPhy.EnablePcap ("adhoc_chain_flocking", devices);

        }
        else if (wifiType == "SpectrumWifiPhy")
        {
            Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

            // create the channel condition model
            Ptr<ChannelConditionModel> m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
            m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

            // create the propagation loss model and add it to the channel condition
            this->m_propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
            m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
            m_propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(false));
            m_propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));
            spectrumChannel->AddPropagationLossModel(m_propagationLossModel);

            // Create the delay model and add it to the channel condition
            Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
            spectrumChannel->SetPropagationDelayModel(delayModel);

            spectrumWifiPhy.SetChannel(spectrumChannel);
            spectrumWifiPhy.SetErrorRateModel(errorModelType);
            spectrumWifiPhy.Set("TxPowerStart", DoubleValue(16)); // dBm  (1.26 mW)
            spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(16));
            spectrumWifiPhy.SetPcapDataLinkType (SpectrumWifiPhyHelper::DLT_IEEE802_11_RADIO);

            // Connect a callback that will save the pathloss value in an attribute every time it is calculated
            spectrumChannel->TraceConnectWithoutContext("PathLoss", MakeCallback(&AdhocChainFlocking::SpectrumPathLossTrace, this));

            devices = wifi.Install(spectrumWifiPhy, mac, this->nodes);

            spectrumWifiPhy.EnablePcap ("adhoc_chain_flocking", devices);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Unsupported WiFi type %s", wifiType.c_str());
            exit(EXIT_FAILURE);
        }

        Config::Set(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
            BooleanValue(true));
        // Fix non-unicast data rate to be the same as that of unicast
        Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
            StringValue (phyMode));

        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of WIFI module");

        /* **************** IP / ROUTING MODULE **************** */
        InternetStackHelper internet;

        if (routingAlgorithm == "BATMAN")
        {
            // Add Batman routing
            Ipv4ListRoutingHelper ipv4List;
            BatmandHelper batmand;
            batmand.Set("OGMInterval", TimeValue(Seconds(0.1)));
            Ipv4StaticRoutingHelper staticRouting;
            ipv4List.Add (staticRouting, 0);
            ipv4List.Add (batmand, 100);
            internet.SetRoutingHelper(ipv4List);
        }
        else if (routingAlgorithm == "OLSR")
        {
            // Add OLSR routing
            Ipv4ListRoutingHelper ipv4List;
            OlsrHelper olsr;
            olsr.Set("HelloInterval", TimeValue(Seconds(0.1)));
            ipv4List.Add(olsr, 100);
            internet.SetRoutingHelper(ipv4List);
        }
        else if (routingAlgorithm == "AODV")
        {
            // Add AODV routing
            Ipv4ListRoutingHelper ipv4List;
            AodvHelper aodv;
            ipv4List.Add(aodv, 100);
            internet.SetRoutingHelper(ipv4List);
        }
        else if (routingAlgorithm == "DSDV")
        {
            // Add DSDV routing
            DsdvHelper dsdv;
            dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(15)));
            dsdv.Set("SettlingTime", TimeValue(Seconds(6)));
            internet.SetRoutingHelper(dsdv);
        }
        else if (routingAlgorithm == "DSR")
        {
            // Add DSR routing (must be done after internet stack installation)
            DsrHelper dsr;
            DsrMainHelper dsrMain;
            dsrMain.Install(dsr, this->nodes);
        }
        else 
        {
            RCLCPP_FATAL(this->get_logger(), "Unsupported routing algorithm %s", routingAlgorithm.c_str());
            exit(EXIT_FAILURE);
        }

        // Actually install the internet stack on all nodes
        internet.Install(this->nodes);

        // Assign IP addresses to the net devices
        Ipv4AddressHelper addressAdhoc;
        addressAdhoc.SetBase("10.0.0.0", "255.255.255.0");
        Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign(devices);

        // Connect Trace callbacks on all nodes 
        for (int i = 0; i < numNodes; i++)
        {
            // Access the internal list of aggregated objects
            Ptr<WifiMac> wifi_mac = devices.Get(i)->GetObject<WifiNetDevice>()->GetMac();
            Ptr<WifiPhy> wifi_phy = devices.Get(i)->GetObject<WifiNetDevice>()->GetPhy();

            this->mac_to_id[wifi_mac->GetAddress()] = i;
            wifi_phy->TraceConnect("PhyTxBegin", std::to_string(i), MakeCallback(&AdhocChainFlocking::PhyTxBeginTrace, this));
            wifi_phy->TraceConnect("PhyRxEnd", std::to_string(i), MakeCallback(&AdhocChainFlocking::PhyRxEndTrace, this));
        }

        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of IP/ROUTING module");

        /* **************** APPLICATION MODULE **************** */

        // "Mission" flow : unicast, unidirectional
        bool mission_flow = config["mission_flow"]["enable"].as<bool>();
        // uint32_t source_node_id = config["mission_flow"]["source_robot_id"].as<uint32_t>();
        uint32_t sink_node_id = config["mission_flow"]["sink_robot_id"].as<uint32_t>();
        double start_traffic_time = config["mission_flow"]["start_time"].as<double>();  // s
        double stop_traffic_time = config["mission_flow"]["stop_time"].as<double>();    // s
        uint32_t packet_size = config["mission_flow"]["packet_size"].as<uint32_t>();                 // bytes        
        uint64_t interval = config["mission_flow"]["interval"].as<uint64_t>();                       // us
        uint16_t mission_flow_port = config["mission_flow"]["port"].as<uint16_t>();
        this->mission_flow_id = config["mission_flow"]["flow_id"].as<uint8_t>();
        this->mission_flow_timeout = MicroSeconds(config["mission_flow"]["timeout"].as<uint64_t>());

        // "Sender" and "Receiver" are classes defined in the custom ns3 Application wifi-application.h
        if (mission_flow)
        {
            for (auto source_node_id : config["mission_flow"]["source_robot_ids"])
            {
                Ptr<Sender> sender = CreateObject<Sender>();
                Ptr<ns3::Node> source_node = this->nodes.Get(source_node_id.as<uint32_t>());
                source_node->AddApplication(sender);
                sender->SetStartTime(Seconds(start_traffic_time));
                sender->SetStopTime(Seconds(stop_traffic_time));
                sender->SetAttribute("Destination", Ipv4AddressValue(this->nodes.Get(sink_node_id)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal()));
                sender->SetAttribute("Port", UintegerValue(mission_flow_port));
                sender->SetAttribute("PacketSize", UintegerValue(packet_size));
                sender->SetAttribute("NumPackets", UintegerValue(4294967295));
                sender->SetAttribute("FlowId", UintegerValue(this->mission_flow_id));
                Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
                rand->SetAttribute("Constant", DoubleValue(interval/1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
                sender->SetAttribute("Interval", PointerValue(rand));
                sender->TraceConnectWithoutContext("Tx", MakeCallback(&AdhocChainFlocking::mission_flow_sender_clbk, this));

                RCLCPP_INFO(this->get_logger(), "Configured mission app (source) for node %d", source_node_id.as<uint32_t>());
            }
            
            Ptr<Receiver> receiver = CreateObject<Receiver>();
            Ptr<ns3::Node> sink_node = this->nodes.Get(sink_node_id);
            sink_node->AddApplication(receiver);
            receiver->SetStartTime(Seconds(0.0));
            receiver->SetAttribute("Port", UintegerValue(mission_flow_port));
            receiver->TraceConnectWithoutContext("Rx", MakeCallback(&AdhocChainFlocking::mission_flow_receiver_clbk, this));

            RCLCPP_INFO(this->get_logger(), "Configured mission app (sink) for node %d", sink_node_id);
            
        }

        // Broadcast flow
        bool broadcast_flow = config["broadcast_flow"]["enable"].as<bool>();
        double start_broadcast_time = config["broadcast_flow"]["start_time"].as<double>();
        double stop_broadcast_time = config["broadcast_flow"]["stop_time"].as<double>();
        uint32_t broadcast_packet_size = config["broadcast_flow"]["packet_size"].as<uint32_t>();
        uint64_t broadcast_interval = config["broadcast_flow"]["interval"].as<uint64_t>();
        uint16_t broadcast_flow_port = config["broadcast_flow"]["port"].as<uint16_t>();
        this->broadcast_flow_timeout = MicroSeconds(config["broadcast_flow"]["timeout"].as<uint64_t>());
        
        if (broadcast_flow)
        {
            // Random start, otherwise they never access the medium (I think)
            double min = 0.0;
            double max = 1.0;
            Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
            x->SetAttribute("Min", DoubleValue(min));
            x->SetAttribute("Max", DoubleValue(max));

            for (uint32_t i=0; i < this->nodes.GetN(); i++)
            {
                Ptr<ns3::Node> node = this->nodes.Get(i);

                Ptr<FlockingBroadcaster> broadcaster = CreateObject<FlockingBroadcaster>();
                Ptr<FlockingReceiver> flock_receiver = CreateObject<FlockingReceiver>();

                broadcaster->SetStartTime(Seconds(start_broadcast_time + x->GetValue()));
                broadcaster->SetStopTime(Seconds(stop_broadcast_time));
                broadcaster->SetAttribute("PacketSize", UintegerValue(broadcast_packet_size));
                broadcaster->SetAttribute("Port", UintegerValue(broadcast_flow_port));
                Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
                rand->SetAttribute("Constant", DoubleValue(broadcast_interval/1000000.0f)); // from us to s (because custom class Sender uses Interval as Seconds)
                broadcaster->SetAttribute("Interval", PointerValue(rand));

                flock_receiver->SetStartTime(Seconds(0.0));

                if (broadcaster->TraceConnect("Tx",std::to_string(i), MakeCallback(&AdhocChainFlocking::flocking_broadcaster_clbk, this)))
                {
                    RCLCPP_DEBUG(this->get_logger(), "Connected broadcaster to node %d", i);
                }
                if (flock_receiver->TraceConnect("Rx", std::to_string(i), MakeCallback(&AdhocChainFlocking::flock_receiver_clbk, this)))
                {
                    RCLCPP_DEBUG(this->get_logger(), "Connected receiver to node %d", i);
                }
                
                node->AddApplication(broadcaster);
                node->AddApplication(flock_receiver);
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of APPLICATION module");

        /* **************** STATS MODULE **************** */
        bool stats_enabled = config["enable_stats"].as<bool>();

        std::string experiment = config["experience_name"].as<std::string>();
        DataCollector data;
        if (stats_enabled)
        {
            std::string strategy = "VAT flocking";
            std::string input = config_file_path;
            std::string runID = config["run_id"].as<std::string>();

            data.DescribeRun(experiment, strategy, input, runID);

            // Add any information we wish to record about this run.
            data.AddMetadata("author", "tbalaguer");

            // Create a counter for the number of Packets sent WITHIN THE TARGETS
            this->missionTotalTx = CreateObject<PacketCounterCalculator>();
            this->missionTotalTx->SetKey("mission_sent_packets_within_target");
            // this->missionTotalTx->SetContext("mission flow node[" + std::to_string(source_node_id) + "]");
            data.AddDataCalculator(this->missionTotalTx);

            // Create a counter for the number of Packets received WITHIN THE TARGETS
            this->missionTotalRx = CreateObject<PacketCounterCalculator>();
            this->missionTotalRx->SetKey("mission_received_packets_within_target");
            // this->missionTotalRx->SetContext("mission flow node[" + std::to_string(sink_node_id) + "]");
            data.AddDataCalculator(this->missionTotalRx);

            // Create a statistics object for the delay of the packets
            this->missionDelay = CreateObject<TimeMinMaxAvgTotalCalculator>();
            this->missionDelay->SetKey("mission_packet_delay");
            this->missionDelay->SetContext("mission flow");
            data.AddDataCalculator(this->missionDelay);

            // Create a counter for the number of Packets received by each 'navigation flow' server
            for (int i = 0; i < numNodes; i++)
            {
                Ptr<PacketCounterCalculator> navTotalRx = CreateObject<PacketCounterCalculator>();
                navTotalRx->SetKey("nav_received_packets");
                navTotalRx->SetContext("nav flow node[" + std::to_string(i) + "]");
                this->navTotalRxVector.push_back(navTotalRx);
                data.AddDataCalculator(this->navTotalRxVector[i]);
            }

            // Connect the callback to all the trace sources
            Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&AdhocChainFlocking::PhyTxEndTrace, this));

            this->navEffectiveTx = CreateObject<PacketCounterCalculator>();
            navEffectiveTx->SetKey("nav_effective_tx");
            navEffectiveTx->SetContext("nav flow effectively sent packets");
            data.AddDataCalculator(this->navEffectiveTx);

            // Create a counter for the number of Packets sent by each 'navigation flow' client
            for (int i = 0; i < numNodes; i++)
            {
                Ptr<PacketCounterCalculator> navTotalTx = CreateObject<PacketCounterCalculator>();
                navTotalTx->SetKey("nav_sent_packets");
                navTotalTx->SetContext("nav flow node[" + std::to_string(i) + "]");
                this->navTotalTxVector.push_back(navTotalTx);
                data.AddDataCalculator(this->navTotalTxVector[i]);
            }
            RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of STATS module");
        }

        // Colours are fun: "\x1b[32m" enables red color ; "\x1b[0m" set default back
        RCLCPP_INFO(this->get_logger(), "\x1b[32mns-3 configuration finished. Starting simulation !\x1b[0m");

        if (cosim_mode)
        {
            // **************** UDS SOCKET FOR NETWORK COORDINATOR ****************
            // Create and connect UDS Socket
            CustomSocket *socket;
            boost::asio::io_context io_context;
            if (config["net_use_uds"].as<bool>())
            {
                socket = new UDSSocket(io_context);
                socket->accept(config["net_uds_server_address"].as<std::string>(), 0);
            }
            else
            {
                socket = new TCPSocket(io_context);
                socket->accept(config["net_ip_server_address"].as<std::string>(), config["net_ip_server_port"].as<unsigned short>());
            }
            RCLCPP_INFO(this->get_logger(), "\x1b[32m Socket connected with Coordinator \x1b[0m");

            uint64_t bytes_received = 0;

            // **************** MAIN SIMULATION LOOP ****************
            while ((currTime < simEndTime || simEndTime == Seconds(0.0)) && rclcpp::ok())
            {

                // Wait until reception of a message on the UDS socket
                std::string received_data = gzip_decompress(socket->receive_one_message());

                // Initialize empty protobuf message type [PhysicsUpdate]
                physics_update_proto::PhysicsUpdate physics_update_msg;
                // Transform the message received from the UDS socket [string] -> [protobuf]
                physics_update_msg.ParseFromString(received_data);

                // Read the "physical" information transmitted by the NetworkCoordinator, and update the node's positions
                // Also verifies that the number of nodes sent by the NetworkCoordinator corresponds to the number of existing nodes in NS-3
                rclcpp::Clock clock;
                robots_positions_proto::RobotsPositions robots_positions_msg;

                // if (currTime % Seconds(0.1) == Time(0)){
                //     uint64_t bytes_received_this_iteration = mission_server->GetReceived()*packet_size - bytes_received;
                //     bytes_received = mission_server->GetReceived()*packet_size;
                //     RCLCPP_INFO(this->get_logger(), "Mission flow throughput: %f Mbps", (float)(bytes_received_this_iteration * 10 / 1000000.0));
                // }

                if (!physics_update_msg.robots_positions().empty())
                {
                    RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                    robots_positions_msg.ParseFromString(gzip_decompress(physics_update_msg.robots_positions()));

                    // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                    // Then, update the node's positions (orientation is ignored for now)
                    if (this->nodes.GetN() != (uint32_t)robots_positions_msg.robot_pose_size())
                    {
                        if (verbose)
                        {
                            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                                 clock,
                                                 1000, // ms
                                                 "Network simulator received position information of %i robots but NS-3 has %u nodes.",
                                                 robots_positions_msg.robot_pose_size(),
                                                 this->nodes.GetN());
                        }
                    }
                    else
                    {
                        for (uint32_t i = 0; i < this->nodes.GetN(); i++)
                        {
                            Vector pos;
                            pos.x = robots_positions_msg.robot_pose(i).x();
                            pos.y = robots_positions_msg.robot_pose(i).y();
                            pos.z = robots_positions_msg.robot_pose(i).z();

                            // ns-s don't like negative and 0 positions
                            if(pos.z <= 0)
                            {
                                pos.z = 0.1;
                            }

                            // Set node position
                            Ptr<MobilityModel> mobility = this->nodes.Get(i)->GetObject<MobilityModel>();
                            mobility->SetPosition(pos);
                        }
                    }
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
                }

                if (wifiType == "YansWifiPhy")
                {
                    this->updateNeighborsPathloss();
                }

                // Once all the events are scheduled, advance W time in the simulation and stop
                Simulator::Stop(step_size);

                if (this->save_compute_time)
                {
                    this->probe.start();
                }

                Simulator::Run();

                if (this->save_compute_time)
                {
                    this->probe.stop();
                }

                this->timeoutNeighbors();

                // Create an object giving the neighborhood of each node, based on the packets received by their UDP server, neighbors lifetime is 1 second.
                // std::map<uint32_t, std::map<uint32_t, double>> neighbors = create_neighbors(neighbor_timeout_value);

                std::string response = GenerateResponseProtobuf();

                // Send the response to the network coordinator
                socket->send_one_message(response);

                currTime += step_size;
            }
        }
        else
        {
            Simulator::Stop(simEndTime);
            std::cout << "Starting simulation." << std::endl;

            if (this->save_compute_time)
            {
                this->probe.start();
            }

            Simulator::Run();

            if (this->save_compute_time)
            {
                this->probe.stop();
            }

        }

        RCLCPP_INFO(this->get_logger(), "Simulation finished.");

        if (stats_enabled)
        {
            Ptr<DataOutputInterface> output = CreateObject<SqliteDataOutput>();

            if (output)
            {
                output->SetFilePrefix(experiment);
                output->Output(data);
            }

            if (mission_flow)
            {
                // std::cout << "Mission source sent " << sender->GetSent() << " packets" << std::endl;
                // std::cout << "Mission sink received " << receiver->GetReceived() << " packets ( " << (float)(100.0 * receiver->GetReceived() / (float)sender->GetSent()) << "% )" << std::endl;
            }
        }

        Simulator::Destroy();

        exit(EXIT_SUCCESS);
    }

private:
    NodeContainer nodes;
    std::string m_ros_ws_path;

    std::map<uint32_t, std::map<uint32_t, double>> pathlosses;
    std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> mission_neighbors;
    std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> broadcast_neighbors;
    std::map<Mac48Address, uint32_t> mac_to_id;
    uint8_t mission_flow_id;
    Time mission_flow_timeout;
    Time broadcast_flow_timeout;
    uint32_t max_neighbors;

    Ptr<PropagationLossModel> m_propagationLossModel;

    bool targets_reached;

    // ns3 Trace callbacks
    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    void PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow);
    void PhyTxEndTrace(Ptr<const Packet> packet);
    void PhyRxEndTrace(std::string context, Ptr<const Packet> packet);
    void mission_flow_receiver_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk(Ptr<const Packet> packet);
    void flock_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id);
    void flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet);

    // save compute time (optional)
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;

    // Stats attributes (optional)
    Ptr<PacketCounterCalculator> missionTotalRx;
    Ptr<PacketCounterCalculator> missionTotalTx;
    Ptr<TimeMinMaxAvgTotalCalculator> missionDelay;
    std::vector<Ptr<PacketCounterCalculator>> navTotalRxVector;
    std::vector<Ptr<PacketCounterCalculator>> navTotalTxVector;
    Ptr<PacketCounterCalculator> navEffectiveTx;

    // co-simulation specific methods
    std::string GenerateResponseProtobuf();
    std::string generate_neighbors_msg();

    void timeoutNeighbors();
    void updateNeighborsPathloss();

};

void AdhocChainFlocking::SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t txId = txPhy->GetDevice()->GetNode()->GetId();
    uint32_t rxId = rxPhy->GetDevice()->GetNode()->GetId();
    this->pathlosses[txId][rxId] = -lossDb;
    // std::cout << "Tx: " << txId << " Rx: " << rxId << " Loss: " << lossDb << std::endl;
}

void AdhocChainFlocking::PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow)
{
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t destId = this->mac_to_id.find(hdr.GetAddr1())->second;
    
    // From a networking POV, this is cheating, we are sending a message, we can't know what is the reception power, however here it is accepted for the sake of simplicity
    double rxPow = this->pathlosses[nodeId][destId];
    Time txTime = Simulator::Now();

    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->mission_flow_id)
        {
            // We are sending a "mission" packet, link-layer destination should be our (outgoing) neighbor
            std::cout << nodeId << ": Sending a mission packet to " << destId << std::endl;
            this->mission_neighbors[nodeId][destId] = std::make_pair(rxPow, txTime);
        }
    }
}

void AdhocChainFlocking::PhyTxEndTrace(Ptr<const Packet> packet)
{    
    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->mission_flow_id)
        {
            this->navEffectiveTx->PacketUpdate("", packet);
        }
    }
}

void AdhocChainFlocking::PhyRxEndTrace(std::string context, Ptr<const Packet> packet)
{
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t sourceId = this->mac_to_id.find(hdr.GetAddr2())->second;
    uint32_t destId = this->mac_to_id.find(hdr.GetAddr1())->second;

    // There should be a way of reading this directly from the packet, but this is easier (?)
    double rxPow = this->pathlosses[nodeId][sourceId];
    Time rxTime = Simulator::Now();

    // We "heard" a packet from a peer, the link-layer source is a broadcast neighbor
    // this->broadcast_neighbors[nodeId][sourceId] = std::make_pair(rxPow, rxTime);

    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->mission_flow_id && nodeId == destId)
        {
            // We are receiving a "mission" packet, link-layer destination should be our (outgoing) neighbor
            std::cout << nodeId << ": Receiving a mission packet from " << sourceId << std::endl;
            this->mission_neighbors[nodeId][sourceId] = std::make_pair(rxPow, rxTime);
        }
    }
}


void AdhocChainFlocking::mission_flow_receiver_clbk(Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Mission flow packet received");
}

void AdhocChainFlocking::mission_flow_sender_clbk(Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Mission flow packet sent");
}

void AdhocChainFlocking::flock_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    RCLCPP_DEBUG(this->get_logger(), "Flock packet received");

    Time rxTime = Simulator::Now();
    double rxPow = this->pathlosses[std::stoi(context)][peer_id];
    this->broadcast_neighbors[std::stoi(context)][peer_id] = std::make_pair(rxPow, rxTime);
}

void AdhocChainFlocking::flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Flocking broadcast packet sent");
}

/**
 * @brief Generates an ordered_neighbors_msg containing the neighbors of each node, sorted by pathloss, and serializes it
 * 
 * @returns A string-serialized version of the "ordered_neighbors" protobuf message
 */
std::string
AdhocChainFlocking::generate_neighbors_msg()
{
    std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> neighbors;
    std::map<uint32_t, ordered_neighbors_proto::OrderedNeighbors_Role> agent_roles;

    ordered_neighbors_proto::OrderedNeighborsList ordered_neighbors_msg;

    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        if (this->broadcast_neighbors.find(i) == this->broadcast_neighbors.end() || this->broadcast_neighbors[i].empty())
        {
            // We don't have any broadcast neighbors, that means we are disconnected from the fleet !
            // std::cout << "No broadcast neighbors for node " << i << std::endl;
            agent_roles[i] = ordered_neighbors_proto::OrderedNeighbors_Role_IDLE;
        }
        else if (this->mission_neighbors.find(i) == this->mission_neighbors.end() || this->mission_neighbors[i].empty())
        {
            // Use broadcast neighbors, we don't have any mission neighbors !
            // std::cout << "Using broadcast neighbors for node " << i << std::endl;
            neighbors[i] = this->broadcast_neighbors[i];
            agent_roles[i] = ordered_neighbors_proto::OrderedNeighbors_Role_POTENTIAL;
        }
        else
        {
            // Use mission neighbors
            // std::cout << "Using mission neighbors for node " << i << std::endl;
            neighbors[i] = this->mission_neighbors[i];
            agent_roles[i] = ordered_neighbors_proto::OrderedNeighbors_Role_MISSION;
        }
    }

    // Create and add the ordered_neighbors protobuf message
    for (auto const &agent : neighbors)
    {
        ordered_neighbors_proto::OrderedNeighbors *neighbor_msg = ordered_neighbors_msg.add_ordered_neighbors();
        std::vector<double> ordered_pathlosses;
        neighbor_msg->set_agentid(agent.first);
        neighbor_msg->set_role(agent_roles[agent.first]);
        // Sort the pathlosses to add them in the protobuf message in the right order.
        for (auto const &neigh : agent.second)
        {
            ordered_pathlosses.push_back(neigh.second.first);
        }
        std::sort(ordered_pathlosses.begin(), ordered_pathlosses.end(), std::greater<double>());

        // Search on the values of the pathloss map, which is quite bad if two neighbors have exactly the same pathloss
        for (auto const &pathloss : ordered_pathlosses)
        {
            for (auto const &neigh : agent.second)
            {
                if (neigh.second.first == pathloss && pathloss != 0) 
                {
                    neighbor_msg->add_neighborid(neigh.first);
                    neighbor_msg->add_linkquality(neigh.second.first);
                    neighbor_msg->add_neighbortype(agent_roles[agent.first]);           //!< agent_roles[agent.first] always valid because created above for all agents
                    neighbor_msg->add_time_val(neigh.second.second.ToInteger(Time::US));
                }
            }
        }

    }

    // std::cout << ordered_neighbors_msg.DebugString() << std::endl;
    
    std::string str_response;
    ordered_neighbors_msg.SerializeToString(&str_response);
    return str_response;
}

/**
 * \brief Generates the final protobuf message of protobuf_msgs/NetworkUpdate.
 *
 * Fills the response message with the updated neighborhood information.
 * String-serialize the protobuf message, compress it, and return it to the network coordinator.
 *
 * \param PhysicsUpdate_msg A protobuf message of type NetworkUpdate.
 * \return A string-serialized version of the updated protobuf message.
 */
std::string
AdhocChainFlocking::GenerateResponseProtobuf()
{
    // Change message type to "END"
    network_update_proto::NetworkUpdate network_update_msg;
    network_update_msg.set_msg_type(network_update_proto::NetworkUpdate::END);
    network_update_msg.set_ordered_neighbors(gzip_compress(generate_neighbors_msg()));
    std::string str_response;
    network_update_msg.SerializeToString(&str_response);
    str_response = gzip_compress(str_response);

    return str_response;
}

void AdhocChainFlocking::timeoutNeighbors()
{
    // remove timed-out neighbors from broadcast_neighbors  
    for (std::pair<uint32_t, std::map<uint32_t, std::pair<double, Time>>> const &agent : this->broadcast_neighbors)
    {
        for (std::pair<uint32_t, std::pair<double, Time>> const &neigh : agent.second)
        {
            if (Simulator::Now() - neigh.second.second > this->broadcast_flow_timeout)
            {
                // std::cout << "Removing broadcast neighbor " << neigh.first << " from agent " << agent.first << std::endl;
                // printNeighborhood(this->broadcast_neighbors);
                this->broadcast_neighbors[agent.first].erase(neigh.first);
            }
        }
    }
    // remove timed-out neighbors from mission_neighbors
    for (std::pair<uint32_t, std::map<uint32_t, std::pair<double, Time>>> const &agent : this->mission_neighbors)
    {
        for (auto const &neigh : agent.second)
        {
            if (Simulator::Now() - neigh.second.second > this->mission_flow_timeout)
            {
                // std::cout << "Removing mission neighbor " << neigh.first << " from agent " << agent.first << std::endl;
                this->mission_neighbors[agent.first].erase(neigh.first);
            }
        }
    }
}

void AdhocChainFlocking::updateNeighborsPathloss()
{
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < this->nodes.GetN(); j++)
        {
            if (i != j)
            {
                Ptr<MobilityModel> mobilityA = this->nodes.Get(i)->GetObject<MobilityModel>();
                Ptr<MobilityModel> mobilityB = this->nodes.Get(j)->GetObject<MobilityModel>();

                double rxPow = this->m_propagationLossModel->CalcRxPower(18.0, mobilityA, mobilityB);
                this->pathlosses[i][j] = rxPow; // it's not really a pathloss but it's the same with a constant difference
                // std::cout << "Node " << i << " -> Node " << j << " : " << rxPow << std::endl;
            }
        }
    }

}

/**
 * \brief The main function, spins the ROS2 Node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdhocChainFlocking>());
    rclcpp::shutdown();
    return 0;
}

/**
 * This file is the network simulator based on ns-3 for the DANCERS co-simulator.
 *
 * WIFI : Ad-hoc mode
 */

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "boost/filesystem.hpp"
#include "boost/asio.hpp"
#include "boost/iostreams/filtering_streambuf.hpp"
#include "boost/iostreams/copy.hpp"
#include "boost/iostreams/filter/gzip.hpp"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/node-list.h"
#include "ns3/buildings-module.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/csma-helper.h"

#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/wifi-module.h"

#include "ns3/udp-client-server-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/flow-monitor-helper.h"

#include "ns3/stats-module.h"

#include "wifi-application.h"
#include "chain-flocking-application.h"

#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include "udp_tcp_socket.hpp"

#include <time_probe.hpp>

#include <yaml-cpp/yaml.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("UAV_fleet_adhoc_network");

/**
 * \brief Updates the position of a node.
 *
 * \param node A pointer to the target node.
 * \param position A ns3::Vector containing the [x, y, z] position of the drone, with respect to the origin.
 */
void SetNodePosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    mobility->SetPosition(position);

    // std::cout << node->GetId() << ": " << position << std::endl;
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

/**
 * \brief Receives a message from a socket.
 *
 * This function will block until the next message is received, read its header (first 4 bytes)
 * and then read the content of the message and return it as a string.
 *
 * \param sock The socket on which to listen for the next message.
 * \return The received message as a std::string.
 */
std::string
receive_one_message(boost::asio::local::stream_protocol::socket &sock)
{
    // Read Preamble
    uint32_t data_preamble[4];
    size_t length = sock.receive(boost::asio::buffer(data_preamble, 4));
    uint32_t receive_length = ntohl(*data_preamble);
    // Read Message
    char data[receive_length];
    length = sock.receive(boost::asio::buffer(data, receive_length));
    std::string data_string(data, length);

    return data_string;
}

/**
 * \brief Sends a message from a socket.
 *
 * \param sock The socket used to send the message.
 * \param str The string message to send.
 */
void send_one_message(boost::asio::local::stream_protocol::socket &sock, std::string str)
{
    // Send Preamble
    std::size_t response_size = str.size();
    // static_cast<uint32_t>(response_size);
    uint32_t send_length = htonl(response_size);
    sock.send(boost::asio::buffer(&send_length, 4));
    // Send Message
    sock.send(boost::asio::buffer(str.data(), str.size()));
}

std::string
generate_neighbors_msg(std::map<uint32_t, std::map<uint32_t, double>> neighbors, int max_neighbors)
{
    ordered_neighbors_proto::OrderedNeighborsList ordered_neighbors_msg;
    for (auto const &agent : neighbors)
    {
        std::vector<double> ordered_pathlosses;
        ordered_neighbors_proto::OrderedNeighbors *neighbor_msg = ordered_neighbors_msg.add_ordered_neighbors();
        neighbor_msg->set_agentid(agent.first);
        int num_neighbors = 0;
        // Sort the pathlosses to add them in the protobuf message in the right order.
        for (auto const &neigh : agent.second)
        {
            ordered_pathlosses.push_back(neigh.second);
        }
        std::sort(ordered_pathlosses.begin(), ordered_pathlosses.end(), std::greater<double>());
        for (auto const &pathloss : ordered_pathlosses)
        {
            for (auto const &neigh : agent.second)
            {
                if (neigh.second == pathloss && pathloss != 0 && num_neighbors < max_neighbors) // search on the values of the pathloss map, generates a bug if two agent pairs have exactly the same pathloss
                {
                    neighbor_msg->add_neighborid(neigh.first);
                    neighbor_msg->add_linkquality(neigh.second);
                    num_neighbors++;
                }
            }
        }
        // std::cout << neighbor_msg->DebugString() << std::endl;
    }

    std::string str_response;
    ordered_neighbors_msg.SerializeToString(&str_response);
    return str_response;
}

/**
 * \brief Generates the final protobuf message of protobuf_msgs/NetworkUpdate.
 *
 * Usually, this function will be passed the [NetworkUpdate] protobuf message with message-type BEGIN from the Network coordinator.
 * It will fill the message with the Bit Error Rate (BER) of each packet ,
 * change the message-type to END, string-serialize the protobuf message and return it to the network coordinator.
 *
 * \param PhysicsUpdate_msg A protobuf message of type NetworkUpdate.
 * \return A string-serialized version of the updated protobuf message.
 */
std::string
generate_response(network_update_proto::NetworkUpdate NetworkUpdate_msg, std::map<uint32_t, std::map<uint32_t, double>> neighbors, int max_neighbors)
{
    // Change message type to "END"
    NetworkUpdate_msg.set_msg_type(network_update_proto::NetworkUpdate::END);
    NetworkUpdate_msg.set_ordered_neighbors(gzip_compress(generate_neighbors_msg(neighbors, max_neighbors)));
    std::string str_response;
    NetworkUpdate_msg.SerializeToString(&str_response);

    return str_response;
}

/**
 * We declare a ROS2 Node here. It takes the form of a class which constructor will be called by rclcpp::spin().
 * The use of ROS for the NS3 simulator is essentially to harmonize the code, and allow the use of a standard logging system (RCLCPP_INFO, etc.).
 * We could very well have standard NS-3 code here.
 */
class Ns3Simulation : public rclcpp::Node
{
public:
    Ns3Simulation() : Node("ns3_simulation")
    {

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
        boost::filesystem::path config_file(config_file_path);
        boost::filesystem::path config_folder = config_file.parent_path();

        // Get the path to the ROS_WS, it is mandatory to run
        if (getenv("ROS_WS") == NULL)
        {
            RCLCPP_FATAL(this->get_logger(), "ROS_WS environment variable not set, aborting.");
            exit(EXIT_FAILURE);
        }
        else
        {
            this->ros_ws_path = getenv("ROS_WS");
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        // ========================= COMPUTATION TIME SAVING =========================
        this->save_compute_time = config["save_compute_time"].as<bool>();

        if (this->save_compute_time)
        {

            // Create a folder based on the experience name, if not existant already
            std::string experience_name = config["experience_name"].as<std::string>();
            if (boost::filesystem::create_directories(this->ros_ws_path + "/data/" + experience_name))
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
                temp_path = ros_ws_path + "/data/" + experience_name + "/network" + std::to_string(i) + ".csv";
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

        // ========================= NS3 CONFIGURATION =========================

        // Set the seed for the random number generator
        SeedManager::SetRun(config["seed"].as<int>());

        Time step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        Time currTime = MicroSeconds(0);                                       // us
        Time simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)
        Time neighbor_timeout_value = MicroSeconds(config["neighbor_timeout_value"].as<uint32_t>());
        uint32_t numNodes = config["robots_number"].as<int>();

        std::string wifiType = config["wifi_type"].as<std::string>();
        std::string errorModelType = config["error_model_type"].as<std::string>();
        std::string propagation_loss_model = config["propagation_loss_model"].as<std::string>();
        std::string phyMode(config["phy_mode"].as<std::string>()); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
        double frequency = 5.2e9;                                  // operating frequency in Hz

        uint64_t mission_bytes_received_last_second = 0;

        this->max_neighbors = config["max_neighbors"].as<int>();

        // Create the nodes
        this->nodes.Create(numNodes);

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

        // **************** MOBILITY MODULE ****************

        // For all the nodes create a ConstantPositionMobilityModel to initialize their position.
        // Assign different positions for each robot as these values will be overwritten at first step of simulation anyway
        for (uint32_t i = 0; i < numNodes; i++)
        {
            Ptr<MobilityModel> nodeMob;
            nodeMob = CreateObject<ConstantPositionMobilityModel>();
            nodeMob->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(i, i, 0.5));
            this->nodes.Get(i)->AggregateObject(nodeMob);
        }
        this->targets_reached = false;
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of MOBILITY module");

        // **************** PROPAGATION MODULE ****************

        SpectrumWifiPhyHelper spectrumPhy;
        YansWifiPhyHelper wifiPhy;

        if (wifiType == "YansWifiPhy")
        {
            YansWifiChannelHelper channel;
            if (propagation_loss_model == "LogDistancePropagationLossModel")
            {
                channel = YansWifiChannelHelper::Default();
                this->m_propagationLossModel = CreateObject<LogDistancePropagationLossModel>();
            }
            else if (propagation_loss_model == "HybridBuildingsPropagationLossModel")
            {
                this->m_propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
                this->m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));     // Default 2.4e9
                this->m_propagationLossModel->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0)); // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                this->m_propagationLossModel->SetAttribute("ShadowSigmaIndoor", DoubleValue(8.0));   // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                this->m_propagationLossModel->SetAttribute("ShadowSigmaOutdoor", DoubleValue(7.0));  // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                channel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                           "Frequency",            // Additional loss for each internal wall [dB]
                                           DoubleValue(frequency), // Default 2.4e9
                                           "ShadowSigmaExtWalls",  // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                                           DoubleValue(5.0),       // Default 5
                                           "ShadowSigmaIndoor",    // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                                           DoubleValue(8.0),       // Default 8
                                           "ShadowSigmaOutdoor",   // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                                           DoubleValue(7.0));
                channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
            }

            wifiPhy.SetChannel(channel.Create());
            wifiPhy.Set("TxPowerStart", DoubleValue(18));
            wifiPhy.Set("TxPowerEnd", DoubleValue(18));
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

            spectrumPhy.SetChannel(spectrumChannel);
            spectrumPhy.SetErrorRateModel(errorModelType);
            spectrumPhy.Set("TxPowerStart", DoubleValue(16)); // dBm  (1.26 mW)
            spectrumPhy.Set("TxPowerEnd", DoubleValue(16));

            spectrumChannel->TraceConnectWithoutContext("PathLoss", MakeCallback(&Ns3Simulation::SpectrumPathLossTrace, this));
        }
        else
        {
            NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
        }
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of PHY layer");

        // **************** WIFI MODULE ****************

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211g);
        WifiMacHelper mac;

        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode));

        NetDeviceContainer devices;

        mac.SetType("ns3::AdhocWifiMac");

        if (wifiType == "YansWifiPhy")
        {
            devices = wifi.Install(wifiPhy, mac, this->nodes);
        }

        else if (wifiType == "SpectrumWifiPhy")
        {
            devices = wifi.Install(spectrumPhy, mac, this->nodes);
        }
        else
        {
            NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
        }

        Config::Set(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
            BooleanValue(true));

        // [Buildings] -- Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
        BuildingsHelper::Install(this->nodes);

        /* **************** IP / ROUTING MODULE **************** */
        Ipv4ListRoutingHelper ipv4List;
        AodvHelper aodv;
        InternetStackHelper internet;
        ipv4List.Add(aodv, 100);

        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
        aodv.PrintRoutingTableAllEvery(Seconds(5), routingStream);

        internet.SetRoutingHelper(ipv4List);
        internet.Install(this->nodes);

        Ipv4AddressHelper addressAdhoc;
        addressAdhoc.SetBase("10.0.0.0", "255.255.255.0");
        Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign(devices);

        // Create the broadcast address
        Ipv4Address broadcastAddress = Ipv4Address::GetBroadcast();

        /* **************** APPLICATION MODULE **************** */

        // "Flocking" flow : broadcast position and velocity to neighbors 
        uint32_t nav_flow_broadcast_period = config["pose_broadcast_period"].as<uint32_t>();        // us
        uint32_t nav_flow_packet_size = config["pose_broadcast_packet_size"].as<uint32_t>();        // bytes
        uint32_t nav_flow_num_relays = config["max_neighbors"].as<uint32_t>();                   // units

        // Random start, otherwise they never access the medium (I think)
        double min = 0.0;
        double max = 1.0;
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
        x->SetAttribute("Min", DoubleValue(min));
        x->SetAttribute("Max", DoubleValue(max));
        for (int i = 0; i < numNodes; i++)
        {
            Ptr<ChainFlocking> flocking_application = CreateObject<ChainFlocking>();
            Ptr<ConstantRandomVariable> random_var = CreateObject<ConstantRandomVariable>();
            random_var->SetAttribute("Constant", DoubleValue(nav_flow_broadcast_period/1000000.0f)); // from us to s (because custom class Sender uses Interval as Seconds)
            flocking_application->SetAttribute("Interval", PointerValue(random_var));
            flocking_application->SetAttribute("PacketSize", UintegerValue(nav_flow_packet_size));
            flocking_application->SetAttribute("Port", UintegerValue(8080));
            flocking_application->SetStartTime(Seconds(0.0 + x->GetValue()));
            flocking_application->SetAttribute("NumRelays", UintegerValue(nav_flow_num_relays));
            flocking_application->SetAttribute("Timeout", TimeValue(Seconds(1.0)));
            if (YAML::Node nav_flow_target = config["secondary_objectives"][i])
            {
                if(nav_flow_target[3].as<bool>())
                {
                    flocking_application->SetLeaderRank(0);
                }
                else
                {
                    flocking_application->SetLeaderRank(UINT32_MAX-1);
                }
            }

            this->nodes.Get(i)->AddApplication(flocking_application);
        }


        // "Mission" flow : unicast, unidirectional
        bool mission_flow = config["mission_flow"]["enable"].as<bool>();
        uint32_t source_node_id = config["mission_flow"]["source_robot_id"].as<uint32_t>();
        uint32_t sink_node_id = config["mission_flow"]["sink_robot_id"].as<uint32_t>();
        double start_traffic_time = config["mission_flow"]["start_traffic_time"].as<double>();  // s
        double stop_traffic_time = config["mission_flow"]["stop_traffic_time"].as<double>();    // s
        uint32_t packet_size = config["mission_flow"]["packet_size"].as<uint32_t>();                 // bytes        
        uint64_t interval = config["mission_flow"]["interval"].as<uint64_t>();                       // us
        uint16_t mission_flow_port = config["mission_flow"]["port"].as<uint16_t>();

        Ptr<Sender> sender = CreateObject<Sender>();
        Ptr<Receiver> receiver = CreateObject<Receiver>();
        if (mission_flow)
        {
            Ptr<ns3::Node> source_node = this->nodes.Get(source_node_id);
            source_node->AddApplication(sender);
            sender->SetStartTime(Seconds(start_traffic_time));
            sender->SetStopTime(Seconds(stop_traffic_time));
            sender->SetAttribute("Destination", Ipv4AddressValue(this->nodes.Get(sink_node_id)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal()));
            sender->SetAttribute("Port", UintegerValue(mission_flow_port));
            sender->SetAttribute("PacketSize", UintegerValue(packet_size));
            sender->SetAttribute("NumPackets", UintegerValue(4294967295));
            Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
            rand->SetAttribute("Constant", DoubleValue(interval/1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
            sender->SetAttribute("Interval", PointerValue(rand));

            Ptr<ns3::Node> sink_node = this->nodes.Get(sink_node_id);
            sink_node->AddApplication(receiver);
            receiver->SetStartTime(Seconds(0.0));
            receiver->SetAttribute("Port", UintegerValue(mission_flow_port));
        }

        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of APPLICATION module");

        /* **************** STATS MODULE **************** */

        std::string experiment = config["experience_name"].as<std::string>();
        std::string strategy = "VAT flocking";
        std::string input = config_file_path;
        std::string runID = config["run_id"].as<std::string>();

        DataCollector data;
        data.DescribeRun(experiment, strategy, input, runID);

        // Add any information we wish to record about this run.
        data.AddMetadata("author", "tbalaguer");

        // Create a counter for the number of Packets sent WITHIN THE TARGETS
        this->missionTotalTx = CreateObject<PacketCounterCalculator>();
        this->missionTotalTx->SetKey("mission_sent_packets_within_target");
        this->missionTotalTx->SetContext("mission flow node[" + std::to_string(source_node_id) + "]");
        data.AddDataCalculator(this->missionTotalTx);

        // Create a counter for the number of Packets received WITHIN THE TARGETS
        this->missionTotalRx = CreateObject<PacketCounterCalculator>();
        this->missionTotalRx->SetKey("mission_received_packets_within_target");
        this->missionTotalRx->SetContext("mission flow node[" + std::to_string(sink_node_id) + "]");
        data.AddDataCalculator(this->missionTotalRx);

        // Create a statitics object for the delay of the packets
        this->missionDelay = CreateObject<TimeMinMaxAvgTotalCalculator>();
        this->missionDelay->SetKey("mission_packet_delay");
        this->missionDelay->SetContext("mission flow");
        data.AddDataCalculator(this->missionDelay);

        if (mission_flow)
        {
            sender->TraceConnectWithoutContext("Tx", MakeCallback(&Ns3Simulation::mission_flow_sender_clbk, this));
            receiver->TraceConnectWithoutContext("Rx", MakeCallback(&Ns3Simulation::mission_flow_receiver_clbk, this));
        }

        // Create a counter for the number of Packets received by each 'navigation flow' server
        for (int i = 0; i < numNodes; i++)
        {
            Ptr<PacketCounterCalculator> navTotalRx = CreateObject<PacketCounterCalculator>();
            navTotalRx->SetKey("nav_received_packets");
            navTotalRx->SetContext("nav flow node[" + std::to_string(i) + "]");
            this->navTotalRxVector.push_back(navTotalRx);
            data.AddDataCalculator(this->navTotalRxVector[i]);
        }

        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&Ns3Simulation::wifi_phy_tx_clbk, this));

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

                // Initialize empty protobuf message type [NetworkUpdate]
                network_update_proto::NetworkUpdate NetworkUpdate_msg;
                // Transform the message received from the UDS socket [string] -> [protobuf]
                NetworkUpdate_msg.ParseFromString(received_data);

                this->targets_reached = NetworkUpdate_msg.targets_reached();

                // Read the "physical" information transmitted by the NetworkCoordinator, and update the node's positions
                // Also verifies that the number of nodes sent by the NetworkCoordinator corresponds to the number of existing nodes in NS-3
                rclcpp::Clock clock;
                robots_positions_proto::RobotsPositions robots_positions_msg;

                // if (currTime % Seconds(0.1) == Time(0)){
                //     uint64_t bytes_received_this_iteration = mission_server->GetReceived()*packet_size - bytes_received;
                //     bytes_received = mission_server->GetReceived()*packet_size;
                //     RCLCPP_INFO(this->get_logger(), "Mission flow throughput: %f Mbps", (float)(bytes_received_this_iteration * 10 / 1000000.0));
                // }

                if (!NetworkUpdate_msg.robots_positions().empty())
                {
                    RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                    robots_positions_msg.ParseFromString(gzip_decompress(NetworkUpdate_msg.robots_positions()));

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
                            pos.z = 10.0;
                            SetNodePosition(this->nodes.Get(i), pos);
                        }
                    }
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
                }

                // this->updateNeighborsPathloss();

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

                // Create an object giving the neighborhood of each node, based on the packets received by their UDP server, neighbors lifetime is 1 second.
                std::map<uint32_t, std::map<uint32_t, double>> neighbors = create_neighbors(neighbor_timeout_value);

                std::string response = gzip_compress(generate_response(NetworkUpdate_msg, neighbors, this->max_neighbors));

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

        std::cout << "Simulation finished." << std::endl;

        Ptr<DataOutputInterface> output = CreateObject<SqliteDataOutput>();

        if (output)
        {
            output->SetFilePrefix(experiment);
            output->Output(data);
        }

        // int k = 0;
        // for (auto s = servers.Begin(); s != servers.End(); ++s)
        // {
        //     Ptr<UdpServer> server = (*s)->GetObject<UdpServer>();
        //     std::cout << "Position server " << k << " received " << server->GetReceived() * nav_flow_packet_size << " bytes." << std::endl;
        //     k++;
        // }
        // std::cout << std::endl;

        if (mission_flow)
        {
            std::cout << "Mission source sent " << sender->GetSent() << " packets" << std::endl;
            std::cout << "Mission sink received " << receiver->GetReceived() << " packets ( " << (float)(100.0 * receiver->GetReceived() / (float)sender->GetSent()) << "% )" << std::endl;
        }

        Simulator::Destroy();

        exit(EXIT_SUCCESS);
    }

private:
    NodeContainer nodes;
    bool targets_reached;
    std::map<uint32_t, std::map<uint32_t, Time>> neigh_last_received;
    std::map<uint32_t, std::map<uint32_t, double>> neigh_pathloss;
    int max_neighbors;

    ordered_neighbors_proto::OrderedNeighborsList ordered_neighbors_list_msg;

    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    std::map<uint32_t, std::map<uint32_t, double>> create_neighbors(Time timeout);
    void server_receive_clbk(std::string context, const Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress);
    void client_send_clbk(std::string context, Ptr<const Packet> packet);
    void mission_flow_receiver_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk(Ptr<const Packet> packet);
    void wifi_phy_tx_clbk(Ptr<const Packet> packet);

    void updateNeighborsPathloss();

    // Stats objects
    Ptr<PacketCounterCalculator> missionTotalRx;
    Ptr<PacketCounterCalculator> missionTotalTx;
    Ptr<TimeMinMaxAvgTotalCalculator> missionDelay;
    std::vector<Ptr<PacketCounterCalculator>> navTotalRxVector;
    std::vector<Ptr<PacketCounterCalculator>> navTotalTxVector;
    Ptr<PacketCounterCalculator> navEffectiveTx;

    // Propagation model
    Ptr<PropagationLossModel> m_propagationLossModel;

    // saving of computation time
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;

    std::string ros_ws_path;
};

void Ns3Simulation::SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t txId = txPhy->GetDevice()->GetNode()->GetId();
    uint32_t rxId = rxPhy->GetDevice()->GetNode()->GetId();
    // std::cout << "Tx: " << txId << " Rx: " << rxId << " Loss: " << lossDb << std::endl;
    this->neigh_pathloss[txId][rxId] = lossDb;
}

void Ns3Simulation::server_receive_clbk(std::string context, const Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress)
{
    uint32_t nodeId = std::stoi(context);
    uint32_t txId = InetSocketAddress::ConvertFrom(srcAddress).GetIpv4().CombineMask("0.0.0.255").Get() - 1;
    Time now = Simulator::Now();
    RCLCPP_DEBUG(this->get_logger(), "UDP (flocking) Server %d received a packet from %d at time %f", nodeId, txId, now.ToDouble(Time::Unit::S));
    this->neigh_last_received[nodeId][txId] = now;
    this->navTotalRxVector[nodeId]->PacketUpdate("", packet);

    // for(auto x : this->neigh_last_received)
    // {
    //     for(auto y : x.second)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Node %d -> %d : %f", x.first, y.first, y.second.ToDouble(Time::Unit::US));
    //     }
    // }
    // std::cout << std::endl;
}

void Ns3Simulation::client_send_clbk(std::string context, Ptr<const Packet> packet)
{
    uint32_t nodeId = std::stoi(context);
    this->navTotalTxVector[nodeId]->PacketUpdate("", packet);
    // add a tag to the packet
    FlowIdTag nav_flow(1);
    packet->AddPacketTag(nav_flow);
}

void Ns3Simulation::wifi_phy_tx_clbk(Ptr<const Packet> packet)
{
    FlowIdTag nav_flow(1);
    if (packet->PeekPacketTag(nav_flow))
    {
        this->navEffectiveTx->PacketUpdate("", packet);
        RCLCPP_DEBUG(this->get_logger(), "WifiPhy sending a packet for nav flow !");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "WifiPhy sending a packet without the tag...");
    }
}

std::map<uint32_t, std::map<uint32_t, double>>
Ns3Simulation::create_neighbors(Time timeout)
{
    std::map<uint32_t, std::map<uint32_t, double>> neighbors;
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < this->nodes.GetN(); j++)
        {
            if (i != j && this->neigh_last_received.find(i) != this->neigh_last_received.end())
            {
                if (this->neigh_last_received[i].find(j) != this->neigh_last_received[i].end())
                {
                    Time now = Simulator::Now();
                    Time last_received = this->neigh_last_received[i][j];
                    if (now - last_received < timeout)
                    {
                        // agent j is a potential neighbor of agent i

                        // restrict neighborhood to be "chain-like"
                        if (j == i - 1 || j == i + 1)
                        {
                            neighbors[i][j] = -this->neigh_pathloss[i][j]; // Yes, we assume here that neigh_last_received and neigh_pathloss have same keys at all time
                        }
                    }
                }
            }
        }
    }
    return neighbors;
}

void Ns3Simulation::mission_flow_receiver_clbk(Ptr<const Packet> packet)
{
    if (this->targets_reached)
    {
        this->missionTotalRx->PacketUpdate("", packet);
        RCLCPP_INFO(this->get_logger(), "Mission sink received a packet and both source and sink are in targets !");

        myTimestampTag timestamp;
        if (packet->FindFirstMatchingByteTag(timestamp))
        {
            Time tx = timestamp.GetTimestamp();
            this->missionDelay->Update(Simulator::Now() - tx);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Packet received without timestamp");
        }
    }
}

void Ns3Simulation::mission_flow_sender_clbk(Ptr<const Packet> packet)
{
    myTimestampTag timestamp;
    timestamp.SetTimestamp(Simulator::Now());
    packet->AddByteTag(timestamp);

    if (this->targets_reached)
    {
        this->missionTotalTx->PacketUpdate("", packet);
    }
}

void Ns3Simulation::updateNeighborsPathloss()
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
                this->neigh_pathloss[i][j] = -rxPow; // it's not really a pathloss but it's the same with a constant difference
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
    rclcpp::spin(std::make_shared<Ns3Simulation>());
    rclcpp::shutdown();
    return 0;
}

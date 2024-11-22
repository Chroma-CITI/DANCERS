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
#include "ns3/tap-bridge-module.h"
#include "ns3/csma-module.h"

#include "protobuf_msgs/network_update.pb.h"
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


class CarouselUseCase : public rclcpp::Node
{
public:
    CarouselUseCase() : Node("adhoc_chain_flocking")
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

        if (!cosim_mode)
        {
            // we interact with the "real world" through tap bridges, so we need to use the real-time implementation of the simulator
            GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
            GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
        }

        Time step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        Time currTime = MicroSeconds(0);                                       // us
        Time simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)
        int numNodes = config["robots_number"].as<int>();

        std::string wifiType = config["wifi_type"].as<std::string>();
        std::string errorModelType = config["error_model_type"].as<std::string>();
        std::string propagation_loss_model = config["propagation_loss_model"].as<std::string>();
        std::string phyMode(config["phy_mode"].as<std::string>()); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
        double frequency = 2.4e9;                                  // operating frequency in Hz

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
        YansWifiPhyHelper wifiPhy;
        YansWifiChannelHelper wifiChannel;
        wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                        "Exponent", DoubleValue(3.0));
        wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        wifiPhy.SetChannel(wifiChannel.Create());

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211g);
        WifiMacHelper mac;

        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode));
        // wifi.SetRemoteStationManager("ns3::IdealWifiManager");

        NetDeviceContainer devices;

        mac.SetType("ns3::AdhocWifiMac");

        devices = wifi.Install(wifiPhy, mac, nodes);

        Config::Set(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
            BooleanValue(true)
        );
        
        // // The NetDevices on which we will install WiFi
        // NetDeviceContainer devices;

        // WifiHelper wifi;
        // wifi.SetStandard(WIFI_STANDARD_80211g);
        // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode));

        // // MAC layer
        // WifiMacHelper mac;
        // mac.SetType("ns3::AdhocWifiMac");

        // // PHY layer (we support two WifiPhy types: YansWifiPhy and SpectrumWifiPhy)
        // SpectrumWifiPhyHelper spectrumWifiPhy;
        // YansWifiPhyHelper yansWifiPhy;

        // if (wifiType == "YansWifiPhy")
        // {
        //     YansWifiChannelHelper yansChannel;
        //     if (propagation_loss_model == "LogDistancePropagationLossModel")
        //     {
        //         yansChannel = YansWifiChannelHelper::Default();
        //         this->m_propagationLossModel = CreateObject<LogDistancePropagationLossModel>();
        //     }
        //     else if (propagation_loss_model == "HybridBuildingsPropagationLossModel")
        //     {
        //         this->m_propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
        //         this->m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));     // Default 2.4e9
        //         this->m_propagationLossModel->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0)); // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
        //         this->m_propagationLossModel->SetAttribute("ShadowSigmaIndoor", DoubleValue(8.0));   // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
        //         this->m_propagationLossModel->SetAttribute("ShadowSigmaOutdoor", DoubleValue(7.0));  // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
        //         yansChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
        //                                    "Frequency",            // Additional loss for each internal wall [dB]
        //                                    DoubleValue(frequency), // Default 2.4e9
        //                                    "ShadowSigmaExtWalls",  // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
        //                                    DoubleValue(5.0),       // Default 5
        //                                    "ShadowSigmaIndoor",    // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
        //                                    DoubleValue(8.0),       // Default 8
        //                                    "ShadowSigmaOutdoor",   // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
        //                                    DoubleValue(7.0));
        //         yansChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        //     }

        //     yansWifiPhy.SetChannel(yansChannel.Create());
        //     yansWifiPhy.Set("TxPowerStart", DoubleValue(18));
        //     yansWifiPhy.Set("TxPowerEnd", DoubleValue(18));

        //     devices = wifi.Install(yansWifiPhy, mac, this->nodes);

        // }
        // else if (wifiType == "SpectrumWifiPhy")
        // {
        //     Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

        //     // create the channel condition model
        //     Ptr<ChannelConditionModel> m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
        //     m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

        //     // create the propagation loss model and add it to the channel condition
        //     this->m_propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
        //     m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
        //     m_propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(true));
        //     m_propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));
        //     spectrumChannel->AddPropagationLossModel(m_propagationLossModel);

        //     // Create the delay model and add it to the channel condition
        //     Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        //     spectrumChannel->SetPropagationDelayModel(delayModel);

        //     spectrumWifiPhy.SetChannel(spectrumChannel);
        //     spectrumWifiPhy.SetErrorRateModel(errorModelType);
        //     spectrumWifiPhy.Set("TxPowerStart", DoubleValue(16)); // dBm  (1.26 mW)
        //     spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(16));

        //     // Connect a callback that will save the pathloss value in an attribute every time it is calculated
        //     spectrumChannel->TraceConnectWithoutContext("PathLoss", MakeCallback(&CarouselUseCase::SpectrumPathLossTrace, this));

        //     devices = wifi.Install(spectrumWifiPhy, mac, this->nodes);
        // }
        // else
        // {
        //     RCLCPP_FATAL(this->get_logger(), "Unsupported WiFi type %s", wifiType.c_str());
        //     exit(EXIT_FAILURE);
        // }

        // Config::Set(
        //     "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        //     BooleanValue(true));

        // RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of WIFI module");

        /* **************** IP / ROUTING MODULE **************** */
        // InternetStackHelper internet;

        // /** For carousel experiment, do not implement routing **/

        // // Add AODV routing
        // // Ipv4ListRoutingHelper ipv4List;
        // // AodvHelper aodv;
        // // ipv4List.Add(aodv, 100);
        // // internet.SetRoutingHelper(ipv4List);

        // // Add OLSR routing
        // // Ipv4ListRoutingHelper ipv4List;
        // // OlsrHelper olsr;
        // // olsr.Set("HelloInterval", TimeValue(Seconds(0.1)));
        // // ipv4List.Add(olsr, 100);
        // // internet.SetRoutingHelper(ipv4List);
        
        // // Add DSR routing (must be done after internet stack installation)
        // // DsrHelper dsr;
        // // DsrMainHelper dsrMain;
        // // dsrMain.Install(dsr, this->nodes);

        // // Add DSDV routing
        // // DsdvHelper dsdv;
        // // dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(15)));
        // // dsdv.Set("SettlingTime", TimeValue(Seconds(6)));
        // // internet.SetRoutingHelper(dsdv);

        // // Actually install the internet stack on all nodes
        // internet.Install(this->nodes);

        // // Assign IP addresses to the net devices (10.0.0.1, 10.0.0.2, etc.)
        // Ipv4AddressHelper addressAdhoc;
        // addressAdhoc.SetBase("10.0.0.0", "255.255.255.0");
        // Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign(devices);

        // RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of IP/ROUTING module");

        /* **************** APPLICATION MODULE **************** */

        // In the carousel experiment, we use the robot virtualization variation. It enables to use "real" network packets, generated outside of ns-3. Thus, we don't use the application module.

        // **************** TAP-BRIDGE MODULE ****************

        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue("UseLocal"));
        char buffer[10];
        for (uint32_t i = 0; i < numNodes; i++)
        {
            sprintf(buffer, "wifi_tap%d", i + 1);           // The tap interfaces must be created beforehand with the right names
            tapBridge.SetAttribute("DeviceName", StringValue(buffer));
            tapBridge.Install(nodes.Get(i), devices.Get(i));
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

                // Initialize empty protobuf message type [NetworkUpdate]
                network_update_proto::NetworkUpdate NetworkUpdate_msg;
                // Transform the message received from the UDS socket [string] -> [protobuf]
                NetworkUpdate_msg.ParseFromString(received_data);

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

                std::string response = gzip_compress(generate_response(NetworkUpdate_msg));

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

        Simulator::Destroy();

        exit(EXIT_SUCCESS);
    }

private:
    NodeContainer nodes;
    std::string m_ros_ws_path;

    std::map<uint32_t, std::map<uint32_t, double>> pathlosses;

    Ptr<PropagationLossModel> m_propagationLossModel;

    // ns3 Trace callbacks
    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);

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
    std::string generate_response(network_update_proto::NetworkUpdate &NetworkUpdate_msg);
};

void CarouselUseCase::SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t txId = txPhy->GetDevice()->GetNode()->GetId();
    uint32_t rxId = rxPhy->GetDevice()->GetNode()->GetId();
    this->pathlosses[txId][rxId] = -lossDb;
    // std::cout << "Tx: " << txId << " Rx: " << rxId << " Loss: " << lossDb << std::endl;
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
CarouselUseCase::generate_response(network_update_proto::NetworkUpdate &NetworkUpdate_msg)
{
    // Change message type to "END"
    NetworkUpdate_msg.set_msg_type(network_update_proto::NetworkUpdate::END);
    std::string str_response;
    NetworkUpdate_msg.SerializeToString(&str_response);

    return str_response;
}


/**
 * \brief The main function, spins the ROS2 Node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarouselUseCase>());
    rclcpp::shutdown();
    return 0;
}

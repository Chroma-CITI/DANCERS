#include <rclcpp/rclcpp.hpp>


#include <yaml-cpp/yaml.h>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include <ns3/propagation-loss-model.h>
#include "ns3/udp-client-server-helper.h"

// Boost libraries used for:
#include <boost/asio.hpp>           //!< UDS sockets
#include <boost/filesystem.hpp>     //!< Filesystem manipulations

// Custom libraries used for:
#include <udp_tcp_socket.hpp>       //!< Custom UDS/UDP/TCP sockets
#include <util.hpp>                 //!< Stuff that is, well, useful
#include <time_probe.hpp>           //!< Time probe that counts either wall clock time or CPU cycles

// Protobuf messages
#include "protobuf_msgs/dancers_update.pb.h"
#include "protobuf_msgs/pose_vector.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

using namespace ns3;

/**
 * @brief The ns-3 ROS2 Node, part of the DANCERS co-simulator
 *
 * This class is a ROS2 node that holds the network simulation for mobile communicating nodes, using the ns-3 simulator.
 */
class Ns3Sim : public rclcpp::Node
{
public:
    Ns3Sim() : Node("ns3_sim_standard_wifi_network")
    {
        RCLCPP_INFO(this->get_logger(), "Network Simulation Node Created");

        // ========================= PARAMETER READING AND LOG FILE INITIALIZATION =========================
        // Declare two parameters for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        param_desc.description = "Executes in co-simulation or ns-3 only.";
        this->declare_parameter("cosim_mode", true, param_desc);

        // Fetch the parameter path to the config file using a ros2 parameter
        this->config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        this->cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

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
        this->config = YAML::LoadFile(config_file_path);

        // ========================= OUTPUT FILES =========================
        this->save_compute_time = config["save_compute_time"].as<bool>();

        this->experience_name = config["experience_name"].as<std::string>();
        this->run_id = config["run_id"].as<int>();

        if (this->save_compute_time)
        {

            // Create a folder based on the experience name, if not existant already
            if (boost::filesystem::create_directories(this->m_ros_ws_path + "/data/" + this->experience_name))
            {
                RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", this->experience_name.c_str());
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
                temp_path = this->m_ros_ws_path + "/data/" + this->experience_name + "/network" + std::to_string(i) + ".csv";
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

        this->step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        this->currTime = MicroSeconds(0);                                       // us
        this->simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)

        this->ns3_config = this->ReadNs3Config(config);

        this->ConfigureNs3(this->ns3_config);

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&Ns3Sim::Loop, this);

    }

    ~Ns3Sim()
    {
        if (loop_thread_.joinable())
        {
            RCLCPP_INFO(this->get_logger(), "Ns3Sim Node Destroyed");
            loop_thread_.join();
        }
    }


private:    
    // Simulation configuration
    YAML::Node config;
    ns3_configuration_t ns3_config;
    std::string experience_name;
    int run_id;
    bool cosim_mode;

    // Usefull paths
    std::string config_file_path;
    std::string m_ros_ws_path;


    // Thread running the co-simulation Loop
    std::thread loop_thread_;

    // Co-simulation related variables
    Time step_size; //!< The length of a ns-3 "step". An "iteration" can contain multiple "steps".
    Time currTime; //!< The current time of the simulation
    Time simEndTime; //!< The time at which the simulation should end

    NodeContainer nodes; //!< ns-3 nodes container. This is our access to reconfigure the network (add application, etc.) on the go, during the simulation

    // Methods
    ns3_configuration_t ReadNs3Config(const YAML::Node& config);
    void ConfigureNs3(const ns3_configuration_t& ns3_config);
    void Loop();

    // Callbacks
    void udp_server_receive_clbk(Ptr<const Packet> packet);
    void udp_client_send_clbk(Ptr<const Packet> packet);

    // Co-simulation specific methods
    std::string generateResponseProtobuf();
    std::string generateNeighborsMsg();

    // Save compute time (optional)
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;



};

/**
 * @brief Read the ns3 configuration from the config file
 *
 * Read the YAML configuration file and translate it in a struct object, I find it easier to handle
 */
ns3_configuration_t Ns3Sim::ReadNs3Config(const YAML::Node &config)
{
    ns3_configuration_t ns3_config;

    ns3_config.seed = config["seed"].as<int>();
    ns3_config.num_nodes = config["robots_number"].as<uint32_t>();
    ns3_config.wifi_phy_mode = config["phy_mode"].as<std::string>(); // Define a "Phy mode" that will be given to the WifiRemoteStationManager

    return ns3_config;
}

/**
 * @brief Configures the network in the ns-3 fashion
 * 
 * This is where the ns-3 nodes are created and configured. It holds most of the network configuration and options and is called once at the launch of the simulation.
 */
void Ns3Sim::ConfigureNs3(const ns3_configuration_t& ns3_config)
{

    // Set the seed for the random number generator
    SeedManager::SetRun(ns3_config.seed);

    // Create the nodes
    this->nodes.Create(ns3_config.num_nodes);

    // **************** MOBILITY MODULE ****************

    // For all the nodes, create a ConstantPositionMobilityModel to initialize their position.
    // Assign different positions for each robot as these values will be overwritten at first step of simulation anyway
    for (uint32_t i = 0; i < ns3_config.num_nodes; i++)
    {
        Ptr<MobilityModel> nodeMob;
        nodeMob = CreateObject<ConstantPositionMobilityModel>();
        nodeMob->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(i, i, 0.5));
        this->nodes.Get(i)->AggregateObject(nodeMob);
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of MOBILITY module");

    // **************** WIFI MODULE ****************

    // The NetDevices on which we will install WiFi
    NetDeviceContainer devices;

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(ns3_config.wifi_phy_mode), "ControlMode", StringValue(ns3_config.wifi_phy_mode));
    
    // Fix non-unicast WiFi data rate to be the same as that of unicast
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                        StringValue(ns3_config.wifi_phy_mode));

    // MAC layer
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    YansWifiPhyHelper yansWifiPhy;
    YansWifiChannelHelper yansChannel = YansWifiChannelHelper::Default();
    Ptr<PropagationLossModel> propagationLossModel;

    propagationLossModel = CreateObject<LogDistancePropagationLossModel>();
    yansWifiPhy.SetChannel(yansChannel.Create());
    devices = wifi.Install(yansWifiPhy, mac, this->nodes);

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of WIFI module");

    /* **************** IP / ROUTING MODULE **************** */
    InternetStackHelper internet;

    // Actually install the internet stack on all nodes
    internet.Install(this->nodes);

    // Assign IP addresses to the net devices
    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign(devices);
    Address serverAddress = Address(adhocInterfaces.GetAddress(1));

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of IP/ROUTING module");

    /* **************** APPLICATION MODULE **************** */

    if (this->nodes.GetN() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "This simulation requires at least 2 nodes, aborting.");
        exit(EXIT_FAILURE);
    }

    RCLCPP_DEBUG(this->get_logger(), "Create UdpServer application on node 1.");
    uint16_t port = 4000;
    UdpServerHelper server(port);
    ApplicationContainer apps = server.Install(this->nodes.Get(1));
    apps.Start(Seconds(1.0));
    apps.Stop(this->simEndTime);

    if (apps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&Ns3Sim::udp_server_receive_clbk, this)))
    {
        RCLCPP_DEBUG(this->get_logger(), "Connected receiver to UDP server.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect receiver to UDP server.");
    }


    RCLCPP_DEBUG(this->get_logger(), "Create UdpClient application on node 0 to send to node 1.");
    uint32_t MaxPacketSize = 1024;
    Time interPacketInterval = Seconds(1.0);
    uint32_t maxPacketCount = 320;
    UdpClientHelper client(serverAddress, port);
    client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
    client.SetAttribute("Interval", TimeValue(interPacketInterval));
    client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
    apps = client.Install(this->nodes.Get(0));
    apps.Start(Seconds(2.0));
    apps.Stop(this->simEndTime);

    if (apps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&Ns3Sim::udp_client_send_clbk, this)))
    {
        RCLCPP_DEBUG(this->get_logger(), "Connected sender callback to UDP client.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect sender callback to UDP client.");
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of APPLICATION module");

}

/**
 * @brief Callback for the reception of a packet at the application layer
 */
void Ns3Sim::udp_server_receive_clbk(Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "UDP Server received a packet !");
}

void Ns3Sim::udp_client_send_clbk(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "UDP Client sent a packet.");
}


void Ns3Sim::Loop()
{
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
            dancers_update_proto::DancersUpdate physics_update_msg;
            // Transform the message received from the UDS socket [string] -> [protobuf]
            physics_update_msg.ParseFromString(received_data);

            // Read the "physical" information transmitted by the NetworkCoordinator, and update the node's positions
            // Also verifies that the number of nodes sent by the NetworkCoordinator corresponds to the number of existing nodes in NS-3
            rclcpp::Clock clock;
            dancers_update_proto::PoseVector robots_positions_msg;

            if (!physics_update_msg.payload().empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                robots_positions_msg.ParseFromString(gzip_decompress(physics_update_msg.payload()));

                // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                // Then, update the node's positions (orientation is ignored for now)
                if (this->nodes.GetN() != (uint32_t)robots_positions_msg.pose_size())
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(),
                                            clock,
                                            1000, // ms
                                            "Network simulator received position information of %i robots but NS-3 has %u nodes.",
                                            robots_positions_msg.pose_size(),
                                            this->nodes.GetN());
                }
                else
                {
                    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
                    {
                        Vector pos;
                        pos.x = robots_positions_msg.pose(i).x();
                        pos.y = robots_positions_msg.pose(i).y();
                        pos.z = robots_positions_msg.pose(i).z();

                        // ns-s don't like negative and 0 positions
                        if (pos.z <= 0)
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

            std::string response = gzip_compress(generateResponseProtobuf());

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

/**
 * @brief Generates an ordered_neighbors_msg containing the neighbors of each node, sorted by pathloss, and serializes it
 *
 * @returns A string-serialized version of the "ordered_neighbors" protobuf message
 */
std::string
Ns3Sim::generateNeighborsMsg()
{
    dancers_update_proto::OrderedNeighborsList ordered_neighbors_msg;

    // There are no notion of neighbors in this simulation, so we just return an empty message

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
Ns3Sim::generateResponseProtobuf()
{
    // Change message type to "END"
    dancers_update_proto::DancersUpdate network_update_msg;
    network_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);
    network_update_msg.set_payload(gzip_compress(generateNeighborsMsg()));
    std::string str_response;
    network_update_msg.SerializeToString(&str_response);

    return str_response;
}


/**
 * \brief The main function, spins the ROS2 Node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ns3Sim>());
    rclcpp::shutdown();
    return 0;
}

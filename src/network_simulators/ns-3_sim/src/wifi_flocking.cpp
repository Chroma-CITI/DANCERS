#include <rclcpp/rclcpp.hpp>

#include <stack>
#include <map>
#include <random>

// YAML
#include <yaml-cpp/yaml.h>

// Useful home-made tools
#include <util.hpp>
#include <time_probe.hpp>
#include <udp_tcp_socket.hpp>

// Boost
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

// ns-3 modules
#include "ns3/core-module.h"
#include "ns3/buildings-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"

// Custom ns-3 applications
#include <wifi-application.h>
#include <flocking-application.h>

// Protobuf messages
#include "protobuf_msgs/dancers_update.pb.h"
#include "protobuf_msgs/pose_vector.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"
#include "protobuf_msgs/velocity_heading_vector.pb.h"

// ROS2 messages
#include <dancers_msgs/msg/target.hpp>
#include <dancers_msgs/msg/agent_struct_array.hpp>
#include <dancers_msgs/srv/get_agent_velocities.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace ns3;
using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief The ns-3 ROS2 Node, part of the DANCERS co-simulator
 *
 * This class is a ROS2 node that holds the network simulation for mobile communicating nodes, using the ns-3 simulator.
 */
class Ns3Sim : public rclcpp::Node
{
public:
    Ns3Sim() : Node("wifi_flocking")
    {
        RCLCPP_INFO(this->get_logger(), "wifi_flocking Node Created");

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
            this->ros_ws_path_ = getenv("ROS_WS");
        }

        // Parse the config file
        this->config = YAML::LoadFile(config_file_path);

        this->experience_name = config["experience_name"].as<std::string>();
        this->run_id = config["run_id"].as<int>();
        this->mode = config["cosimulation_mode"].as<std::string>();
        this->publish_agent_structs = config["publish_agent_structs"].as<bool>();
        this->print_simulation_advancement_ = config["ns3_print_simulation_advancement"].as<bool>();

        this->SetupOutputFiles(config);

        /* ----------- ROS2 Subscribers ----------- */
        this->targets_sub_ = this->create_subscription<dancers_msgs::msg::Target>("targets", 10, std::bind(&Ns3Sim::targets_clbk, this, _1));
        this->spawn_uav_ = this->create_subscription<dancers_msgs::msg::AgentStruct>("spawn_uav", 10, std::bind(&Ns3Sim::spawn_uav_clbk, this, _1));


        /* ----------- ROS2 Publishers ----------- */
        this->network_mission_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_mission_links", 10);
        this->network_potential_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_potential_links", 10);
        this->network_idle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_idle_links", 10);
        if (this->publish_agent_structs)
        {
            this->agent_structs_pub_ = this->create_publisher<dancers_msgs::msg::AgentStructArray>("agent_structs", 10);
        }



        // ========================= NS3 =========================

        this->ns3_config = this->ReadNs3Config(config);

        this->ConfigureNs3(this->ns3_config);

        this->InitAgents(config);

        this->step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        this->currTime = MicroSeconds(0);                                       // us
        this->simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)
        int numNodes = config["robots_number"].as<int>();
        this->pseudoRoutingAlgo = config["pseudo_routing_algorithm"].as<std::string>();
        this->random_generator = std::mt19937(ns3_config.seed);

        this->broadcast_window_size = Seconds(config["minimize_error_rate"]["broadcast_window_size"].as<double>());

        // ========================= Pseudo-routing parameters =========================
        if (pseudoRoutingAlgo == "shortest_dist")
        {
            this->shortest_dist_r1 = config["shortest_dist"]["distance_threshold"].as<double>();
            this->shortest_dist_c1 = config["shortest_dist"]["constant_before_threshold"].as<double>();
        }
        else if (pseudoRoutingAlgo == "hybrid_dist_error_rate")
        {
            this->hybrid_r1 = config["hybrid_dist_error_rate"]["distance_threshold"].as<double>();
            this->hybrid_c1 = config["hybrid_dist_error_rate"]["constant_before_threshold"].as<double>();
            this->hybrid_e1 = config["hybrid_dist_error_rate"]["error_rate_exponent"].as<double>();
            this->hybrid_k1 = config["hybrid_dist_error_rate"]["distance_gain"].as<double>();
            this->hybrid_k2 = config["hybrid_dist_error_rate"]["error_rate_gain"].as<double>();
        }

        /* ----------- Service client ----------- */
        command_client_ = this->create_client<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities");

        // Colours are fun: "\x1b[32m" enables red color ; "\x1b[0m" set default back
        RCLCPP_INFO(this->get_logger(), "\x1b[32mns-3 configuration finished. Starting simulation !\x1b[0m");

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&Ns3Sim::Loop, this);
    }

    ~Ns3Sim()
    {
        if (loop_thread_.joinable())
        {
            loop_thread_.join();
            RCLCPP_INFO(this->get_logger(), "Ns3Sim Node Destroyed");
        }
    }

private:
    NodeContainer nodes; //!< ns-3 nodes container. This is our access to reconfigure the network (add application, etc.) on the go, during the simulation

    std::string config_file_path;
    std::string ros_ws_path_;
    ns3_configuration_t ns3_config;
    std::string mode;

    std::unique_ptr<WifiHelper> wifiHelper;
    std::unique_ptr<WifiMacHelper> wifiMacHelper;
    std::unique_ptr<WifiPhyHelper> wifiPhyHelper;
    std::unique_ptr<InternetStackHelper> internetHelper;
    std::unique_ptr<Ipv4AddressHelper> ipv4AddressHelper;

    Ptr<FlockingBroadcaster> flocking_broadcaster;
    Ptr<FlockingReceiver> flocking_receiver;
    Ptr<Sender> mission_application;
    Ptr<Receiver> mission_receiver;

    std::mt19937 random_generator;

    std::map<uint32_t, std::map<uint32_t, double>> pathlosses;
    std::map<uint32_t, std::map<uint32_t, double>> snrs;
    std::map<uint32_t, std::map<uint32_t, std::vector<Time>>> broadcast_received_packets;
    std::map<Mac48Address, uint32_t> mac_to_id;

    std::vector<agent_t> agents;

    // Co-simulation related variables
    Time step_size;  //!< The length of a ns-3 "step". An "iteration" can contain multiple "steps".
    Time currTime;   //!< The current time of the simulation
    Time simEndTime; //!< The time at which the simulation should end

    Time broadcast_window_size;
    std::string pseudoRoutingAlgo;

    double shortest_dist_r1;
    double shortest_dist_c1;

    double hybrid_r1;
    double hybrid_c1;
    double hybrid_e1;
    double hybrid_k1;
    double hybrid_k2;

    YAML::Node config;

    bool cosim_mode;
    bool print_simulation_advancement_;

    // Thread running the co-simulation Loop
    std::thread loop_thread_;

    bool stats_enabled;
    bool mission_flow;
    bool publish_agent_structs;


    std::string experience_name;
    int run_id;
    DataCollector data;

    /* Subscribers */
    rclcpp::Subscription<dancers_msgs::msg::Target>::SharedPtr targets_sub_;
    void targets_clbk(dancers_msgs::msg::Target msg);

    rclcpp::Subscription<dancers_msgs::msg::AgentStruct>::SharedPtr spawn_uav_;
    void spawn_uav_clbk(dancers_msgs::msg::AgentStruct msg);


    /* Publishers */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_mission_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_potential_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_idle_pub_;
    rclcpp::Publisher<dancers_msgs::msg::AgentStructArray>::SharedPtr agent_structs_pub_;


    Ptr<PropagationLossModel> m_propagationLossModel;

    bool targets_reached;

    // ns3 Trace callbacks
    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    void PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow);
    void PhyTxEndTrace(std::string context, Ptr<const Packet> packet);
    void PhyRxEndTrace(std::string context, Ptr<const Packet> packet);
    void MacTxTrace(std::string context, Ptr<const Packet> packet);
    void mission_flow_receiver_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk_2(Ptr<const Packet> packet);
    void flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id);
    void flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet);
    void MonitorSnifferRxTrace(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId);

    // save compute time (optional)
    bool save_compute_time;
    std::string computation_time_file_;
    WallTimeProbe probe;

    // save received mission packets (optional)
    bool save_mission_packets;
    std::string mission_packets_file_;

    // Stats attributes (optional)
    Ptr<PacketCounterCalculator> missionTotalRx;
    Ptr<PacketCounterCalculator> missionTotalTx;
    Ptr<TimeMinMaxAvgTotalCalculator> missionDelay;
    std::vector<Ptr<PacketCounterCalculator>> navTotalRxVector;
    std::vector<Ptr<PacketCounterCalculator>> navTotalTxVector;
    Ptr<PacketCounterCalculator> navEffectiveTx;

    /* Service client*/
    rclcpp::Client<dancers_msgs::srv::GetAgentVelocities>::SharedPtr command_client_;

    // co-simulation specific methods
    std::string generateResponseProtobuf();
    std::string generateNeighborsMsg();
    std::string generateCommandsMsg();

    void timeoutNeighbors();
    void updateNeighborsPathloss();
    double costFunction(double dist, double r1, double c1);
    double hybrid_dist_error_rate_cost_function(double dist, double r1, double c1, double error_rate, double e1, double k1, double k2);
    void updateStaticIpv4Routes(const std::vector<int> path);
    void AddNodeToWifiNetwork(Ptr<ns3::Node> node);
    void Loop();

    void SetupOutputFiles(const YAML::Node &config);
    ns3_configuration_t ReadNs3Config(const YAML::Node &config);
    void InitAgents(const YAML::Node &config);
    void ConfigureNs3(const ns3_configuration_t &ns3_config);
    void RunPseudoRoutingAlgorithm(const std::string pseudo_routing_algo_name);
    void UpdateRolesAndNeighbors();
    void UpdateCmds();
    void DisplayNetworkRviz();

    dancers_msgs::msg::AgentStruct agent_t_to_ROS_msg(agent_t agent);
    agent_t ROS_msg_to_agent_t(dancers_msgs::msg::AgentStruct agent);
};

dancers_msgs::msg::AgentStruct Ns3Sim::agent_t_to_ROS_msg(agent_t agent)
{
    dancers_msgs::msg::AgentStruct a;
    a.agent_id = agent.id;
    a.agent_role = static_cast<uint8_t>(agent.role);
    a.state.position.x = agent.position.x();
    a.state.position.y = agent.position.y();
    a.state.position.z = agent.position.z();
    a.state.velocity_heading.velocity.x = agent.velocity.x();
    a.state.velocity_heading.velocity.y = agent.velocity.y();
    a.state.velocity_heading.velocity.z = agent.velocity.z();
    a.heartbeat_received = agent.heartbeat_received;
    a.heartbeat_sent = agent.heartbeat_sent;

    for (const auto &neighbor : agent.neighbors)
    {
        dancers_msgs::msg::Neighbor n;
        n.agent_id = neighbor.second.id;
        n.link_quality = neighbor.second.link_quality;
        n.position.x = neighbor.second.position.x();
        n.position.y = neighbor.second.position.y();
        n.position.z = neighbor.second.position.z();
        n.velocity.x = neighbor.second.velocity.x();
        n.velocity.y = neighbor.second.velocity.y();
        n.velocity.z = neighbor.second.velocity.z();
        n.agent_role = static_cast<uint8_t>(neighbor.second.role);

        a.neighbor_array.neighbors.push_back(n);
    }

    return a;
}

agent_t Ns3Sim::ROS_msg_to_agent_t(dancers_msgs::msg::AgentStruct agent)
{
    agent_t a;
    a.id = agent.agent_id;
    a.role = static_cast<AgentRoleType>(agent.agent_role);
    a.position = Eigen::Vector3d(agent.state.position.x, agent.state.position.y, agent.state.position.z);
    a.velocity = Eigen::Vector3d(agent.state.velocity_heading.velocity.x, agent.state.velocity_heading.velocity.y, agent.state.velocity_heading.velocity.z);
    a.heartbeat_received = agent.heartbeat_received;
    a.heartbeat_sent = agent.heartbeat_sent;
    a.neighbors = std::map<int, NeighborInfo_t>();

    for (const auto &neighbor : agent.neighbor_array.neighbors)
    {
        NeighborInfo_t n;
        n.id = neighbor.agent_id;
        n.link_quality = neighbor.link_quality;
        n.position = Eigen::Vector3d(neighbor.position.x, neighbor.position.y, neighbor.position.z);
        n.velocity = Eigen::Vector3d(neighbor.velocity.x, neighbor.velocity.y, neighbor.velocity.z);
        n.role = static_cast<AgentRoleType>(neighbor.agent_role);

        a.neighbors[neighbor.agent_id] = n;
    }

    return a;
}


/**
 * @brief Create the output files for the simulation
 *
 * In the config file, the user can specify the output files to be created. This function creates them in the $ROS_WS/data/<experiment_name>/
 */
void Ns3Sim::SetupOutputFiles(const YAML::Node &config)
{
    // Create a folder based on the experience name, if not existant already
    if (boost::filesystem::create_directories(this->ros_ws_path_ + "/data/" + this->experience_name))
    {
        RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", this->experience_name.c_str());
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Using existing data folder for this experiment");
    }

    // Find the next available "run_x" folder
    int run_number = 1;
    std::string run_folder;
    do
    {
        run_folder = this->ros_ws_path_ + "/data/" + this->experience_name + "/run_" + std::to_string(run_number);
        run_number++;
    } while (boost::filesystem::exists(run_folder));

    // Create the new "run_x" folder
    if (boost::filesystem::create_directory(run_folder))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Created run folder: %s", run_folder.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create run folder: %s", run_folder.c_str());
    }

    this->save_compute_time = config["save_compute_time"].as<bool>();
    if (this->save_compute_time)
    {
        this->computation_time_file_ = run_folder + "/network_compute_time.csv";
        // initialize the output file with headers
        this->probe = WallTimeProbe(this->computation_time_file_);
    }

    this->save_mission_packets = config["mission_flow"]["save_packets"].as<bool>();
    if (this->save_mission_packets)
    {
        this->mission_packets_file_ = run_folder + "/mission_received_packets.csv";

        // Write headers
        std::ofstream f(this->mission_packets_file_, std::ios::app);
        f << "rcv_time(us),delay(us)" << std::endl;
        f.close();
    }
}

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
    ns3_config.wifi_standard = config["wifi_standard"].as<std::string>();
    ns3_config.wifi_phy_type = config["wifi_type"].as<std::string>();
    ns3_config.wifi_phy_mode = config["phy_mode"].as<std::string>(); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
    ns3_config.error_model = config["error_model_type"].as<std::string>();
    ns3_config.propagation_loss_model = config["propagation_loss_model"].as<std::string>();
    ns3_config.frequency = config["frequency"].as<double>();
    ns3_config.routing_algorithm = config["routing_algorithm"].as<std::string>();
    ns3_config.short_guard_interval_supported = config["short_guard_interval_supported"].as<bool>();
    ns3_config.use_real_routing_algorithm = config["use_real_routing_algorithm"].as<bool>();
    for (auto building : config["buildings"])
    {
        AABB_t b;
        b.x_min = building["x"].as<float>() - (building["size_x"].as<float>() / 2);
        b.x_max = building["x"].as<float>() + (building["size_x"].as<float>() / 2);
        b.y_min = building["y"].as<float>() - (building["size_y"].as<float>() / 2);
        b.y_max = building["y"].as<float>() + (building["size_y"].as<float>() / 2);
        b.z_min = 0.0;
        b.z_max = building["height"].as<float>();
        ns3_config.buildings.push_back(b);
    }
    ns3_config.use_localization_noise = config["use_localization_noise"].as<bool>();
    ns3_config.localization_noise_stddev = config["localization_noise_stddev"].as<double>();

    ns3_config.enable_broadcast_flow = config["broadcast_flow"]["enable"].as<bool>();
    ns3_config.start_broadcast_time = config["broadcast_flow"]["start_time"].as<double>();
    ns3_config.stop_broadcast_time = config["broadcast_flow"]["stop_time"].as<double>();
    ns3_config.broadcast_packet_size = config["broadcast_flow"]["packet_size"].as<uint32_t>();
    ns3_config.broadcast_interval = config["broadcast_flow"]["interval"].as<uint32_t>(); // us
    ns3_config.broadcast_port = config["broadcast_flow"]["port"].as<uint16_t>();
    ns3_config.broadcast_timeout = config["broadcast_flow"]["timeout"].as<uint32_t>();
    ns3_config.broadcast_flow_id = config["broadcast_flow"]["flow_id"].as<uint8_t>();

    ns3_config.enable_mission_flow = config["mission_flow"]["enable"].as<bool>();
    ns3_config.source_robots_ids = config["mission_flow"]["source_robot_ids"].as<std::vector<uint32_t>>();
    ns3_config.sink_robot_id = config["mission_flow"]["sink_robot_id"].as<uint32_t>();
    ns3_config.start_mission_time = config["mission_flow"]["start_time"].as<double>();     // s
    ns3_config.stop_mission_time = config["mission_flow"]["stop_time"].as<double>();       // s
    ns3_config.mission_packet_size = config["mission_flow"]["packet_size"].as<uint32_t>(); // bytes
    ns3_config.mission_interval = config["mission_flow"]["interval"].as<uint32_t>();       // us
    ns3_config.mission_port = config["mission_flow"]["port"].as<uint16_t>();
    ns3_config.mission_flow_id = config["mission_flow"]["flow_id"].as<uint8_t>();
    // ns3_config.mission_timeout = config["mission_flow"]["timeout"].as<uint32_t>();

    ns3_config.enable_stats_module = config["enable_stats"].as<bool>();

    return ns3_config;
}

/**
 * @brief Initialize the agents structs 
 */
void Ns3Sim::InitAgents(const YAML::Node &config)
{

    for (int i = 0; i < this->config["robots_number"].as<int>(); i++)
    {
        dancers_msgs::msg::AgentStruct agent_ros_msg;
        agent_ros_msg.agent_id = i;
        agent_ros_msg.agent_role = dancers_msgs::msg::AgentStruct::AGENT_ROLE_UNDEFINED;
        // This is only for the first iteration. Starting with the second iteration, positions will be updated by the physics simulator
        agent_ros_msg.state.position.x = float(i);
        agent_ros_msg.state.position.y = 0.0;
        agent_ros_msg.state.position.z = 1.0;
        agent_ros_msg.state.velocity_heading.velocity.x = 0.0;
        agent_ros_msg.state.velocity_heading.velocity.y = 0.0;
        agent_ros_msg.state.velocity_heading.velocity.z = 0.0;

        agent_ros_msg.heartbeat_received = 0;
        agent_ros_msg.heartbeat_sent = 0;
        
        this->spawn_uav_clbk(agent_ros_msg);
    }
}

/**
 * @brief Configures the network in the ns-3 fashion
 *
 * This is where the ns-3 nodes are created and configured. It holds most of the network configuration and options and is called once at the launch of the simulation.
 */
void Ns3Sim::ConfigureNs3(const ns3_configuration_t &ns3_config)
{
    // Set the seed for the random number generator
    SeedManager::SetRun(ns3_config.seed);

    // Initialize an empty node container
    this->nodes = NodeContainer();

    // **************** BUILDINGS MODULE ****************
    
    // Create the buildings in the simulation
    for (AABB_t build : ns3_config.buildings)
    {
        Ptr<Building> ns3_building = CreateObject<Building>();
        ns3_building->SetBoundaries(Box(build.x_min, build.x_max, build.y_min, build.y_max, build.z_min, build.z_max));
        ns3_building->SetBuildingType(Building::Office);
        ns3_building->SetExtWallsType(Building::ConcreteWithWindows);
        ns3_building->SetNFloors(1);
        ns3_building->SetNRoomsX(1);
        ns3_building->SetNRoomsY(1);

        RCLCPP_DEBUG(this->get_logger(), "Created a building with corners (%f, %f, %f, %f)", build.x_min, build.y_min, build.x_max, build.y_max);
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of BUILDINGS module");

    // **************** WIFI MODULE ****************

    this->wifiHelper = std::make_unique<WifiHelper>();

    if (ns3_config.wifi_standard == "802.11g")
    {
        this->wifiHelper->SetStandard(WIFI_STANDARD_80211g);
    }
    else if (ns3_config.wifi_standard == "802.11n")
    {
        this->wifiHelper->SetStandard(WIFI_STANDARD_80211n);
    }
    else if (ns3_config.wifi_standard == "802.11ax")
    {
        this->wifiHelper->SetStandard(WIFI_STANDARD_80211ax);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported WiFi standard: %s", ns3_config.wifi_standard.c_str());
        exit(EXIT_FAILURE);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Using WiFi standard: " << ns3_config.wifi_standard.c_str());

    this->wifiHelper->SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(ns3_config.wifi_phy_mode), "ControlMode", StringValue(ns3_config.wifi_phy_mode));

    // Fix non-unicast WiFi data rate to be the same as that of unicast
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                       StringValue(ns3_config.wifi_phy_mode));

    // The "Ht" configuration is for 802.11n
    // Short guard interval = 400ns (vs 800ns otherwise)
    // Config::Set(
        //     "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        //     BooleanValue(ns3_config.short_guard_interval_supported));
    Config::SetDefault("ns3::HtConfiguration::ShortGuardIntervalSupported", BooleanValue(ns3_config.short_guard_interval_supported));

    // MAC layer
    this->wifiMacHelper = std::make_unique<WifiMacHelper>();
    this->wifiMacHelper->SetType("ns3::AdhocWifiMac");

    Ptr<PropagationLossModel> propagationLossModel;
        
    // PHY layer (we support two WifiPhy types: YansWifiPhy and SpectrumWifiPhy)
    if (ns3_config.wifi_phy_type == "YansWifiPhy")
    {
        this->wifiPhyHelper = std::make_unique<YansWifiPhyHelper>();
        // That's a bit hacky...
        std::unique_ptr<YansWifiPhyHelper> yansPhyHelper = std::unique_ptr<YansWifiPhyHelper>(dynamic_cast<YansWifiPhyHelper*>(this->wifiPhyHelper.get()));

        YansWifiChannelHelper yansChannel;

        if (ns3_config.propagation_loss_model == "LogDistancePropagationLossModel")
        {
            yansChannel = YansWifiChannelHelper::Default();
            propagationLossModel = CreateObject<LogDistancePropagationLossModel>();
        }
        else if (ns3_config.propagation_loss_model == "HybridBuildingsPropagationLossModel")
        {
            propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
            propagationLossModel->SetAttribute("Frequency", DoubleValue(ns3_config.frequency)); // Default 2.4e9
            propagationLossModel->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0));        // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
            propagationLossModel->SetAttribute("ShadowSigmaIndoor", DoubleValue(8.0));          // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
            propagationLossModel->SetAttribute("ShadowSigmaOutdoor", DoubleValue(7.0));         // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
            yansChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                           "Frequency",                       // Additional loss for each internal wall [dB]
                                           DoubleValue(ns3_config.frequency), // Default 2.4e9
                                           "ShadowSigmaExtWalls",             // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                                           DoubleValue(5.0),                  // Default 5
                                           "ShadowSigmaIndoor",               // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                                           DoubleValue(8.0),                  // Default 8
                                           "ShadowSigmaOutdoor",              // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                                           DoubleValue(7.0));
            yansChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        }

        yansPhyHelper->SetChannel(yansChannel.Create());
        yansPhyHelper->Set("TxPowerStart", DoubleValue(14));
        yansPhyHelper->Set("TxPowerEnd", DoubleValue(14));
        yansPhyHelper->SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

    }
    else if (ns3_config.wifi_phy_type == "SpectrumWifiPhy")
    {
        this->wifiPhyHelper = std::make_unique<SpectrumWifiPhyHelper>();
        SpectrumWifiPhyHelper* spectrumPhyHelper = dynamic_cast<SpectrumWifiPhyHelper*>(this->wifiPhyHelper.get());

        Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

        // create the channel condition model
        Ptr<ChannelConditionModel> m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
        m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

        // create the propagation loss model and add it to the channel condition
        propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
        propagationLossModel->SetAttribute("Frequency", DoubleValue(ns3_config.frequency));
        propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(false));
        propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));
        spectrumChannel->AddPropagationLossModel(propagationLossModel);

        // Create the delay model and add it to the channel condition
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);

        spectrumPhyHelper->SetChannel(spectrumChannel);
        spectrumPhyHelper->SetErrorRateModel(ns3_config.error_model);
        spectrumPhyHelper->Set("TxPowerStart", DoubleValue(14)); // dBm
        spectrumPhyHelper->Set("TxPowerEnd", DoubleValue(14));
        // Note: There is a unique ns3::WifiPhy attribute, named ChannelSettings, that enables to set channel number, channel width, frequency band and primary20 index for each segment all together.
        // When a parameter is set to 0, the default value is used, depending on the previously set WifiStandard.

        if (ns3_config.frequency == 2.4e9)
            spectrumPhyHelper->Set("ChannelSettings", StringValue("{0, 0, BAND_2_4GHZ, 0}"));
        else if (ns3_config.frequency == 5e9)
            spectrumPhyHelper->Set("ChannelSettings", StringValue("{0, 0, BAND_5GHZ, 0}"));

        spectrumPhyHelper->SetPcapDataLinkType(SpectrumWifiPhyHelper::DLT_IEEE802_11_RADIO);

        // Connect a callback that will save the pathloss value in an attribute every time it is calculated
        spectrumChannel->TraceConnectWithoutContext("PathLoss", MakeCallback(&Ns3Sim::SpectrumPathLossTrace, this));
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(), "Unsupported WiFi type %s", ns3_config.wifi_phy_type.c_str());
        exit(EXIT_FAILURE);
    }

    this->m_propagationLossModel = propagationLossModel;

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of WIFI module");

    /* **************** IP / ROUTING MODULE **************** */
    this->internetHelper = std::make_unique<InternetStackHelper>();

    // There is a great dichotomy between using a routing algorithm properly implemented in ns-3 such as OLSR or AODV, and using "pseudo-routing algorithms"
    // that use global information to form routes (non-realistic simulation)
    if (ns3_config.use_real_routing_algorithm)
    {
        if (ns3_config.routing_algorithm == "OLSR")
        {
            // Add OLSR routing
            Ipv4ListRoutingHelper ipv4List;
            OlsrHelper olsr;
            olsr.Set("HelloInterval", TimeValue(Seconds(0.5)));
            olsr.Set("TcInterval", TimeValue(Seconds(1)));
            ipv4List.Add(olsr, 100);
            this->internetHelper->SetRoutingHelper(ipv4List);

            // Print routing table every 5 seconds
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
            olsr.PrintRoutingTableAllEvery(Seconds(20), routingStream);
        }
        else if (ns3_config.routing_algorithm == "AODV")
        {
            // Add AODV routing
            Ipv4ListRoutingHelper ipv4List;
            AodvHelper aodv;
            ipv4List.Add(aodv, 100);
            this->internetHelper->SetRoutingHelper(ipv4List);

            // Print routing table every 5 seconds
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
            aodv.PrintRoutingTableAllEvery(Seconds(20), routingStream);
        }
        else if (ns3_config.routing_algorithm == "DSDV")
        {
            // Add DSDV routing
            DsdvHelper dsdv;
            dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(15)));
            dsdv.Set("SettlingTime", TimeValue(Seconds(6)));
            this->internetHelper->SetRoutingHelper(dsdv);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Unsupported routing algorithm %s", ns3_config.routing_algorithm.c_str());
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        Ipv4StaticRoutingHelper staticRouting;
        this->internetHelper->SetRoutingHelper(staticRouting);

        // Print routing table every 20 seconds
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
        staticRouting.PrintRoutingTableAllEvery(Seconds(20), routingStream);
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of IP/ROUTING module");

    /* **************** APPLICATION MODULE **************** */

    // Broadcast flow
    if (ns3_config.enable_broadcast_flow)
    {           
        // Configure the flocking application broadcaster
        this->flocking_broadcaster = CreateObject<FlockingBroadcaster>();
        
        this->flocking_broadcaster->SetStopTime(Seconds(ns3_config.stop_broadcast_time));
        this->flocking_broadcaster->SetAttribute("PacketSize", UintegerValue(ns3_config.broadcast_packet_size));
        this->flocking_broadcaster->SetAttribute("Port", UintegerValue(ns3_config.broadcast_port));
        Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
        rand->SetAttribute("Constant", DoubleValue(ns3_config.broadcast_interval / 1000000.0f)); // from us to s (because custom class Sender uses Interval as Seconds)
        this->flocking_broadcaster->SetAttribute("Interval", PointerValue(rand));
        
        // Configure the flocking application receiver
        this->flocking_receiver = CreateObject<FlockingReceiver>();
        flocking_receiver->SetStartTime(Seconds(0.0)); // Always have the receiver activated.
    }

    // "Mission" flow : unicast, unidirectional
    // "Sender" and "Receiver" are classes defined in the custom ns3 Application wifi-application.h
    if (ns3_config.enable_mission_flow)
    {
        // Configure the sender mission application (will be installed on all source nodes)
        this->mission_application = CreateObject<Sender>();

        this->mission_application->SetStartTime(Seconds(ns3_config.start_mission_time));
        this->mission_application->SetStopTime(Seconds(ns3_config.stop_mission_time));
        this->mission_application->SetAttribute("Destination", Ipv4AddressValue(this->nodes.Get(ns3_config.sink_robot_id)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()));
        this->mission_application->SetAttribute("Port", UintegerValue(ns3_config.mission_port));
        this->mission_application->SetAttribute("PacketSize", UintegerValue(ns3_config.mission_packet_size));
        this->mission_application->SetAttribute("NumPackets", UintegerValue(4294967295)); // do not limit number of sent packets (infinite flow)
        this->mission_application->SetAttribute("FlowId", UintegerValue(ns3_config.mission_flow_id));
        Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
        rand->SetAttribute("Constant", DoubleValue(ns3_config.mission_interval / 1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
        this->mission_application->SetAttribute("Interval", PointerValue(rand));
        this->mission_application->TraceConnectWithoutContext("Tx", MakeCallback(&Ns3Sim::mission_flow_sender_clbk, this));


        // Configure the receiver mission applicatuin (will be installed on the sink node)
        this->mission_receiver = CreateObject<Receiver>();

        this->mission_receiver->SetStartTime(Seconds(0.0));
        this->mission_receiver->SetAttribute("Port", UintegerValue(ns3_config.mission_port));
        this->mission_receiver->TraceConnectWithoutContext("Rx", MakeCallback(&Ns3Sim::mission_flow_receiver_clbk, this));

        RCLCPP_INFO(this->get_logger(), "Configured mission app (sink) for node %d", ns3_config.sink_robot_id);
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of APPLICATION module");

    /* **************** STATS MODULE **************** */
    if (ns3_config.enable_stats_module)
    {
        std::string strategy = "VAT flocking";

        this->data.DescribeRun(this->experience_name, strategy, this->config_file_path, std::to_string(this->run_id));

        // Add any information we wish to record about this run.
        this->data.AddMetadata("author", "tbalaguer");

        // Create a counter for the number of Packets sent WITHIN THE TARGETS
        this->missionTotalTx = CreateObject<PacketCounterCalculator>();
        this->missionTotalTx->SetKey("mission_sent_packets_within_target");
        // this->missionTotalTx->SetContext("mission flow node[" + std::to_string(source_node_id) + "]");
        this->data.AddDataCalculator(this->missionTotalTx);

        // Create a counter for the number of Packets received WITHIN THE TARGETS
        this->missionTotalRx = CreateObject<PacketCounterCalculator>();
        this->missionTotalRx->SetKey("mission_received_packets_within_target");
        // this->missionTotalRx->SetContext("mission flow node[" + std::to_string(sink_node_id) + "]");
        this->data.AddDataCalculator(this->missionTotalRx);

        // Create a statistics object for the delay of the packets
        this->missionDelay = CreateObject<TimeMinMaxAvgTotalCalculator>();
        this->missionDelay->SetKey("mission_packet_delay");
        this->missionDelay->SetContext("mission flow");
        this->data.AddDataCalculator(this->missionDelay);

        // Create a counter for the number of Packets received by each 'navigation flow' server
        for (int i = 0; i < this->nodes.GetN(); i++)
        {
            Ptr<PacketCounterCalculator> navTotalRx = CreateObject<PacketCounterCalculator>();
            navTotalRx->SetKey("nav_received_packets");
            navTotalRx->SetContext("nav flow node[" + std::to_string(i) + "]");
            this->navTotalRxVector.push_back(navTotalRx);
            this->data.AddDataCalculator(this->navTotalRxVector[i]);
        }

        this->navEffectiveTx = CreateObject<PacketCounterCalculator>();
        navEffectiveTx->SetKey("nav_effective_tx");
        navEffectiveTx->SetContext("nav flow effectively sent packets");
        this->data.AddDataCalculator(this->navEffectiveTx);

        // Create a counter for the number of Packets sent by each 'navigation flow' client
        for (int i = 0; i < this->nodes.GetN(); i++)
        {
            Ptr<PacketCounterCalculator> navTotalTx = CreateObject<PacketCounterCalculator>();
            navTotalTx->SetKey("nav_sent_packets");
            navTotalTx->SetContext("nav flow node[" + std::to_string(i) + "]");
            this->navTotalTxVector.push_back(navTotalTx);
            this->data.AddDataCalculator(this->navTotalTxVector[i]);
        }
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of STATS module");
    }
}

void Ns3Sim::RunPseudoRoutingAlgorithm(const std::string pseudo_routing_algo_name)
{

    double bandwidth = DynamicCast<WifiNetDevice>(this->nodes.Get(0)->GetDevice(0))->GetPhy()->GetChannelWidth() * 1e6;

    // Pseudo Routing algorithm
    std::vector<std::vector<std::pair<int, double>>> graph;
    // Create a graph object where the nodes are the agents, and the edges exist only if they received a broadcast message from the neighbor, and weighted according to a cost function
    for (int16_t i = 0; i < this->nodes.GetN(); i++)
    {
        std::vector<std::pair<int, double>> neighbors;
        for (int j = 0; j < this->nodes.GetN(); j++)
        {
            if (i == j)
                continue;

            // j is not a "broadcast neighbor" of i
            if (this->agents[i].neighbors.find(j) == this->agents[i].neighbors.end())
                continue;

            if (pseudo_routing_algo_name == "shortest_dist")
            {
                double distance = this->nodes.Get(i)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(j)->GetObject<MobilityModel>());
                double cost = costFunction(distance, shortest_dist_r1, shortest_dist_c1);
                neighbors.push_back(std::make_pair(j, cost));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with cost %f", i, j, cost);
            }

            else if (pseudo_routing_algo_name == "capacity_bottleneck")
            {
                double capacity = bandwidth * std::log2(1 + this->snrs[i][j]);
                neighbors.push_back(std::make_pair(j, capacity));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with capacity %f", i, j, capacity);
            }

            else if (pseudo_routing_algo_name == "minimize_error_rate")
            {
                // purge old packets
                while (Simulator::Now() - this->broadcast_received_packets[i][j][0] > this->broadcast_window_size)
                {
                    this->broadcast_received_packets[i][j].erase(this->broadcast_received_packets[i][j].begin());
                }
                // Compute error rate
                uint32_t expected_packets_count = this->broadcast_window_size.ToInteger(Time::US) / ns3_config.broadcast_interval;
                double success_prob = this->broadcast_received_packets[i][j].size() / expected_packets_count;
                neighbors.push_back(std::make_pair(j, -std::log(success_prob)));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with success rate %f (%d/%d)", i, j, success_prob, this->broadcast_received_packets[i][j].size(), expected_packets_count);
            }
            else if (pseudo_routing_algo_name == "hybrid_dist_error_rate")
            {
                // purge old packets
                while (Simulator::Now() - this->broadcast_received_packets[i][j][0] > this->broadcast_window_size)
                {
                    this->broadcast_received_packets[i][j].erase(this->broadcast_received_packets[i][j].begin());
                }
                // Compute error rate
                uint32_t expected_packets_count = this->broadcast_window_size.ToInteger(Time::US) / ns3_config.broadcast_interval;
                double error_rate = 1 - (this->broadcast_received_packets[i][j].size() / expected_packets_count);
                double distance = this->nodes.Get(i)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(j)->GetObject<MobilityModel>());
                double cost = hybrid_dist_error_rate_cost_function(distance, hybrid_r1, hybrid_c1, error_rate, hybrid_e1, hybrid_k1, hybrid_k2);
                neighbors.push_back(std::make_pair(j, cost));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with error rate %f (%d/%d)", i, j, error_rate, this->broadcast_received_packets[i][j].size(), expected_packets_count);
            }
            else if (pseudo_routing_algo_name == "predefined_chain")
            {
                continue;
            }
            else
            {
                RCLCPP_FATAL(this->get_logger(), "Pseudo routing algorithm %s not supported", pseudo_routing_algo_name);
                exit(EXIT_FAILURE);
            }
        }
        graph.push_back(neighbors);
    }

    // Run the right graph traversal algorithm
    for (uint32_t source_node_id : this->ns3_config.source_robots_ids)
    {
        int source = source_node_id;
        std::vector<double> dist;
        std::vector<int> parent;

        if (pseudo_routing_algo_name == "shortest_dist" || pseudo_routing_algo_name == "minimize_error_rate")
        {
            dijkstra(source, graph, dist, parent);
        }
        else if (pseudo_routing_algo_name == "capacity_bottleneck")
        {
            dijkstra_bottleneck(source, graph, dist, parent);
            double cost = dist[ns3_config.sink_robot_id];
            // std::ofstream file;
            // file.open(capacity_cost_out_file, std::ios::app);
            // file << Simulator::Now().ToDouble(Time::MS) << "," << cost << std::endl;
            // file.close();
        }
        else if (pseudo_routing_algo_name == "hybrid_dist_error_rate")
        {
            dijkstra(source, graph, dist, parent);
        }
        else if (pseudo_routing_algo_name == "predefined_chain")
        {
            for (int i = 0; i < this->nodes.GetN() - 1; i++)
            {
                parent.push_back(i + 1);
            }
            parent[source] = -1;
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Pseudo routing algorithm %s not supported", pseudo_routing_algo_name);
            exit(EXIT_FAILURE);
        }

        // Transfer the result of the graph traversal algorithm to the simulation structures

        // Downgrade all agents and all neighborhood relations to be "undefined"
        for (auto &agent : this->agents)
        {
            agent.role = AgentRoleType::Undefined;
            for (auto &neighbor : agent.neighbors)
            {
                neighbor.second.role = AgentRoleType::Undefined;
                neighbor.second.link_type = LinkType::FlockingLink;
            }
        }

        std::stack<int> path;
        std::vector<int> path_list;
        int current = this->ns3_config.sink_robot_id;

        // Trace back from destination to source using the parent array
        while (current != -1)
        {
            path.push(current);
            path_list.push_back(current);
            current = parent[current];
        }

        // No path to source
        if (path.top() != source)
        {
            RCLCPP_WARN(this->get_logger(), "No path to source !!");
            continue;
        }

        if (!this->ns3_config.use_real_routing_algorithm)
        {
            updateStaticIpv4Routes(path_list);
        }

        int curr = source;
        // std::cout << "Path: " << source;
        while (path.size() > 1)
        {
            path.pop();
            int neighbor = path.top();
            double distance = this->nodes.Get(curr)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(neighbor)->GetObject<MobilityModel>());

            this->agents[curr].role = AgentRoleType::Mission;
            this->agents[neighbor].role = AgentRoleType::Mission;

            // The current agent and current neighbor are along the data route, so define their links as such.
            if (this->agents[curr].neighbors.find(neighbor) != this->agents[curr].neighbors.end())
            {
                this->agents[curr].neighbors[neighbor].link_type = LinkType::DataLink;
            }
            else
            {
                // Technically, that should not happen since edge (i,j) is in the graph only if i and j are flocking neighbors
                RCLCPP_WARN(this->get_logger(), "Pseudo routing defines that %d and %d should be mission neighbors but %d has no info on %d", curr, neighbor, curr, neighbor);
            }
            if (this->agents[neighbor].neighbors.find(curr) != this->agents[neighbor].neighbors.end())
            {
                this->agents[neighbor].neighbors[curr].link_type = LinkType::DataLink;
            }
            else
            {
                // Technically, that should not happen since edge (i,j) is in the graph only if i and j are flocking neighbors
                RCLCPP_WARN(this->get_logger(), "Pseudo routing defines that %d and %d should be mission neighbors but %d has no info on %d", neighbor, curr, neighbor, curr);
            }

            // The current agent and neighbor agent are mission agent, so they should be considered as mission neighbors by all those that know about them
            for (auto &agent : this->agents)
            {
                if (agent.neighbors.find(curr) != agent.neighbors.end())
                {
                    agent.neighbors[curr].role = AgentRoleType::Mission;
                }
                if (agent.neighbors.find(neighbor) != agent.neighbors.end())
                {
                    agent.neighbors[neighbor].role = AgentRoleType::Mission;
                }
            }

            curr = neighbor;
        }
        // std::cout << std::endl;
    }
}

/**
 * @brief Updates the roles of the agents based on their neighborhood 
 * 
 * When this method is called, we suppose that the "mission" neighbors have ben determined and assigned to agents. We also suppose that the neighbors of each agents have the appropriate neighbor role. 
 * If an agent has at least one "mission" neighbor, it is Potential.
 * If an agent has neighbors but no "mission" neighbor, it is Idle.
 * If an agent has no neighbors, it is Undefined.
 */
void Ns3Sim::UpdateRolesAndNeighbors()
{

    for (auto &agent : this->agents)
    {
        if (agent.role != AgentRoleType::Mission)
        {
            bool has_neighbors = !agent.neighbors.empty();
            bool has_mission_neighbor = false;
            for (auto &neighbor : agent.neighbors)
            {
                if (neighbor.second.role == AgentRoleType::Mission)
                {
                    has_mission_neighbor = true;
                    break;
                }
            }

            if (has_mission_neighbor)
            {
                // agent is a "potential" agent, flag its neighbors as potential neighbors
                agent.role = AgentRoleType::Potential;
                for (auto &neighbor : agent.neighbors)
                {
                    if (neighbor.second.role != AgentRoleType::Mission)
                    {
                        neighbor.second.role = AgentRoleType::Potential;
                    }
                }
            }
            else if (has_neighbors)
            {
                // agent is an "idle" agent, flag its neighbors as "idle" links to display as gray
                agent.role = AgentRoleType::Idle;
                for (auto &neighbor : agent.neighbors)
                {
                    neighbor.second.role = AgentRoleType::Idle;
                }
            }
            else
            {
                // agent is an "undefined" agent, it has no neighbors
                agent.role = AgentRoleType::Undefined;
            }
        }
        else
        {
            if (agent.neighbors.empty())
            {
                // agent is a mission agent but has no neighbors ! Switch to Undefined.
                RCLCPP_WARN(this->get_logger(), "Mission agent %d has no neighbors !", agent.id);
                agent.role = AgentRoleType::Undefined;
            }
            // agent is a mission agent, it should keep its "mission" neighbors but switch its other neighbors to "potential" to display as blue
            for (auto &neighbor : agent.neighbors)
            {
                if (neighbor.second.link_type != LinkType::DataLink)
                {
                    neighbor.second.role = AgentRoleType::Potential;
                }
            }
        }
    }
}

/**
 * @brief Updates the internal state of the agents with the new input commands (called at every timestep)
 * 
 * This is where the controller code goes ! It uses a ROS2 service to fetch the command from the controller.
 */
void Ns3Sim::UpdateCmds()
{
    /*************** Get commands ***************/
    // Wait for existence of service: 1s.
    while(!command_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Service "<< command_client_->get_service_name() << " not available, waiting again...");
    }
    
    auto request = std::make_shared<dancers_msgs::srv::GetAgentVelocities::Request>();

    dancers_msgs::msg::AgentStructArray agent_structs;

    for (agent_t& agent : this->agents)
    {
        dancers_msgs::msg::AgentStruct agent_struct;
        agent_struct.agent_id = agent.id;
        agent_struct.state.position.x = agent.position.x();
        agent_struct.state.position.y = agent.position.y();
        agent_struct.state.position.z = agent.position.z();

        agent_struct.state.velocity_heading.velocity.x = agent.velocity.x();
        agent_struct.state.velocity_heading.velocity.y = agent.velocity.y();
        agent_struct.state.velocity_heading.velocity.z = agent.velocity.z();

        agent_struct.agent_role = static_cast<uint8_t>(agent.role);

        for (auto& neighbor: agent.neighbors)
        {
            dancers_msgs::msg::Neighbor neighbor_msg;
            neighbor_msg.agent_id = neighbor.second.id;
            neighbor_msg.link_quality = neighbor.second.link_quality;
            neighbor_msg.agent_role = static_cast<uint8_t>(neighbor.second.role);
            neighbor_msg.position.x = neighbor.second.position.x();
            neighbor_msg.position.y = neighbor.second.position.y();
            neighbor_msg.position.z = neighbor.second.position.z();
            neighbor_msg.velocity.x = neighbor.second.velocity.x();
            neighbor_msg.velocity.y = neighbor.second.velocity.y();
            neighbor_msg.velocity.z = neighbor.second.velocity.z();
            
            agent_struct.neighbor_array.neighbors.push_back(neighbor_msg);
        }

        agent_struct.heartbeat_received = agent.heartbeat_received;
        agent_struct.heartbeat_sent = agent.heartbeat_sent;

        request->agent_structs.push_back(agent_struct);

        if(publish_agent_structs)
        {
            agent_structs.structs.push_back(agent_struct);
        }
    }

    if (publish_agent_structs)
    {
        agent_structs_pub_->publish(agent_structs);
    }

    auto result = command_client_->async_send_request(request);

    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    if (result.valid())
    {
        // Can't get the reference directly of velocity heading directly since result becomes invalid after the get() call.
        std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> request_msg = result.get();
        std::vector<dancers_msgs::msg::VelocityHeading>& velocity_headings = request_msg->velocity_headings.velocity_heading_array;
        
        assert(velocity_headings.size() == this->agents.size());

        RCLCPP_DEBUG(this->get_logger(), "Service answered with %d commands !", velocity_headings.size());

        // save the command vectors
        for (int i=0; i < this->agents.size(); i++)
        {
            this->agents[i].cmd_velocity[0] = velocity_headings[i].velocity.x;
            this->agents[i].cmd_velocity[1] = velocity_headings[i].velocity.y;
            this->agents[i].cmd_velocity[2] = velocity_headings[i].velocity.z;
            this->agents[i].cmd_heading = velocity_headings[i].heading;
        }
    }
    else
    {
        command_client_->remove_pending_request(result);
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't call the command service: " << command_client_->get_service_name()<< ". Skipping control step");
        return;
    }

}

void Ns3Sim::DisplayNetworkRviz()
{
        // Display edges between neighbors
    // mission neighbors
    visualization_msgs::msg::Marker network_marker_mission{};
    network_marker_mission.header.frame_id = "map";
    network_marker_mission.id = 1010;
    network_marker_mission.type = visualization_msgs::msg::Marker::LINE_LIST;
    network_marker_mission.action = visualization_msgs::msg::Marker::ADD;
    network_marker_mission.scale.x = 0.2;
    network_marker_mission.color.r = 1.0;  // Red color
    network_marker_mission.color.g = 0.0;
    network_marker_mission.color.b = 0.0;
    network_marker_mission.color.a = 1.0;  // Fully opaque
    network_marker_mission.lifetime = rclcpp::Duration::from_seconds(0.1);

    // potential neighbors
    visualization_msgs::msg::Marker network_marker_potential{};
    network_marker_potential.header.frame_id = "map";
    network_marker_potential.id = 1011;
    network_marker_potential.type = visualization_msgs::msg::Marker::LINE_LIST;
    network_marker_potential.action = visualization_msgs::msg::Marker::ADD;
    network_marker_potential.scale.x = 0.1;
    network_marker_potential.color.r = 0.0;  
    network_marker_potential.color.g = 0.0;
    network_marker_potential.color.b = 1.0;  // Blue color
    network_marker_potential.color.a = 1.0;  // Fully opaque
    network_marker_potential.lifetime = rclcpp::Duration::from_seconds(0.1);

    // idle neighbors
    visualization_msgs::msg::Marker network_marker_idle{};
    network_marker_idle.header.frame_id = "map";
    network_marker_idle.id = 1012;
    network_marker_idle.type = visualization_msgs::msg::Marker::LINE_LIST;
    network_marker_idle.action = visualization_msgs::msg::Marker::ADD;
    network_marker_idle.scale.x = 0.1;
    network_marker_idle.color.r = 0.8;  
    network_marker_idle.color.g = 0.8;
    network_marker_idle.color.b = 0.8;
    network_marker_idle.color.a = 1.0;  // Fully opaque
    network_marker_idle.lifetime = rclcpp::Duration::from_seconds(0.1);

    for (const auto& agent : this->agents)
    {
        for (const auto& neighbor : agent.neighbors)
        {
            geometry_msgs::msg::Point p1{};
            p1.x = agent.position.x();
            p1.y = agent.position.y();
            p1.z = agent.position.z();
            geometry_msgs::msg::Point p2{};
            p2.x = this->agents[neighbor.first].position.x();
            p2.y = this->agents[neighbor.first].position.y();
            p2.z = this->agents[neighbor.first].position.z();

            // Red links are between two mission agents and carry data
            if (neighbor.second.role == AgentRoleType::Mission && neighbor.second.link_type == LinkType::DataLink)
            {
                network_marker_mission.points.push_back(p1);
                network_marker_mission.points.push_back(p2);
            }
            // Blue links are for potential neighbors
            else if (neighbor.second.role == AgentRoleType::Potential)
            {
                network_marker_potential.points.push_back(p1);
                network_marker_potential.points.push_back(p2);
            }
            // Gray links are for idle neighbors
            else if (neighbor.second.role == AgentRoleType::Idle)
            {
                network_marker_idle.points.push_back(p1);
                network_marker_idle.points.push_back(p2);
            }
        }
    }

    this->network_mission_pub_->publish(network_marker_mission);
    this->network_potential_pub_->publish(network_marker_potential);
    this->network_idle_pub_->publish(network_marker_idle);
}

/**
 * @brief Callback for the "targets" topic, used to add a new target area to the simulation.
 * 
 * @param msg a Target message that contains the position, the type (source / sink) and the associated agent of the target.
 */
void Ns3Sim::targets_clbk(dancers_msgs::msg::Target msg)
{
    Ptr<ns3::Node> target_node = this->nodes.Get(msg.id);
    // Any Agent has a broadcast sender and receiver, prevent duplicate mission application by checking the number of applications.
    // We also do a double-check in case this agent was already considered a source agent
    if (target_node->GetNApplications() < 3 && std::find(this->ns3_config.source_robots_ids.begin(), this->ns3_config.source_robots_ids.end(), msg.id) == this->ns3_config.source_robots_ids.end())
    {
        // add sender application
        Ptr<Sender> sender = CreateObject<Sender>();
        target_node->AddApplication(sender);
        sender->SetStartTime(Seconds(this->ns3_config.start_mission_time));
        sender->SetStopTime(Seconds(this->ns3_config.stop_mission_time));
        sender->SetAttribute("Destination", Ipv4AddressValue(this->nodes.Get(this->ns3_config.sink_robot_id)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()));
        sender->SetAttribute("Port", UintegerValue(this->ns3_config.mission_port));
        sender->SetAttribute("PacketSize", UintegerValue(this->ns3_config.mission_packet_size));
        sender->SetAttribute("NumPackets", UintegerValue(4294967295));
        sender->SetAttribute("FlowId", UintegerValue(this->ns3_config.mission_flow_id));
        Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
        rand->SetAttribute("Constant", DoubleValue(ns3_config.mission_interval / 1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
        sender->SetAttribute("Interval", PointerValue(rand));
        sender->TraceConnectWithoutContext("Tx", MakeCallback(&Ns3Sim::mission_flow_sender_clbk_2, this));

        this->ns3_config.source_robots_ids.push_back(msg.id);

        RCLCPP_INFO(this->get_logger(), "Configured mission app (source) for node %d", target_node->GetId());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Node %d already has 2 applications", msg.id);
    }
}

void Ns3Sim::spawn_uav_clbk(dancers_msgs::msg::AgentStruct msg)
{
    // Spawn a new UAV in the simulation
    RCLCPP_INFO(this->get_logger(), "Spawning UAV %d at position (%f, %f, %f)", msg.agent_id, msg.state.position.x, msg.state.position.y, msg.state.position.z);

    // Create a new node
    Ptr<ns3::Node> new_node = CreateObject<ns3::Node>();
    this->nodes.Add(new_node);
    
    RCLCPP_DEBUG(this->get_logger(), "Node %d created. There is currently %d nodes.", new_node->GetId(), this->nodes.GetN());

    // Set the mobility model
    Ptr<MobilityModel> nodeMob;
    nodeMob = CreateObject<ConstantVelocityMobilityModel>();
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(msg.state.position.x, msg.state.position.y, msg.state.position.z));
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(msg.state.velocity_heading.velocity.x, msg.state.velocity_heading.velocity.y, msg.state.velocity_heading.velocity.z));
    new_node->AggregateObject(nodeMob);

    // Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
    BuildingsHelper::Install(new_node);


    // Add the new node to the agents vector
    agent_t new_agent = this->ROS_msg_to_agent_t(msg);
    new_agent.cmd_velocity = {0.0, 0.0, 0.0};
    new_agent.cmd_heading = 0.0;
    this->agents.push_back(new_agent);

    // Add the node to the network
    this->AddNodeToWifiNetwork(new_node);

    RCLCPP_INFO(this->get_logger(), "UAV %d spawned successfully", msg.agent_id);
}

void Ns3Sim::AddNodeToWifiNetwork(Ptr<ns3::Node> node)
{
    uint32_t nodeId = node->GetId();

    // check that all the helpers pointers are initialized
    if (!this->wifiPhyHelper || !this->wifiMacHelper || !this->wifiHelper || !this->internetHelper)
    {
        RCLCPP_FATAL(this->get_logger(), "Wifi helpers are not initialized, cannot add node to wifi network");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Adding node %d to the wifi network", nodeId);

    // Install the (already configured) wifi module on the node.
    NetDeviceContainer new_net_device = (*this->wifiHelper).Install((*this->wifiPhyHelper), (*this->wifiMacHelper), node);
    
    // Install the (already configured) internet helper on the node 
    this->internetHelper->Install(node);

    // Connect all the trace callbacks
    Ptr<WifiMac> wifi_mac = node->GetDevice(0)->GetObject<WifiNetDevice>()->GetMac();
    Ptr<WifiPhy> wifi_phy = node->GetDevice(0)->GetObject<WifiNetDevice>()->GetPhy();
    this->mac_to_id[wifi_mac->GetAddress()] = nodeId;
    wifi_phy->TraceConnect("PhyTxBegin", std::to_string(nodeId), MakeCallback(&Ns3Sim::PhyTxBeginTrace, this));
    wifi_phy->TraceConnect("PhyTxEnd", std::to_string(nodeId), MakeCallback(&Ns3Sim::PhyTxEndTrace, this));
    wifi_phy->TraceConnect("PhyRxEnd", std::to_string(nodeId), MakeCallback(&Ns3Sim::PhyRxEndTrace, this));
    wifi_phy->TraceConnect("MonitorSnifferRx", std::to_string(nodeId), MakeCallback(&Ns3Sim::MonitorSnifferRxTrace, this));
    wifi_mac->TraceConnect("MacTx", std::to_string(nodeId), MakeCallback(&Ns3Sim::MacTxTrace, this));

    // Assign the right IPv4 address (10.0.0.<nodeId>) to the node
    Ipv4Address nodeIpAddr (("10.0.0." + std::to_string(nodeId + 1)).c_str());
    Ipv4Mask networkMask ("255.255.255.0");
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    // int32_t interface = ipv4->GetInterfaceForDevice(new_net_device.Get(0));
    // if (interface < 0)
    // {
    //     RCLCPP_FATAL(this->get_logger(), "Failed to get Ipv4 interface for node %d", nodeId);
    //     return;
    // }
    ipv4->AddAddress(0, Ipv4InterfaceAddress(nodeIpAddr, networkMask));
    ipv4->SetUp(0);

    // Add the applications to the nodes

    // Random start, otherwise they never access the medium (I think)
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
    x->SetAttribute("Min", DoubleValue(0.0));
    x->SetAttribute("Max", DoubleValue(1.0));

    if (this->ns3_config.enable_broadcast_flow)
    {
        // App 0 is the Flocking broadcaster, it sends a heartbeat message every ns3_config.broadcast_interval
        node->AddApplication(this->flocking_broadcaster);

        // App 1 is the Flocking receiver, it receives the heartbeat message and updates the neighbors
        node->AddApplication(this->flocking_receiver);
        this->flocking_broadcaster->SetStartTime(Seconds(this->ns3_config.start_broadcast_time + x->GetValue()));

        if (this->flocking_broadcaster->TraceConnect("Tx", std::to_string(nodeId), MakeCallback(&Ns3Sim::flocking_broadcaster_clbk, this)))
        {
            RCLCPP_DEBUG(this->get_logger(), "Connected broadcaster to node %d", nodeId);
        }
        if (this->flocking_receiver->TraceConnect("Rx", std::to_string(nodeId), MakeCallback(&Ns3Sim::flocking_receiver_clbk, this)))
        {
            RCLCPP_DEBUG(this->get_logger(), "Connected receiver to node %d", nodeId);
        }

    }

    if (this->ns3_config.enable_mission_flow)
    {
        // Install Mission sender application on all the source robots
        // for (uint32_t source_node_id : ns3_config.source_robots_ids)
        if (find(this->ns3_config.source_robots_ids.begin(), this->ns3_config.source_robots_ids.end(), nodeId) != this->ns3_config.source_robots_ids.end())
        {
            // App 2 is the Mission flow sender (unicast, only for leaders)
            node->AddApplication(this->mission_application);

            RCLCPP_INFO(this->get_logger(), "Configured mission app (source) for node %d", nodeId);
        }

        if (nodeId == this->ns3_config.sink_robot_id)
        {
            // App 3 is the Mission flow receiver, only activate on the sink)
            node->AddApplication(this->mission_receiver);

            RCLCPP_INFO(this->get_logger(), "Configured mission receiver (sink) for node %d", nodeId);
        }
    }

    // Enable pcap output for the NetDevice of thie node.
    this->wifiPhyHelper->EnablePcap("ns3_sim", new_net_device);
    
}

/**
 * @brief Callback for the SpectrumChannel "PathLoss" trace
 *
 * It saves the pathloss value computed by the spectrum propagation module in the pathloss matrix
 */
void Ns3Sim::SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t txId = txPhy->GetDevice()->GetNode()->GetId();
    uint32_t rxId = rxPhy->GetDevice()->GetNode()->GetId();
    this->pathlosses[txId][rxId] = -lossDb;
    // std::cout << "Tx: " << txId <<  " Rx: " << rxId << " Loss: " << lossDb << std::endl;
}

/**
 * @brief Callback for the WifiPhy "PhyTxBegin" trace
 *
 * When a packet tagged with FlowId "mission" is being sent at the PHY level, it saves the MAC layer "destination" (so, the next hop) of the packet as a neighbor.
 */
void Ns3Sim::PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow)
{
    RCLCPP_INFO(this->get_logger(), "Sent a packet at layer PHY (START)");
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t destId = this->mac_to_id.find(hdr.GetAddr1())->second;

    // From a networking POV, this is cheating, we are sending a message, we can't know what is the reception power, however here it is accepted
    double rxPow = this->pathlosses[nodeId][destId];
    Time txTime = Simulator::Now();

    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->ns3_config.mission_flow_id)
        {
            // We are sending a "mission" packet, link-layer destination should be our (outgoing) neighbor
            if (this->agents[nodeId].neighbors.find(destId) != this->agents[nodeId].neighbors.end())
            {
                this->agents[nodeId].neighbors[destId].role = AgentRoleType::Mission;
                this->agents[nodeId].neighbors[destId].link_type = LinkType::DataLink;
            }
            else
            {
                // We have sent a mission message to a node that is not a "flocking neighbor", this is a problem !
            }
        }
    }
}

/**
 * @brief Callback for the WifiPhy "PhyTxEnd" trace
 *
 * Used only for stats here
 */
void Ns3Sim::PhyTxEndTrace(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Sent a packet at layer PHY (END)");
    uint32_t nodeId = std::stoi(context);
    FlowIdTag flow_tag;
    if (packet->PeekPacketTag(flow_tag))
    {
        if (flow_tag.GetFlowId() == this->ns3_config.broadcast_flow_id)
        {
            if (ns3_config.enable_stats_module)
            {
                this->navEffectiveTx->PacketUpdate("", packet);
            }

            this->agents[nodeId].heartbeat_sent++;
            // std::cout << "Heartbeat sent by " << nodeId << " : " << packet->GetSize() << " bytes." << std::endl;
        }
    }
}

/**
 * @brief Callback for the WifiPhy "PhyRxEnd" trace
 *
 * When a packet tagged with FlowId "mission" is being received at the PHY level, it saves the MAC layer "source" (so, the previous hop) of the packet as a neighbor.
 */
void Ns3Sim::PhyRxEndTrace(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Received a packet at layer PHY");

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
        if (nav_flow.GetFlowId() == this->ns3_config.mission_flow_id && nodeId == destId)
        {
            // We are receiving a "mission" packet, link-layer destination should be our (outgoing) neighbor
            if (this->agents[nodeId].neighbors.find(sourceId) != this->agents[nodeId].neighbors.end())
            {
                this->agents[nodeId].neighbors[sourceId].role = AgentRoleType::Mission;
                this->agents[nodeId].neighbors[sourceId].link_type = LinkType::DataLink;
            }
            else
            {
                // We have received a mission packet from a neighbor we didn't know about yet. Since we do not have its Pose, do nothing
            }
        }
    }
}

void Ns3Sim::MacTxTrace(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Sent a packet at layer MAC");
}

/**
 * @brief Callback for the mission flow receiver
 */
void Ns3Sim::mission_flow_receiver_clbk(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Mission flow packet received from.");

    if (this->save_mission_packets)
    {
        // print packet to file
        myTimestampTag timestamp;
        // Should never not be found since the sender is adding it, but
        // you never know.
        if (packet->FindFirstMatchingByteTag(timestamp))
        {
            Time tx = timestamp.GetTimestamp();
            Time now = Simulator::Now();
            Time delay = now - tx;

            std::ofstream f(this->mission_packets_file_, std::ios::app);
            f << now.ToInteger(Time::US) << "," << delay.ToInteger(Time::US) << std::endl;
            f.close();
        }
    }
}

/**
 * @brief Callback for the mission flow sender
 */
void Ns3Sim::mission_flow_sender_clbk(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Mission flow packet sent");
}

/**
 * @brief Callback for the mission flow sender
 */
void Ns3Sim::mission_flow_sender_clbk_2(Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Mission flow packet sent BY DYNAMIC SOURCE NODE");
}

/**
 * @brief Callback for the reception of a packet at the application layer
 */
void Ns3Sim::flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    RCLCPP_INFO(this->get_logger(), "Flock packet received");

    Time rxTime = Simulator::Now();
    double rxPow = this->pathlosses[std::stoi(context)][peer_id];
    
    // Read Header and update corresponding neighbor entry (the "NeighborInfo_t" represents the "freshest" information about a neighbor)
    FlockingHeader fl_hdr;
    packet->PeekHeader(fl_hdr);
    
    Vector neighbor_position = fl_hdr.GetPosition();
    Vector neighbor_velocity = fl_hdr.GetVelocity();

    // It might be the first time we see this agent, add it to the map
    if (this->agents[std::stoi(context)].neighbors.find(peer_id) == this->agents[std::stoi(context)].neighbors.end())
    {
        this->agents[std::stoi(context)].neighbors[peer_id] = NeighborInfo_t();
    }

    this->agents[std::stoi(context)].neighbors[peer_id].position = Eigen::Vector3d(neighbor_position.x, neighbor_position.y, neighbor_position.z);    
    this->agents[std::stoi(context)].neighbors[peer_id].velocity = Eigen::Vector3d(neighbor_velocity.x, neighbor_velocity.y, neighbor_velocity.z);
    this->agents[std::stoi(context)].neighbors[peer_id].id = peer_id;
    this->agents[std::stoi(context)].neighbors[peer_id].last_seen = rxTime.ToInteger(Time::US);
    this->agents[std::stoi(context)].neighbors[peer_id].link_quality = rxPow;
    this->agents[std::stoi(context)].neighbors[peer_id].link_type = LinkType::FlockingLink;

    this->agents[std::stoi(context)].heartbeat_received++;

    this->broadcast_received_packets[std::stoi(context)][peer_id].push_back(rxTime);
}

/**
 * @brief Callback for the transmission of a packet at the application layer
 */
void Ns3Sim::flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet)
{
    Time txTime = Simulator::Now();

    RCLCPP_DEBUG(this->get_logger(), "Flocking broadcast packet sent");
}

void Ns3Sim::MonitorSnifferRxTrace(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    uint32_t nodeId = std::stoi(context);
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t sourceId = this->mac_to_id.find(hdr.GetAddr2())->second;

    this->snrs[nodeId][sourceId] = (double)pow(10, signalNoise.signal / 10.0) / (double)pow(10, signalNoise.noise / 10.0);

    RCLCPP_DEBUG(this->get_logger(), "[%d] Packet heard from %d, SNR = %f", nodeId, sourceId, this->snrs[nodeId][sourceId]);
}

std::string Ns3Sim::generateCommandsMsg()
{
    dancers_update_proto::VelocityHeadingVector velocity_headings_msg;

    for (uint32_t i = 0; i < this->agents.size(); i++)
    {
        dancers_update_proto::VelocityHeading* velocity_heading_msg = velocity_headings_msg.add_velocity_heading();
        velocity_heading_msg->set_agentid(this->agents[i].id);
        velocity_heading_msg->set_vx(this->agents[i].cmd_velocity[0]);
        velocity_heading_msg->set_vy(this->agents[i].cmd_velocity[1]);
        velocity_heading_msg->set_vz(this->agents[i].cmd_velocity[2]);
        velocity_heading_msg->set_heading(this->agents[i].cmd_heading);
    }

    // std::cout << velocity_headings_msg.DebugString() << std::endl;

    std::string str_response;
    velocity_headings_msg.SerializeToString(&str_response);
    return str_response;
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

    // Assign roles based on their neighborhood type
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        std::vector<NeighborInfo_t> orderedNeighbors;

        // Sort neighbors by descending link quality
        for (auto const &neighbor : this->agents[i].neighbors)
        {
            orderedNeighbors.push_back(neighbor.second);
        }
        std::sort(orderedNeighbors.begin(), orderedNeighbors.end(), [](const NeighborInfo_t &a, const NeighborInfo_t &b)
                  { return a.link_quality > b.link_quality; });
        
        // Create the message for agent i  
        dancers_update_proto::OrderedNeighbors *neighbor_msg = ordered_neighbors_msg.add_ordered_neighbors();
        neighbor_msg->set_agentid(this->agents[i].id);
        neighbor_msg->set_role(static_cast<dancers_update_proto::OrderedNeighbors_Role>(this->agents[i].role));     // When doing these casts, ensure the order of the enum in the protobuf and the NeighborInfo_t are the same
        for (auto const &neighbor : orderedNeighbors)
        {
            neighbor_msg->add_neighborid(neighbor.id);
            neighbor_msg->add_linkquality(neighbor.link_quality);
            neighbor_msg->add_neighbortype(static_cast<dancers_update_proto::OrderedNeighbors_Role>(neighbor.role));
            neighbor_msg->add_time_val(neighbor.last_seen);
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

    if (this->mode == "exchange_neighbors")
    {
        network_update_msg.set_payload(gzip_compress(generateNeighborsMsg()));
    }
    else if (this->mode == "exchange_commands")
    {
        network_update_msg.set_payload(gzip_compress(generateCommandsMsg()));
    }
    else 
    {
        RCLCPP_FATAL(this->get_logger(), "Unknown mode: %s", this->mode.c_str());
        exit(EXIT_FAILURE);
    }

    std::string str_response;
    network_update_msg.SerializeToString(&str_response);

    return str_response;
}

/**
 * @brief Clear timed out neighbors
 *
 * Removes neighbors that have not been heard for a while.
 */
void Ns3Sim::timeoutNeighbors()
{
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        std::vector<uint32_t> toRemove;
        for (auto const &neighbor : this->agents[i].neighbors)
        {
            if ((Simulator::Now() - MicroSeconds(neighbor.second.last_seen)) > MicroSeconds(this->ns3_config.broadcast_timeout))
            {
                toRemove.push_back(neighbor.first);
            }
        }
        for (auto const &neighbor : toRemove)
        {
            this->agents[i].neighbors.erase(neighbor);
        }
    }
}


/**
 * @brief Update the pathlosses matrix when the YansWifiPhy model is used
 *
 * Explanation for this is that the YansWifiPhy model does not provide the same "pathloss" traceSource that the SpectrumChannel entity provides, thus, we need to actually compute and save the pathloss values between nodes, when we use the YansWifiPhy model
 */
void Ns3Sim::updateNeighborsPathloss()
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

void Ns3Sim::updateStaticIpv4Routes(const std::vector<int> path)
{
    int sink = path[0];
    Ipv4Address sink_addr = this->nodes.Get(sink)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress();
    for (int i = 1; i < path.size(); i++)
    {
        int source = path[i];
        int next_hop = path[i - 1];

        Ipv4Address next_hop_addr = this->nodes.Get(next_hop)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress();

        Ptr<Ipv4StaticRouting> staticRouting = DynamicCast<Ipv4StaticRouting>(this->nodes.Get(source)->GetObject<Ipv4>()->GetRoutingProtocol());
        if (staticRouting == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Ipv4StaticRouting not found in node %d", source);
            return;
        }
        while (staticRouting->GetNRoutes() > 2)
        {
            RCLCPP_DEBUG(this->get_logger(), "Removing routes from node %d", source);
            staticRouting->RemoveRoute(2);
        }
        staticRouting->AddHostRouteTo(sink_addr, next_hop_addr, 1, 0);

        RCLCPP_DEBUG(this->get_logger(), "Added route %d -> %d via %d", source, sink, next_hop);
    }
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

            // if (currTime % Seconds(0.1) == Time(0)){
            //     uint64_t bytes_received_this_iteration = mission_server->GetReceived()*packet_size - bytes_received;
            //     bytes_received = mission_server->GetReceived()*packet_size;
            //     RCLCPP_INFO(this->get_logger(), "Mission flow throughput: %f Mbps", (float)(bytes_received_this_iteration * 10 / 1000000.0));
            // }

            if (!physics_update_msg.payload().empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                robots_positions_msg.ParseFromString(gzip_decompress(physics_update_msg.payload()));

                // std::cout << robots_positions_msg.DebugString() << std::endl;

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
                        this->agents[i].position[0] = robots_positions_msg.pose(i).x();
                        this->agents[i].position[1] = robots_positions_msg.pose(i).y();
                        this->agents[i].position[2] = robots_positions_msg.pose(i).z();

                        this->agents[i].velocity[0] = robots_positions_msg.pose(i).vx();
                        this->agents[i].velocity[1] = robots_positions_msg.pose(i).vy();
                        this->agents[i].velocity[2] = robots_positions_msg.pose(i).vz();

                        if (this->ns3_config.use_localization_noise)
                        {
                            std::normal_distribution<double> distribution(0.0, this->ns3_config.localization_noise_stddev);
                            this->agents[i].position[0] += distribution(this->random_generator);
                            this->agents[i].position[1] += distribution(this->random_generator);
                            this->agents[i].position[2] += distribution(this->random_generator);
                        }

                        // Set (ns-3) node position
                        Vector pos, vel;
                        pos.x = this->agents[i].position.x();
                        pos.y = this->agents[i].position.y();
                        pos.z = this->agents[i].position.z();
                        vel.x = this->agents[i].velocity.x();
                        vel.y = this->agents[i].velocity.y();
                        vel.z = this->agents[i].velocity.z();
                        Ptr<ConstantVelocityMobilityModel> mobility = this->nodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
                        mobility->SetPosition(pos);
                        mobility->SetVelocity(vel);
                    }
                }
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
            }

            if (this->ns3_config.wifi_phy_type == "YansWifiPhy")
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

            RCLCPP_DEBUG(this->get_logger(), "Simulation step finished. Starting timeoutNeighbors.");
            this->timeoutNeighbors();

            RCLCPP_DEBUG(this->get_logger(), "Starting RunPseudoRoutingAlgorithm.");
            this->RunPseudoRoutingAlgorithm(this->pseudoRoutingAlgo);

            RCLCPP_DEBUG(this->get_logger(), "Starting UpdateRolesAndNeighbors.");
            this->UpdateRolesAndNeighbors();

            if (this->mode == "exchange_commands")
            {
                RCLCPP_DEBUG(this->get_logger(), "Starting UpdateCmds.");
                this->UpdateCmds();
            }

            RCLCPP_DEBUG(this->get_logger(), "Starting UpdateNetworkRviz.");
            this->DisplayNetworkRviz();

            RCLCPP_DEBUG(this->get_logger(), "Generating response protobuf.");
            std::string response = gzip_compress(generateResponseProtobuf());

            currTime += step_size;
            if (this->print_simulation_advancement_)
            {
                RCLCPP_INFO(this->get_logger(), "Simulation time: %f/%f", currTime.GetSeconds(), simEndTime.GetSeconds());
            }

            // Send the response to the network coordinator
            socket->send_one_message(response);
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
            output->SetFilePrefix(this->experience_name);
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

/**
 * @brief Cost function
 *
 * Here, this cost function is used for the Dijkstra algorithm costs. As a function of ditance, it is a flat function that becomes exponential at r1
 */
double Ns3Sim::costFunction(double dist, double r1, double c1)
{
    if (dist < r1)
    {
        return c1;
    }
    else
    {
        return pow((dist - r1), 2) + c1;
    }
}

/**
 * @brief Cost function
 *
 * Here, this cost function is used for the Dijkstra algorithm costs. As a function of ditance, it is a flat function that becomes exponential at r1
 */
double Ns3Sim::hybrid_dist_error_rate_cost_function(double dist, double r1, double c1, double error_rate, double e1, double k1, double k2)
{
    return k1 * Ns3Sim::costFunction(dist, r1, c1) + k2 * exp(e1 * error_rate);
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

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
#include <std_msgs/msg/u_int32.hpp>

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
            return;
        }

        // get the name of the folder containing the config file
        // boost::filesystem::path config_file(config_file_path);
        // boost::filesystem::path config_folder = config_file.parent_path();

        // Get the path to the ROS_WS, it is mandatory to run
        if (getenv("ROS_WS") == NULL)
        {
            RCLCPP_FATAL(this->get_logger(), "ROS_WS environment variable not set, aborting.");
            return;
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

        rclcpp::QoS reliable_qos(10); // depth of 10
        reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        reliable_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        reliable_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        /* ----------- ROS2 Subscribers ----------- */
        std::string events_namespace = "/events/";
        this->update_target_sub_ = this->create_subscription<dancers_msgs::msg::Target>(
            events_namespace + "update_target",
            reliable_qos,
            std::bind(&Ns3Sim::update_target_clbk, this, _1));
        this->spawn_uav_ = this->create_subscription<dancers_msgs::msg::AgentStruct>(
            events_namespace + "spawn_uav",
            reliable_qos,
            std::bind(&Ns3Sim::spawn_uav_clbk, this, _1));
        this->uav_failure_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            events_namespace + "uav_failure",
            reliable_qos,
            std::bind(&Ns3Sim::uav_failure_clbk, this, _1));

        /* ----------- ROS2 Publishers ----------- */
        this->network_mission_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_mission_links", 10);
        this->network_potential_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_potential_links", 10);
        this->network_idle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_idle_links", 10);
        if (this->publish_agent_structs)
        {
            this->agent_structs_pub_ = this->create_publisher<dancers_msgs::msg::AgentStructArray>("agent_structs", 10);
        }

        // ========================= NS3 =========================

        this->ns3_config_mut.lock();
        this->ns3_config = this->ReadNs3Config(config);
        this->ns3_config_mut.unlock();

        this->ConfigureNs3(this->ns3_config);

        this->InitAgents();

        this->step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        this->currTime = MicroSeconds(0);                                       // us
        this->simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)
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
        RCLCPP_INFO(this->get_logger(), "\x1b[32mns-3 connector initialized. Ready to start simulation !\x1b[0m");

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
    std::mutex nodes_mutex_;

    std::string config_file_path;
    std::string ros_ws_path_;
    ns3_configuration_t ns3_config;
    std::mutex ns3_config_mut;
    std::string mode;

    std::unique_ptr<WifiHelper> wifiHelper;
    std::unique_ptr<WifiMacHelper> wifiMacHelper;
    std::unique_ptr<WifiPhyHelper> wifiPhyHelper;
    std::unique_ptr<InternetStackHelper> internetHelper;
    std::unique_ptr<Ipv4AddressHelper> ipv4AddressHelper;

    Ptr<UniformRandomVariable> random_gen_start_app;

    std::mt19937 random_generator;

    std::map<uint32_t, std::map<uint32_t, double>> pathlosses;
    std::map<uint32_t, std::map<uint32_t, double>> snrs;
    std::map<uint32_t, std::map<uint32_t, std::vector<Time>>> broadcast_received_packets;
    std::map<Mac48Address, uint32_t> mac_to_id;

    std::map<uint32_t, agent_t> agents_;
    std::mutex agents_mutex_;

    std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> last_response_from_cmd_service_;

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
    rclcpp::Subscription<dancers_msgs::msg::Target>::SharedPtr update_target_sub_;
    void update_target_clbk(dancers_msgs::msg::Target msg);
    rclcpp::Subscription<dancers_msgs::msg::AgentStruct>::SharedPtr spawn_uav_;
    void spawn_uav_clbk(dancers_msgs::msg::AgentStruct msg);
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr uav_failure_sub_;
    void uav_failure_clbk(std_msgs::msg::UInt32 msg);

    /* Publishers */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_mission_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_potential_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_idle_pub_;
    rclcpp::Publisher<dancers_msgs::msg::AgentStructArray>::SharedPtr agent_structs_pub_;

    Ptr<PropagationLossModel> m_propagationLossModel;

    bool targets_reached;

    // ns3 Trace callbacks
    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    void Ipv4L3RxTrace(std::string context, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);
    void LocalDeliverTrace(std::string context, const Ipv4Header &header, Ptr<const Packet> packet, uint32_t interface);
    void PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow);
    void PhyTxEndTrace(std::string context, Ptr<const Packet> packet);
    void PhyRxEndTrace(std::string context, Ptr<const Packet> packet);
    void MacTxTrace(std::string context, Ptr<const Packet> packet);
    void MacRxTrace(std::string context, Ptr<const Packet> packet);
    void MacRxDropTrace(std::string context, Ptr<const Packet> packet);
    void mission_flow_receiver_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk(std::string context, Ptr<const Packet> packet);
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

    // Simulation-related methods
    void SetupOutputFiles(const YAML::Node &config);
    ns3_configuration_t ReadNs3Config(const YAML::Node &config);
    void ConfigureNs3(const ns3_configuration_t &ns3_config);
    void InitAgents();
    void timeoutNeighbors();
    void updateNeighborsPathloss();
    void updateStaticIpv4Routes(const std::vector<int> path);
    void updateLeadersApplications();
    void UpdateRolesAndNeighbors();
    void UpdateCmds();
    void AddWifiNetworkStack(Ptr<ns3::Node> node);
    void AddFlockingApplication(Ptr<ns3::Node> node);
    void AddMissionApplication(Ptr<ns3::Node> node);
    void RunPseudoRoutingAlgorithm(const std::string pseudo_routing_algo_name);
    void DisplayNetworkRviz();
    
    // Simulation loop
    void Loop();

    // util functions
    double costFunction(double dist, double r1, double c1);
    double hybrid_dist_error_rate_cost_function(double dist, double r1, double c1, double error_rate, double e1, double k1, double k2);
    void printNodeAggregatedObjects(Ptr<ns3::Node> node);
    uint32_t getNodeIndexFromId(uint32_t nodeId);
    Ptr<ns3::Node> getNodePtrFromIndex(uint32_t nodeIndex);

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
    a.neighbors = std::map<uint32_t, NeighborInfo_t>();

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
        f << "rcv_time(us),delay(us),target_id,in_target" << std::endl;
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

    for (auto target_area : config["secondary_objectives"])
    {
        TargetArea_t t;
        t.id = target_area["id"].as<uint32_t>();
        t.x = target_area["position"]["x"].as<float>();
        t.y = target_area["position"]["y"].as<float>();
        t.z = target_area["position"]["z"].as<float>();
        t.is_sink = target_area["is_sink"].as<bool>();
        for (auto assigned_agent : target_area["assigned_agents"])
        {
            t.assigned_agents.push_back(assigned_agent.as<uint32_t>());
        }
        ns3_config.target_areas.push_back(t);
    }

    ns3_config.enable_stats_module = config["enable_stats"].as<bool>();

    return ns3_config;
}

/**
 * @brief Initialize the agents structs. Called only once at the beginning of the simulation
 */
void Ns3Sim::InitAgents()
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
 * This function is called once, at the beginning of the simulation. It creates the buildings and prepares the objects that will be used to configure the nodes themselves.
 * It does not create the nodes
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
        std::unique_ptr<YansWifiPhyHelper> yansPhyHelper = std::unique_ptr<YansWifiPhyHelper>(dynamic_cast<YansWifiPhyHelper *>(this->wifiPhyHelper.get()));

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
        SpectrumWifiPhyHelper *spectrumPhyHelper = dynamic_cast<SpectrumWifiPhyHelper *>(this->wifiPhyHelper.get());

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

    /* **************** APPLICATION MODULE **************** */

    // Broadcast flow
    if (ns3_config.enable_broadcast_flow)
    {
        // Create a random variable to handle App. random start, otherwise agents all try to send their packet at the same instant.
        this->random_gen_start_app = CreateObject<UniformRandomVariable>();
        this->random_gen_start_app->SetAttribute("Min", DoubleValue(0.0));
        this->random_gen_start_app->SetAttribute("Max", DoubleValue(ns3_config.broadcast_interval * 1e-6));
    }

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
        for (int i = 0; i < this->ns3_config.num_nodes; i++)
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
        for (int i = 0; i < this->ns3_config.num_nodes; i++)
        {
            Ptr<PacketCounterCalculator> navTotalTx = CreateObject<PacketCounterCalculator>();
            navTotalTx->SetKey("nav_sent_packets");
            navTotalTx->SetContext("nav flow node[" + std::to_string(i) + "]");
            this->navTotalTxVector.push_back(navTotalTx);
            this->data.AddDataCalculator(this->navTotalTxVector[i]);
        }
    }
}

void Ns3Sim::RunPseudoRoutingAlgorithm(const std::string pseudo_routing_algo_name)
{
    double bandwidth;
    {
        std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
        bandwidth = DynamicCast<WifiNetDevice>(this->nodes.Get(0)->GetDevice(0))->GetPhy()->GetChannelWidth() * 1e6;
    }

    // Pseudo Routing algorithm
    std::map<uint32_t, std::map<uint32_t, double>> graph;
    // Create a graph object where the nodes are the agents, and the edges exist only if they received a broadcast message from the neighbor, and weighted according to a cost function

    this->agents_mutex_.lock();
    for (const auto &[agent_id, agent_struct] : this->agents_)
    {
        // get agent index
        int agent_index = agent_struct.node_container_index;
        std::map<uint32_t, double> neighbors;
        for (const auto &[neighbor_id, neighbor_struct] : agent_struct.neighbors)
        {
            int neighbor_index = this->agents_[neighbor_id].node_container_index;
            if (agent_id == neighbor_id)
                continue;

            // j is not a "broadcast neighbor" of i
            if (agent_struct.neighbors.find(neighbor_id) == agent_struct.neighbors.end())
                continue;

            if (pseudo_routing_algo_name == "shortest_dist")
            {
                this->agents_mutex_.unlock();
                double distance;
                {
                    std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
                    distance = this->nodes.Get(agent_index)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(neighbor_index)->GetObject<MobilityModel>());
                }
                this->agents_mutex_.lock();
                double cost = costFunction(distance, shortest_dist_r1, shortest_dist_c1);
                neighbors[neighbor_id] = cost;
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with cost %f", agent_id, neighbor_id, cost);
            }

            else if (pseudo_routing_algo_name == "capacity_bottleneck")
            {
                double capacity = bandwidth * std::log2(1 + this->snrs[agent_id][neighbor_id]);
                neighbors[neighbor_id] = capacity;
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with capacity %f", agent_id, neighbor_id, capacity);
            }

            else if (pseudo_routing_algo_name == "minimize_error_rate")
            {
                // purge old packets
                while (Simulator::Now() - this->broadcast_received_packets[agent_id][neighbor_id][0] > this->broadcast_window_size)
                {
                    this->broadcast_received_packets[agent_id][neighbor_id].erase(this->broadcast_received_packets[agent_id][neighbor_id].begin());
                }
                // Compute error rate
                uint32_t expected_packets_count = this->broadcast_window_size.ToInteger(Time::US) / ns3_config.broadcast_interval;
                double success_prob = this->broadcast_received_packets[agent_id][neighbor_id].size() / expected_packets_count;
                neighbors[neighbor_id] = -std::log(success_prob);
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with success rate %f (%d/%d)", agent_id, neighbor_id, success_prob, this->broadcast_received_packets[agent_id][neighbor_id].size(), expected_packets_count);
            }
            else if (pseudo_routing_algo_name == "hybrid_dist_error_rate")
            {
                // purge old packets
                while (Simulator::Now() - this->broadcast_received_packets[agent_id][neighbor_id][0] > this->broadcast_window_size)
                {
                    this->broadcast_received_packets[agent_id][neighbor_id].erase(this->broadcast_received_packets[agent_id][neighbor_id].begin());
                }
                // Compute error rate
                uint32_t expected_packets_count = this->broadcast_window_size.ToInteger(Time::US) / ns3_config.broadcast_interval;
                double error_rate = 1 - (this->broadcast_received_packets[agent_id][neighbor_id].size() / expected_packets_count);
                this->agents_mutex_.unlock();
                double distance;
                {
                    std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
                    distance = this->nodes.Get(agent_index)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(neighbor_index)->GetObject<MobilityModel>());
                }
                this->agents_mutex_.lock();
                double cost = hybrid_dist_error_rate_cost_function(distance, hybrid_r1, hybrid_c1, error_rate, hybrid_e1, hybrid_k1, hybrid_k2);
                neighbors[neighbor_id] = cost;
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with error rate %f (%d/%d)", agent_id, neighbor_id, error_rate, this->broadcast_received_packets[agent_id][neighbor_id].size(), expected_packets_count);
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
        graph[agent_id] = neighbors;
    } // end for
    this->agents_mutex_.unlock();

    {
        std::lock_guard<std::mutex> lock_agents(this->agents_mutex_);
        // Downgrade all agents and all neighborhood relations to be "undefined"
        for (auto &[agent_id, agent] : this->agents_)
        {
            agent.role = AgentRoleType::Undefined;
            for (auto &neighbor : agent.neighbors)
            {
                neighbor.second.role = AgentRoleType::Undefined;
                neighbor.second.link_type = LinkType::FlockingLink;
            }
        }
    }

    // Run the right graph traversal algorithm
    for (const uint32_t source_node_id : this->ns3_config.source_robots_ids)
    {
        int source = source_node_id;
        std::map<uint32_t, double> dist;
        std::map<uint32_t, uint32_t> parent;

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
            // In predefined_chain, the agents are ordered by agent_id and the chain follows the strictly growing order
            std::vector<uint32_t> ordered_agent_ids;
            for (const auto &[agent_id, agent] : this->agents_)
            {
                ordered_agent_ids.push_back(agent_id);
            }
            std::sort(ordered_agent_ids.begin(), ordered_agent_ids.end());
            for (size_t i = 0; i < ordered_agent_ids.size() - 1; i++)
            {
                parent[ordered_agent_ids[i]] = ordered_agent_ids[i + 1];
            }
            // wraps around
            parent[ordered_agent_ids.size() - 1] = parent[0];
            // delete neighbor of the source (breaks the circle)
            parent[source] = -1;
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Pseudo routing algorithm %s not supported", pseudo_routing_algo_name);
            exit(EXIT_FAILURE);
        }

        // Transfer the result of the graph traversal algorithm to the simulation structures

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
            double distance;
            {
                std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
                distance = this->nodes.Get(this->agents_[curr].node_container_index)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(this->agents_[neighbor].node_container_index)->GetObject<MobilityModel>());
            }

            {
                std::lock_guard<std::mutex> lock_agents(this->agents_mutex_);
                this->agents_[curr].role = AgentRoleType::Mission;
                this->agents_[neighbor].role = AgentRoleType::Mission;

                // The current agent and current neighbor are along the data route, so define their links as such.
                if (this->agents_[curr].neighbors.find(neighbor) != this->agents_[curr].neighbors.end())
                {
                    this->agents_[curr].neighbors[neighbor].link_type = LinkType::DataLink;
                }
                else
                {
                    // Technically, that should not happen since edge (i,j) is in the graph only if i and j are flocking neighbors
                    RCLCPP_WARN(this->get_logger(), "Pseudo routing defines that %d and %d should be mission neighbors but %d has no info on %d", curr, neighbor, curr, neighbor);
                }
                if (this->agents_[neighbor].neighbors.find(curr) != this->agents_[neighbor].neighbors.end())
                {
                    this->agents_[neighbor].neighbors[curr].link_type = LinkType::DataLink;
                }
                else
                {
                    // Technically, that should not happen since edge (i,j) is in the graph only if i and j are flocking neighbors
                    RCLCPP_WARN(this->get_logger(), "Pseudo routing defines that %d and %d should be mission neighbors but %d has no info on %d", neighbor, curr, neighbor, curr);
                }

                // The current agent and neighbor agent are mission agent, so they should be considered as mission neighbors by all those that know about them
                for (auto &[agent_id, agent] : this->agents_)
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
            }
            curr = neighbor;
        }
        int next_hop;
        {
        std::lock_guard<std::mutex> lock_agents(this->agents_mutex_);
            for (auto neigh : this->agents_[source].neighbors)
            {
                if (neigh.second.role == AgentRoleType::Mission)
                {
                    next_hop = neigh.second.id;
                }
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Ran the pseudo-routing algorithm for source agent %d. Next hop: %d.", source, next_hop);
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
    // We are going to modify the agents_ object, so lock it
    std::lock_guard<std::mutex> lock(this->agents_mutex_);

    for (auto &[agent_id, agent] : this->agents_)
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
    while (!command_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Service " << command_client_->get_service_name() << " not available, waiting again...");
    }

    
    // Build the request message with the last known agents positions and velocities

    auto request = std::make_shared<dancers_msgs::srv::GetAgentVelocities::Request>();
    dancers_msgs::msg::AgentStructArray agent_structs;
    {
        // We are going to modify the agents_ object, so lock it
        std::lock_guard<std::mutex> lock(this->agents_mutex_);
        for (auto &[agent_id, agent] : this->agents_)
        {
            if (agent.crashed == false)
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
                
                for (auto &neighbor : agent.neighbors)
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
                
                if (publish_agent_structs)
                {
                    agent_structs.structs.push_back(agent_struct);
                }
            }
        }
    }

    // Publish the request, essentially for debugging
    if (publish_agent_structs)
    {
        agent_structs_pub_->publish(agent_structs);
    }

    // Send the request
    auto result = command_client_->async_send_request(request);

    // Get the response of the service, and 
    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    if (result.valid())
    {
        // Get the shared_ptr to the response (keeps ownership while we call get()).
        std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response_msg = result.get();

        // We keep a copy of the response message in a shared_ptr. That is because UpdateCmds() is executed inside the thread running the "Loop()".
        // Many segfaults happened here, because the pointer was deallocated even though this part of the code should still hold a reference to the shared_ptr. The reason is probably that executing a thread in parallel of ROS2 "main" thread (executor) is not the default way of doing things.  
        this->last_response_from_cmd_service_ = response_msg;

        RCLCPP_DEBUG(this->get_logger(), "Service answered with %zu commands !", this->last_response_from_cmd_service_->velocity_headings.velocity_heading_array.size());

        // save the command vectors safely using RAII lock
        {
            std::lock_guard<std::mutex> lock(this->agents_mutex_);

            for (const auto &vel_head : this->last_response_from_cmd_service_->velocity_headings.velocity_heading_array)
            {
                uint32_t agent_id = vel_head.agent_id;

                // If agent may not exist, check or create safely -- this depends on your invariants.
                // e.g. if agents_[agent_id] is valid:
                auto it = this->agents_.find(agent_id);
                if (it == this->agents_.end()) {
                    RCLCPP_WARN(this->get_logger(), "Got command for unknown agent id %u", agent_id);
                    continue;
                }

                it->second.cmd_velocity[0] = vel_head.velocity.x;
                it->second.cmd_velocity[1] = vel_head.velocity.y;
                it->second.cmd_velocity[2] = vel_head.velocity.z;
                it->second.cmd_heading = vel_head.heading;
            }
        } // lock released here    
    }
    else
    {
        command_client_->remove_pending_request(result);
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't call the command service: " << command_client_->get_service_name() << ". Skipping control step");
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
    network_marker_mission.color.r = 1.0; // Red color
    network_marker_mission.color.g = 0.0;
    network_marker_mission.color.b = 0.0;
    network_marker_mission.color.a = 1.0; // Fully opaque
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
    network_marker_potential.color.b = 1.0; // Blue color
    network_marker_potential.color.a = 1.0; // Fully opaque
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
    network_marker_idle.color.a = 1.0; // Fully opaque
    network_marker_idle.lifetime = rclcpp::Duration::from_seconds(0.1);

    // We are going to modify the agents_ object, so lock it
    std::lock_guard<std::mutex> lock(this->agents_mutex_);

    for (const auto &[agent_id, agent] : this->agents_)
    {
        for (const auto &[neighbor_id, neighbor] : agent.neighbors)
        {
            geometry_msgs::msg::Point p1{};
            p1.x = agent.position.x();
            p1.y = agent.position.y();
            p1.z = agent.position.z();
            geometry_msgs::msg::Point p2{};
            p2.x = this->agents_[neighbor_id].position.x();
            p2.y = this->agents_[neighbor_id].position.y();
            p2.z = this->agents_[neighbor_id].position.z();

            // Red links are between two mission agents and carry data
            if (neighbor.role == AgentRoleType::Mission && neighbor.link_type == LinkType::DataLink)
            {
                network_marker_mission.points.push_back(p1);
                network_marker_mission.points.push_back(p2);
            }
            // Blue links are for potential neighbors
            else if (neighbor.role == AgentRoleType::Potential)
            {
                network_marker_potential.points.push_back(p1);
                network_marker_potential.points.push_back(p2);
            }
            // Gray links are for idle neighbors
            else if (neighbor.role == AgentRoleType::Idle)
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
 * @brief Callback for the "targets" topic, used to add a new target area to the simulation or move an existing one.
 *
 * @param msg a Target message that contains the position, the type (source / sink) and the associated agent of the target.
 */
void Ns3Sim::update_target_clbk(dancers_msgs::msg::Target msg)
{
    {
        std::lock_guard<std::mutex> lock(this->ns3_config_mut);
        for (auto &target : this->ns3_config.target_areas)
        {
            if (target.id == msg.target_id)
            {
                // Target already exists, update its position and type
                RCLCPP_DEBUG(this->get_logger(), "Updating target %d position to (%f, %f, %f) and type %s", msg.target_id, msg.position.x, msg.position.y, msg.position.z, msg.is_sink ? "sink" : "source");
                target.x = msg.position.x;
                target.y = msg.position.y;
                target.z = msg.position.z;
                target.is_sink = msg.is_sink;
                target.assigned_agents.clear();
                for (const uint32_t concerned_agent : msg.concerned_agents)
                {
                    target.assigned_agents.push_back(concerned_agent);
                }
            }
        }
    }

    if (this->ns3_config.enable_mission_flow)
    {

        // When we receive an update for a target, add mission application for the concerned agents.
        bool all_agents_concerned = false;
        for (uint32_t concerned_agent : msg.concerned_agents)
        {
            if (concerned_agent == -1)
            {
                // This target should be applied to all agents.
                all_agents_concerned = true;
            }
            else
            {
                uint32_t idx = this->getNodeIndexFromId(concerned_agent);
                Ptr<ns3::Node> new_leader = this->getNodePtrFromIndex(idx);

                if (!msg.is_sink)
                {

                    this->ns3_config_mut.lock();
                    if (std::find(this->ns3_config.source_robots_ids.begin(), this->ns3_config.source_robots_ids.end(), concerned_agent) == this->ns3_config.source_robots_ids.end())
                    {
                        // There is a new "source" robot (i.e. leader)
                        this->ns3_config.source_robots_ids.push_back(concerned_agent);
                    }
                    this->ns3_config_mut.unlock();
                }
                else if (msg.is_sink)
                {
                    // There is a single SINK agent and it is not dynamic for now.
                    RCLCPP_ERROR(this->get_logger(), "Received a target update that changes the SINK agent: Not supported! Update ignored.");
                }

                // Effectively add the Application to the node.
                this->AddMissionApplication(new_leader);
            }
        }

        if (all_agents_concerned)
        {
            // We are going to modify the agents_ object, so lock it
            std::lock_guard<std::mutex> lock(this->agents_mutex_);

            for (auto &[agent_id, agent] : this->agents_)
            {
                Ptr<ns3::Node> new_leader = this->getNodePtrFromIndex(agent.node_container_index);

                if (!msg.is_sink)
                {
                    this->ns3_config_mut.lock();
                    this->ns3_config.source_robots_ids.push_back(agent_id);
                    this->ns3_config_mut.unlock();
                }
                else if (!msg.is_sink)
                {
                    // There is a single SINK agent and it is not dynamic for now.
                    RCLCPP_ERROR(this->get_logger(), "Received a target update that changes the SINK agent: Not supported! Update ignored.");
                }

                // Effectively add the Application to the node.
                this->AddMissionApplication(new_leader);
            }
        }
    }
}

void Ns3Sim::spawn_uav_clbk(dancers_msgs::msg::AgentStruct msg)
{
    // Spawn a new UAV in the simulation
    RCLCPP_INFO(this->get_logger(), "Spawning UAV %d at position (%f, %f, %f)", msg.agent_id, msg.state.position.x, msg.state.position.y, msg.state.position.z);

    // Abort if the agent already exists. 
    // Otherwise, add it to agents_
    {
        std::lock_guard<std::mutex> lock(this->agents_mutex_);
        if (this->agents_.find(msg.agent_id) != this->agents_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Agent %d already exists, skipping spawn.", msg.agent_id);
            return;
        }
    
    
        // Add the new node to the agents vector
        agent_t new_agent = this->ROS_msg_to_agent_t(msg);
        new_agent.cmd_velocity = {0.0, 0.0, 0.0};
        new_agent.cmd_heading = 0.0;
        new_agent.node_container_index = this->nodes.GetN();
        this->agents_[new_agent.id] = new_agent;
    }

    // Create a new ns-3 node
    Ptr<ns3::Node> new_node = CreateObject<ns3::Node>();
    {
        std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
        this->nodes.Add(new_node);
        RCLCPP_INFO(this->get_logger(), "Node %d created. There is currently %d nodes.", msg.agent_id, this->nodes.GetN());
    }

    // Set the mobility model
    Ptr<MobilityModel> nodeMob;
    nodeMob = CreateObject<ConstantVelocityMobilityModel>();
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(msg.state.position.x, msg.state.position.y, msg.state.position.z));
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(msg.state.velocity_heading.velocity.x, msg.state.velocity_heading.velocity.y, msg.state.velocity_heading.velocity.z));
    new_node->AggregateObject(nodeMob);

    // Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
    BuildingsHelper::Install(new_node);

    // Add the node to the network
    this->AddWifiNetworkStack(new_node);

    // Add the applications to the nodes
    if (this->ns3_config.enable_broadcast_flow)
    {
        this->AddFlockingApplication(new_node);
    }

    if (this->ns3_config.enable_mission_flow)
    {
        this->AddMissionApplication(new_node);
    }

    this->printNodeAggregatedObjects(new_node);

    // Print all the applications aggregated to the Node
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Installed applications on Node " << new_node->GetId() << ":");
    // Iterate through the installed applications
    for (uint32_t i = 0; i < new_node->GetNApplications(); ++i)
    {
        Ptr<Application> application = new_node->GetApplication(i);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "  - TypeId: " << application->GetInstanceTypeId().GetName());
    }

    RCLCPP_INFO(this->get_logger(), "UAV %d spawned successfully", msg.agent_id);
}

void Ns3Sim::uav_failure_clbk(std_msgs::msg::UInt32 msg)
{

    // Since we will manipulate the agents_ object, lock its mutex
    // We access the nodes container, so lock it
    
    uint32_t idx;
    {
        std::lock_guard<std::mutex> lock(this->agents_mutex_);
        idx = this->agents_[msg.data].node_container_index;
    }
    Ptr<ns3::Node> node;
    {
        std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
        node = this->nodes.Get(idx);
    }

    // Stop all running applications
    for (uint32_t i = 0; i < node->GetNApplications(); ++i)
    {
        Ptr<Application> application = node->GetApplication(i);
        application->SetStopTime(Simulator::Now());
    }

    // Set Ipv4 interfaces down
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        ipv4->SetDown(i);
    }

    // Set the agent to crashed
    {
        std::lock_guard<std::mutex> lock(this->agents_mutex_);
        this->agents_[msg.data].crashed = true;
    }

    RCLCPP_INFO(this->get_logger(), "UAV %d failure ! Deactivated its applications and IPV4 interfaces.", msg.data);
}

void Ns3Sim::AddWifiNetworkStack(Ptr<ns3::Node> node)
{
    // The ns-3 Node Id can be different from "our" agent_id. In ns-3, node->GetId() happens to be the index of the node in the NodeList (see doc).
    
    // Find the node's ns-3 index
    int node_index;
    {
        std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);
        node_index = node->GetId();
    }
    // Find the corresponding node id
    uint32_t agent_id;
    {
        std::lock_guard<std::mutex> lock_agents(this->agents_mutex_);
        for (const auto &[id, agent] : this->agents_)
        {
            if (agent.node_container_index == node_index)
            {
                // We found the agent_id corresponding to the ns-3 node index
                agent_id = id;
            }
        }
    }

    // Verify that all the helpers pointers are initialized
    if (!this->wifiPhyHelper || !this->wifiMacHelper || !this->wifiHelper || !this->internetHelper)
    {
        RCLCPP_FATAL(this->get_logger(), "Wifi helpers are not initialized, cannot add node to wifi network");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Preparing the network stack of node %d...", agent_id);

    // Install the (already configured) wifi module on the node.
    NetDeviceContainer new_net_device = (*this->wifiHelper).Install((*this->wifiPhyHelper), (*this->wifiMacHelper), node);

    // Install the (already configured) internet helper on the node
    this->internetHelper->Install(node);

    // Assign the right IPv4 address (10.0.0.<agent_id+1>) to the node
    Ipv4InterfaceAddress nodeIpv4AddressInterface(Ipv4Address(("10.0.0." + std::to_string(agent_id + 1)).c_str()), Ipv4Mask("255.255.255.0"));
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    int32_t interfaceIndex = ipv4->AddInterface(new_net_device.Get(0)); // Get the first device
    if (!ipv4->AddAddress(interfaceIndex, nodeIpv4AddressInterface))
    {
        RCLCPP_FATAL(this->get_logger(), "Couldn't assign IPv4 address to node %d", agent_id);
    }
    ipv4->SetUp(interfaceIndex);

    // Enable pcap output for the NetDevice of thie node.
    this->wifiPhyHelper->EnablePcap("ns3_sim", new_net_device);

    // Connect all the trace callbacks
    // Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Rx", MakeCallback(&Ns3Sim::Ipv4L3RxTrace, this));
    Ptr<Ipv4L3Protocol> ipv4_protocol = node->GetObject<Ipv4L3Protocol>();
    Ptr<WifiMac> wifi_mac = node->GetDevice(0)->GetObject<WifiNetDevice>()->GetMac();
    Ptr<WifiPhy> wifi_phy = node->GetDevice(0)->GetObject<WifiNetDevice>()->GetPhy();
    this->mac_to_id[wifi_mac->GetAddress()] = agent_id;
    ipv4_protocol->TraceConnect("Rx", std::to_string(agent_id), MakeCallback(&Ns3Sim::Ipv4L3RxTrace, this));
    ipv4_protocol->TraceConnect("LocalDeliver", std::to_string(agent_id), MakeCallback(&Ns3Sim::LocalDeliverTrace, this));
    wifi_phy->TraceConnect("PhyTxBegin", std::to_string(agent_id), MakeCallback(&Ns3Sim::PhyTxBeginTrace, this));
    wifi_phy->TraceConnect("PhyTxEnd", std::to_string(agent_id), MakeCallback(&Ns3Sim::PhyTxEndTrace, this));
    wifi_phy->TraceConnect("PhyRxEnd", std::to_string(agent_id), MakeCallback(&Ns3Sim::PhyRxEndTrace, this));
    wifi_phy->TraceConnect("MonitorSnifferRx", std::to_string(agent_id), MakeCallback(&Ns3Sim::MonitorSnifferRxTrace, this));
    wifi_mac->TraceConnect("MacTx", std::to_string(agent_id), MakeCallback(&Ns3Sim::MacTxTrace, this));
    wifi_mac->TraceConnect("MacRx", std::to_string(agent_id), MakeCallback(&Ns3Sim::MacRxTrace, this));
    wifi_mac->TraceConnect("MacRxDrop", std::to_string(agent_id), MakeCallback(&Ns3Sim::MacRxDropTrace, this));
}

/**
 * @brief Add the two applications enabling flocking on the node. A FlockingBroadcaster and a FlockingReceiver.
 */
void Ns3Sim::AddFlockingApplication(Ptr<ns3::Node> node)
{
    // Find the agent_id corresponding to the ns-3 node index
    uint32_t agent_id;
    for (const auto &[id, agent] : this->agents_)
    {
        if (agent.node_container_index == node->GetId())
        {
            agent_id = id;
        }
    }

    // Configure the flocking application broadcaster
    Ptr<FlockingBroadcaster> flocking_broadcaster = CreateObject<FlockingBroadcaster>();
    flocking_broadcaster->SetStartTime(Seconds(this->ns3_config.start_broadcast_time + this->random_gen_start_app->GetValue()));
    flocking_broadcaster->SetStopTime(Seconds(ns3_config.stop_broadcast_time));
    flocking_broadcaster->SetAttribute("PacketSize", UintegerValue(ns3_config.broadcast_packet_size));
    flocking_broadcaster->SetAttribute("Port", UintegerValue(ns3_config.broadcast_port));
    Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
    rand->SetAttribute("Constant", DoubleValue(ns3_config.broadcast_interval / 1000000.0f)); // from us to s (because custom class Sender uses Interval as Seconds)
    flocking_broadcaster->SetAttribute("Interval", PointerValue(rand));

    // Configure the flocking application receiver
    Ptr<FlockingReceiver> flocking_receiver = CreateObject<FlockingReceiver>();
    flocking_receiver->SetStartTime(Seconds(0.0)); // Always have the receiver activated.
    flocking_receiver->SetAttribute("Port", UintegerValue(ns3_config.broadcast_port));

    node->AddApplication(flocking_broadcaster);

    // App 1 is the Flocking receiver, it receives the heartbeat message and updates the neighbors
    node->AddApplication(flocking_receiver);

    if (flocking_broadcaster->TraceConnect("Tx", std::to_string(agent_id), MakeCallback(&Ns3Sim::flocking_broadcaster_clbk, this)))
    {
        RCLCPP_DEBUG(this->get_logger(), "Connected broadcaster to node %d", agent_id);
    }
    if (flocking_receiver->TraceConnect("Rx", std::to_string(agent_id), MakeCallback(&Ns3Sim::flocking_receiver_clbk, this)))
    {
        RCLCPP_DEBUG(this->get_logger(), "Connected receiver to node %d", agent_id);
    }
}

/**
 * @brief Installs either the Sender or the Receiver application on the node, if necessary. 
 * 
 * The agent must be both in the "source_robots_ids" and in the "concerned_agents" of a target in the ns3_config
 * To ensure no data race, always call AddMissionApplication while holding the agents_ mutex !
 */
void Ns3Sim::AddMissionApplication(Ptr<ns3::Node> node)
{
    uint32_t agent_id;
    for (const auto &[id, agent] : this->agents_)
    {
        if (agent.node_container_index == node->GetId())
        {
            agent_id = id;
        }
    }

    uint32_t target_id;
    int counter = 0;
    for (auto &target : this->ns3_config.target_areas)
    {
        for (auto assigned_agent : target.assigned_agents)
        {
            if (assigned_agent == agent_id)
            {
                if (counter >= 1)
                {
                    RCLCPP_FATAL(this->get_logger(), "An agent can not be assigned to more than one target area! Exiting.");
                    exit(EXIT_FAILURE);
                }
                target_id = target.id;
                counter += 1;
            }
        }
    }

    // Install Mission sender application on all the source robots
    // for (uint32_t source_node_id : ns3_config.source_robots_ids)
    if (find(this->ns3_config.source_robots_ids.begin(), this->ns3_config.source_robots_ids.end(), agent_id) != this->ns3_config.source_robots_ids.end())
    {
        
        // Check if the "Sender" application is already installed on the node
        for (uint32_t i = 0; i < node->GetNApplications(); ++i)
        {
            Ptr<Application> application = node->GetApplication(i);
            if (application->GetInstanceTypeId().GetName() == "Sender")
            {
                RCLCPP_DEBUG(this->get_logger(), "Sender application already installed on node %d.", agent_id);
                return;
            }
        }

        // Find the index of the "sink" node, and the associated IPv4 address
        uint32_t sink_node_index = this->agents_[ns3_config.sink_robot_id].node_container_index;
        if (sink_node_index >= this->nodes.GetN())
        {
            RCLCPP_FATAL(this->get_logger(), "Sink node index %d is out of range (total number of nodes: %d)", sink_node_index, this->nodes.GetN());
            return;
        }
        Ipv4Address sink_ip_address = this->nodes.Get(sink_node_index)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

        // Configure the sender mission application and install it on the source node
        Ptr<Sender> mission_application = CreateObject<Sender>();
        mission_application->SetStartTime(Seconds(ns3_config.start_mission_time));
        mission_application->SetStopTime(Seconds(ns3_config.stop_mission_time));
        mission_application->SetAttribute("Destination", Ipv4AddressValue(sink_ip_address));
        mission_application->SetAttribute("Port", UintegerValue(ns3_config.mission_port));
        mission_application->SetAttribute("PacketSize", UintegerValue(ns3_config.mission_packet_size));
        mission_application->SetAttribute("NumPackets", UintegerValue(4294967295)); // do not limit number of sent packets (infinite flow)
        mission_application->SetAttribute("FlowId", UintegerValue(ns3_config.mission_flow_id));
        mission_application->SetAttribute("TargetId", UintegerValue(target_id));
        Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
        rand->SetAttribute("Constant", DoubleValue(ns3_config.mission_interval / 1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
        mission_application->SetAttribute("Interval", PointerValue(rand));
        mission_application->TraceConnect("Tx", std::to_string(agent_id), MakeCallback(&Ns3Sim::mission_flow_sender_clbk, this));

        // App 2 is the Mission flow sender (unicast, only for leaders)
        node->AddApplication(mission_application);

        RCLCPP_INFO(this->get_logger(), "Configured mission sender (source) for node %d assigned to target %d", agent_id, target_id);
    }
    else if (agent_id == this->ns3_config.sink_robot_id)
    {
        // Verify if the node already has the correct application
        for (uint32_t i = 0; i < node->GetNApplications(); ++i)
        {
            Ptr<Application> application = node->GetApplication(i);
            if (application->GetInstanceTypeId().GetName() == "Receiver")
            {
                RCLCPP_WARN(this->get_logger(), "Receiver application already installed on node %d.", agent_id);
                return;
            }
        }

        // Configure the receiver mission application
        Ptr<Receiver> mission_receiver = CreateObject<Receiver>();
        mission_receiver->SetStartTime(Seconds(0.0));
        mission_receiver->SetAttribute("Port", UintegerValue(ns3_config.mission_port));
        mission_receiver->TraceConnectWithoutContext("Rx", MakeCallback(&Ns3Sim::mission_flow_receiver_clbk, this));

        // App 3 is the Mission flow receiver, only activate on the sink)
        node->AddApplication(mission_receiver);

        RCLCPP_INFO(this->get_logger(), "Configured mission receiver (sink) for node %d", agent_id);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "AddMissionApplication called for node %d but this node is neither a source node nor a sink node. Ignoring.", agent_id);
    }
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

void Ns3Sim::Ipv4L3RxTrace(std::string context, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] Received a packet at IPv4 layer (interface %d)", context.c_str(), interface);
}

void Ns3Sim::LocalDeliverTrace(std::string context, const Ipv4Header &header, Ptr<const Packet> packet, uint32_t interface)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] Forwarded a packet up the stack (interface %d)", context.c_str(), interface);
}

/**
 * @brief Callback for the WifiPhy "PhyTxBegin" trace
 *
 * When a packet tagged with FlowId "mission" is being sent at the PHY level, it saves the MAC layer "destination" (so, the next hop) of the packet as a neighbor.
 */
void Ns3Sim::PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow)
{
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t destId = this->mac_to_id.find(hdr.GetAddr1())->second;

    // From a networking POV, this is cheating, we are sending a message, we can't know what is the reception power, however here it is accepted
    double rxPow = this->pathlosses[nodeId][destId];
    Time txTime = Simulator::Now();

    // Since we will manipulate the agents_ object, lock its mutex
    std::lock_guard<std::mutex> lock(this->agents_mutex_);

    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->ns3_config.mission_flow_id)
        {
            // We are sending a "mission" packet, link-layer destination should be our (outgoing) neighbor
            if (this->agents_[nodeId].neighbors.find(destId) != this->agents_[nodeId].neighbors.end())
            {
                this->agents_[nodeId].neighbors[destId].role = AgentRoleType::Mission;
                this->agents_[nodeId].neighbors[destId].link_type = LinkType::DataLink;
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
    RCLCPP_DEBUG(this->get_logger(), "[%s] Sent a packet at PHY layer", context.c_str());
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

            std::lock_guard<std::mutex> lock(agents_mutex_);
            this->agents_[nodeId].heartbeat_sent++;
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
    RCLCPP_DEBUG(this->get_logger(), "[%s] Received a packet at PHY layer", context.c_str());
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t sourceId = this->mac_to_id.find(hdr.GetAddr2())->second;
    uint32_t destId;
    auto it = this->mac_to_id.find(hdr.GetAddr1());
    if (it != this->mac_to_id.end())
    {
        destId = it->second;
    }
    else
    {
        // RCLCPP_WARN(this->get_logger(), "[%d] Received a packet but the destination MAC address is not known", nodeId);
    }

    if (hdr.GetAddr1().IsBroadcast())
    {
        RCLCPP_DEBUG(this->get_logger(), "[%d] This packet has broadcast MAC address.", nodeId);
    }

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
            // Since we will manipulate the agents_ object, lock its mutex
            std::lock_guard<std::mutex> lock(this->agents_mutex_);

            // We are receiving a "mission" packet, link-layer destination should be our (outgoing) neighbor
            if (this->agents_[nodeId].neighbors.find(sourceId) != this->agents_[nodeId].neighbors.end())
            {
                this->agents_[nodeId].neighbors[sourceId].role = AgentRoleType::Mission;
                this->agents_[nodeId].neighbors[sourceId].link_type = LinkType::DataLink;
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
    RCLCPP_DEBUG(this->get_logger(), "Sent a packet at layer MAC");
}

void Ns3Sim::MacRxTrace(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Received a packet at layer MAC");
}

void Ns3Sim::MacRxDropTrace(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Dropped a packet at layer MAC");
}

/**
 * @brief Callback for the mission flow receiver
 */
void Ns3Sim::mission_flow_receiver_clbk(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Mission flow packet received.");

    if (this->save_mission_packets)
    {
        // print packet to file

        // Should never not be found since the sender is adding it, but
        // you never know.
        myTimestampTag timestamp;
        MissionHeader missionHdr;
        if (packet->FindFirstMatchingByteTag(timestamp) && packet->PeekHeader(missionHdr) > 0)
        {
            Time tx = timestamp.GetTimestamp();
            Time now = Simulator::Now();
            Time delay = now - tx;

            uint32_t target_id = missionHdr.GetTargetId();
            bool in_target_area = missionHdr.GetInTargetArea();

            std::ofstream f(this->mission_packets_file_, std::ios::app);
            f << now.ToInteger(Time::US) << "," << delay.ToInteger(Time::US) << "," << target_id << "," << in_target_area << std::endl;
            f.close();
        }
    }
}

/**
 * @brief Callback for the mission flow sender
 */
void Ns3Sim::mission_flow_sender_clbk(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "[%d] Mission flow packet sent", std::stoi(context));
}

/**
 * @brief Callback for the mission flow sender
 */
void Ns3Sim::mission_flow_sender_clbk_2(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Mission flow packet sent BY DYNAMIC SOURCE NODE");
}

/**
 * @brief Callback for the reception of a packet at the application layer
 */
void Ns3Sim::flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    uint32_t nodeId = std::stoi(context);

    RCLCPP_DEBUG(this->get_logger(), "[%d] Flock packet received", nodeId);

    Time rxTime = Simulator::Now();
    double rxPow = this->pathlosses[nodeId][peer_id];

    // Read Header and update corresponding neighbor entry (the "NeighborInfo_t" represents the "freshest" information about a neighbor)
    FlockingHeader fl_hdr;
    packet->PeekHeader(fl_hdr);

    Vector neighbor_position = fl_hdr.GetPosition();
    Vector neighbor_velocity = fl_hdr.GetVelocity();

    // It might be the first time we see this agent, add it to the map
    std::lock_guard<std::mutex> lock(agents_mutex_);
    if (this->agents_[nodeId].neighbors.find(peer_id) == this->agents_[nodeId].neighbors.end())
    {
        this->agents_[nodeId].neighbors[peer_id] = NeighborInfo_t();
    }

    this->agents_[nodeId].neighbors[peer_id].position = Eigen::Vector3d(neighbor_position.x, neighbor_position.y, neighbor_position.z);
    this->agents_[nodeId].neighbors[peer_id].velocity = Eigen::Vector3d(neighbor_velocity.x, neighbor_velocity.y, neighbor_velocity.z);
    this->agents_[nodeId].neighbors[peer_id].id = peer_id;
    this->agents_[nodeId].neighbors[peer_id].last_seen = rxTime.ToInteger(Time::US);
    this->agents_[nodeId].neighbors[peer_id].link_quality = rxPow;
    this->agents_[nodeId].neighbors[peer_id].link_type = LinkType::FlockingLink;

    this->agents_[nodeId].heartbeat_received++;

    this->broadcast_received_packets[nodeId][peer_id].push_back(rxTime);
}

/**
 * @brief Callback for the transmission of a packet at the application layer
 */
void Ns3Sim::flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet)
{
    Time txTime = Simulator::Now();

    RCLCPP_DEBUG(this->get_logger(), "[%d] Flocking broadcast packet sent", std::stoi(context));
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

    // Since we will manipulate the agents_ object, lock its mutex
    std::lock_guard<std::mutex> lock(this->agents_mutex_);

    for (const auto &[agent_id, agent] : this->agents_)
    {
        dancers_update_proto::VelocityHeading *velocity_heading_msg = velocity_headings_msg.add_velocity_heading();
        velocity_heading_msg->set_agentid(agent.id);
        velocity_heading_msg->set_vx(agent.cmd_velocity[0]);
        velocity_heading_msg->set_vy(agent.cmd_velocity[1]);
        velocity_heading_msg->set_vz(agent.cmd_velocity[2]);
        velocity_heading_msg->set_heading(agent.cmd_heading);
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

    // Since we will manipulate the agents_ object, lock its mutex
    std::lock_guard<std::mutex> lock(this->agents_mutex_);

    // Assign roles based on their neighborhood type
    for (const auto &[agent_id, agent] : this->agents_)
    {
        std::vector<NeighborInfo_t> orderedNeighbors;

        // Sort neighbors by descending link quality
        for (auto const &neighbor : agent.neighbors)
        {
            orderedNeighbors.push_back(neighbor.second);
        }
        std::sort(orderedNeighbors.begin(), orderedNeighbors.end(), [](const NeighborInfo_t &a, const NeighborInfo_t &b)
                  { return a.link_quality > b.link_quality; });

        // Create the message for agent i
        dancers_update_proto::OrderedNeighbors *neighbor_msg = ordered_neighbors_msg.add_ordered_neighbors();
        neighbor_msg->set_agentid(agent.id);
        neighbor_msg->set_role(static_cast<dancers_update_proto::OrderedNeighbors_Role>(agent.role)); // When doing these casts, ensure the order of the enum in the protobuf and the NeighborInfo_t are the same
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
    // Since we will manipulate the agents_ object, lock its mutex
    std::lock_guard<std::mutex> lock(this->agents_mutex_);

    for (auto &[agent_id, agent] : this->agents_)
    {
        std::vector<uint32_t> toRemove;
        for (auto const &neighbor : agent.neighbors)
        {
            if ((Simulator::Now() - MicroSeconds(neighbor.second.last_seen)) > MicroSeconds(this->ns3_config.broadcast_timeout))
            {
                toRemove.push_back(neighbor.first);
            }
        }
        for (auto const &neighbor : toRemove)
        {
            agent.neighbors.erase(neighbor);
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
    // Since we will manipulate the agents_ object, lock its mutex
    std::lock_guard<std::mutex> lock(this->agents_mutex_);
    // We access the nodes container, so lock it
    std::lock_guard<std::mutex> lock_nodes(this->nodes_mutex_);

    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < this->nodes.GetN(); j++)
        {
            if (i != j)
            {
                Ptr<MobilityModel> mobilityA = this->nodes.Get(i)->GetObject<MobilityModel>();
                Ptr<MobilityModel> mobilityB = this->nodes.Get(j)->GetObject<MobilityModel>();

                double rxPow = this->m_propagationLossModel->CalcRxPower(18.0, mobilityA, mobilityB);
                uint32_t agent_i_id, agent_j_id;
                for (const auto &[id, agent] : this->agents_)
                {
                    if (agent.node_container_index == i)
                    {
                        agent_i_id = id;
                    }
                    if (agent.node_container_index == j)
                    {
                        agent_j_id = id;
                    }
                }
                this->pathlosses[agent_i_id][agent_j_id] = rxPow; // it's not really a pathloss but it's the same with a constant difference
                // std::cout << "Node " << i << " -> Node " << j << " : " << rxPow << std::endl;
            }
        }
    }
}

void Ns3Sim::updateStaticIpv4Routes(const std::vector<int> path)
{
    int sink = path[0];
    Ipv4Address sink_addr = this->nodes.Get(this->agents_[sink].node_container_index)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress();
    for (int i = 1; i < path.size(); i++)
    {
        int source = path[i];
        int next_hop = path[i - 1];

        Ipv4Address next_hop_addr = this->nodes.Get(this->agents_[next_hop].node_container_index)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress();

        Ptr<Ipv4StaticRouting> staticRouting = DynamicCast<Ipv4StaticRouting>(this->nodes.Get(this->agents_[source].node_container_index)->GetObject<Ipv4>()->GetRoutingProtocol());
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

void Ns3Sim::updateLeadersApplications()
{
    if (this->ns3_config.enable_mission_flow == false)
    {
        // Nothing to do, the mission flow is disabled
        return;
    }
    for (const auto &target : this->ns3_config.target_areas)
    {
        if (target.is_sink == true)
        {
            // Nothing to update, the sink is not sending data
            continue;
        }

        for (auto assigned_agent : target.assigned_agents)
        {
            uint32_t idx = this->getNodeIndexFromId(assigned_agent);
            Ptr<ns3::Node> node = this->getNodePtrFromIndex(idx);

            Ptr<Application> application;
            bool found = false;
            for (uint32_t i = 0; i < node->GetNApplications(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "Searching Sender application, looking at app %d", i);
                application = node->GetApplication(i);
                if (application == nullptr)
                {
                    RCLCPP_FATAL(this->get_logger(), "Application %d on node %d is null, this should not happen!", i, assigned_agent);
                    exit(EXIT_FAILURE);
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "Found application "<< application->GetInstanceTypeId().GetName() <<" on node " << assigned_agent);
                if (application->GetInstanceTypeId().GetName() == "Sender")
                {
                    found = true;
                    double distance;
                    {
                        std::lock_guard<std::mutex> lock(this->agents_mutex_);
                        distance = (Eigen::Vector3d(target.x, target.y, target.z) - this->agents_[assigned_agent].position).norm();
                    }
                    if (distance < 5.0)
                    {
                        application->SetAttribute("InTargetArea", BooleanValue(true));
                        RCLCPP_DEBUG(this->get_logger(), "Leader %d is in its target area, switching its Sender application to in_target_area", assigned_agent);
                    }
                    else
                    {
                        application->SetAttribute("InTargetArea", BooleanValue(false));
                        RCLCPP_DEBUG(this->get_logger(), "Leader %d is at %f meters of its assigned target area.", assigned_agent, distance);
                    }
                }
            }
            if (!found)
            {
                RCLCPP_WARN(this->get_logger(), "Could not find Sender application on node %d, cannot update its target area status", assigned_agent);
            }
        }
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

                // // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                // // Then, update the node's positions (orientation is ignored for now)
                // if (this->nodes.GetN() != (uint32_t)robots_positions_msg.pose_size())
                // {
                //     RCLCPP_WARN_THROTTLE(this->get_logger(),
                //                             clock,
                //                             1000, // ms
                //                             "Network simulator received position information of %i robots but NS-3 has %u nodes.",
                //                             robots_positions_msg.pose_size(),
                //                             this->nodes.GetN());
                // }
                // else
                // {
                this->agents_mutex_.lock();
                for (const auto &robot_pose : robots_positions_msg.pose())
                {
                    this->agents_[robot_pose.agent_id()].position[0] = robot_pose.x();
                    this->agents_[robot_pose.agent_id()].position[1] = robot_pose.y();
                    this->agents_[robot_pose.agent_id()].position[2] = robot_pose.z();

                    this->agents_[robot_pose.agent_id()].velocity[0] = robot_pose.vx();
                    this->agents_[robot_pose.agent_id()].velocity[1] = robot_pose.vy();
                    this->agents_[robot_pose.agent_id()].velocity[2] = robot_pose.vz();

                    if (this->ns3_config.use_localization_noise)
                    {
                        std::normal_distribution<double> distribution(0.0, this->ns3_config.localization_noise_stddev);
                        this->agents_[robot_pose.agent_id()].position[0] += distribution(this->random_generator);
                        this->agents_[robot_pose.agent_id()].position[1] += distribution(this->random_generator);
                        this->agents_[robot_pose.agent_id()].position[2] += distribution(this->random_generator);
                    }

                    // Set (ns-3) node position
                    Vector pos, vel;
                    pos.x = this->agents_[robot_pose.agent_id()].position.x();
                    pos.y = this->agents_[robot_pose.agent_id()].position.y();
                    pos.z = this->agents_[robot_pose.agent_id()].position.z();
                    vel.x = this->agents_[robot_pose.agent_id()].velocity.x();
                    vel.y = this->agents_[robot_pose.agent_id()].velocity.y();
                    vel.z = this->agents_[robot_pose.agent_id()].velocity.z();
                    uint32_t node_index = this->agents_[robot_pose.agent_id()].node_container_index;
                    Ptr<ConstantVelocityMobilityModel> mobility;
                    {
                        std::lock_guard<std::mutex> lock(this->nodes_mutex_);
                        mobility = this->nodes.Get(node_index)->GetObject<ConstantVelocityMobilityModel>();
                    }
                    mobility->SetPosition(pos);
                    mobility->SetVelocity(vel);
                }
                this->agents_mutex_.unlock();
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
            }

            if (this->ns3_config.wifi_phy_type == "YansWifiPhy")
            {
                this->updateNeighborsPathloss();
            }

            // Update the applications installed on the Leaders. If a leader has reached its target area, it sends this information to the sink.
            this->updateLeadersApplications();

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

            // // Print the current agents' IDs and corresponding ns-3 index, the real ns-3 index and the number of references to the pointer of the node:
            // this->agents_mutex_.lock();
            // RCLCPP_DEBUG(this->get_logger(), "Current agents:");
            // for (const auto &[agent_id, agent] : this->agents_)
            // {
            //     RCLCPP_DEBUG(this->get_logger(), "Agent %d: ns-3 index %d ; real ns-3 index %d ; Refcount %d",
            //                  agent_id, agent.node_container_index, this->nodes.Get(agent.node_container_index)->GetId(), this->nodes.Get(agent.node_container_index)->GetReferenceCount());
            // }
            // this->agents_mutex_.unlock();

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

void Ns3Sim::printNodeAggregatedObjects(Ptr<ns3::Node> node)
{
    // Print all the objects aggregated to the Node
    Object::AggregateIterator it = node->GetAggregateIterator();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Aggregated objects on Node " << node->GetId() << ":");
    // Iterate through the aggregated objects
    while (it.HasNext())
    {
        Ptr<const Object> aggregatedObject = it.Next();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "  - TypeId: " << aggregatedObject->GetInstanceTypeId().GetName());
    }
}

uint32_t Ns3Sim::getNodeIndexFromId(uint32_t nodeId)
{
    std::lock_guard<std::mutex> lock(this->agents_mutex_);
    if (this->agents_.find(nodeId) == this->agents_.end())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Node " << nodeId << " not found in agents_");
        return -1;
    }
    uint32_t idx = this->agents_[nodeId].node_container_index;
    return idx;
}

Ptr<ns3::Node> Ns3Sim::getNodePtrFromIndex(uint32_t nodeIndex)
{
    std::lock_guard<std::mutex> lock(this->nodes_mutex_);
    if (nodeIndex >= this->nodes.GetN())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Requested node index " << nodeIndex << " is out of bound for NodesContainer of size " << this->nodes.GetN() << ")");
        return nullptr;
    }
    Ptr<ns3::Node> node_ptr = this->nodes.Get(nodeIndex);
    return node_ptr;
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

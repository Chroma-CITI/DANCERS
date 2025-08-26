#include <iostream>
#include <utility>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <uav_system.hpp>
#include <VAT_flocking_controller.hpp>
#include <planned_flight.hpp>
#include <udp_tcp_socket.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <dancers_msgs/msg/velocity_heading_array.hpp>
#include <dancers_msgs/srv/get_agent_velocities.hpp>
#include <dancers_msgs/msg/target.hpp>
#include <dancers_msgs/msg/agent_struct_array.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <protobuf_msgs/dancers_update.pb.h>
#include <protobuf_msgs/ordered_neighbors.pb.h>
#include <protobuf_msgs/pose_vector.pb.h>
#include <protobuf_msgs/velocity_heading_vector.pb.h>

#include "boost/filesystem.hpp"

#include <Eigen/Core>

#include <util.hpp>
#include <time_probe.hpp>

#include <yaml-cpp/yaml.h>

using namespace mrs_multirotor_simulator;
using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief The MiniDancers ROS2 node, part of the DANCERS co-simulator
 *
 * This class is ROS2 node that is the main node for Mini-Dancers, a multi-UAV simulator specifically made to work with the DANCERS co-simulator. You can see this node as a ROS2 wrapper around the multi-rotor model created by the CTU in Prague : https://github.com/ctu-mrs/mrs_uav_system.
 */
class MiniDancers : public rclcpp::Node
{
public:
    MiniDancers() : Node("mini_dancers")
    {
        /* ----------- Read Configuration file ----------- */

        // Declare the config_file ROS2 parameter
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        param_desc.description = "Executes in co-simulation or mini-dancers only.";
        this->declare_parameter("cosim_mode", true, param_desc);

        this->cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

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
                temp_path = ros_ws_path + "/data/" + experience_name + "/physics" + std::to_string(i) + ".csv";
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

        this->save_collisions_number = config["save_collisions_number"].as<bool>();
        if (this->save_collisions_number)
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
            while (this->m_collisions_file.empty())
            {
                temp_path = ros_ws_path + "/data/" + experience_name + "/collisions_run_" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    this->m_collisions_file = temp_path;
                }
            }
            // initialize the output file with headers
            std::ofstream file;
            file.open(this->m_collisions_file.c_str(), std::ios::out);
            file << "agentId,obstacleCollisions,uavCollisions" << std::endl;
            file.close();
        }

        /* ----------- Configuration file parsing ----------- */

        for (auto building : config["buildings"])
        {
            double x = building["x"].as<double>();
            double y = building["y"].as<double>();
            double z = building["height"].as<double>() / 2;
            double size_x = building["size_x"].as<double>();
            double size_y = building["size_y"].as<double>();
            double height = building["height"].as<double>();

            obstacle_t obs{};
            obs.id = building["id"].as<int>();
            obs.center = Eigen::Vector3d(x, y, z);
            obs.size_x = size_x;
            obs.size_y = size_y;
            obs.size_z = height;
            this->obstacles.push_back(obs);
        }
        this->publish_agent_structs = config["publish_agent_structs"].as<bool>();
        this->publish_network_markers = config["publish_network_markers"].as<bool>();
        this->n_uavs = config["robots_number"].as<int>();
        this->start_position = std::make_tuple(config["start_position"]["x"].as<double>(), config["start_position"]["y"].as<double>(), config["start_position"]["z"].as<double>());
        int sync_window_int = config["sync_window"].as<int>();
        int step_size_int = config["phy_step_size"].as<int>();
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the network step size, aborting.");
            exit(EXIT_FAILURE);
        }
        this->sync_window = sync_window_int / 1000000.0f; // us to s
        this->step_size = step_size_int / 1000000.0f; 
        this->it = 0;
        this->simulation_length = config["simulation_length"].as<double>();
        this->it_end_sim = uint64_t(simulation_length / this->step_size);
        this->print_sim_advancement_ = config["mini_dancers_print_advancement"].as<bool>();
        this->mission_flow_id = config["mission_flow"]["flow_id"].as<uint32_t>();
        this->potential_flow_id = config["broadcast_flow"]["flow_id"].as<uint32_t>();
        this->mode = config["cosimulation_mode"].as<std::string>();
        this->communication_range = config["communication_range"].as<double>();
        this->K_penetration_force = config["K_penetration_force"].as<double>();
        this->K_restitution_force = config["K_restitution_force"].as<double>();
        this->K_agent_penetration_force = config["K_agent_penetration_force"].as<double>();
        this->K_agent_restitution_force = config["K_agent_restitution_force"].as<double>();
        this->agent_radius = config["agent_radius"].as<double>();
        for (auto goal : config["secondary_objectives"])
        {
            target_t target;
            target.position = Eigen::Vector3d(goal["position"]["x"].as<double>(), goal["position"]["y"].as<double>(), goal["position"]["z"].as<double>());
            target.is_sink = goal["is_sink"].as<bool>();
            for (auto agent : goal["assigned_agents"])
            {
                target.assigned_agents.push_back(agent.as<int>());
            }
            this->target_areas[goal["id"].as<int>()] = target;
            RCLCPP_INFO(this->get_logger(), "Target area %d created at (%f,%f,%f)", goal["id"].as<int>(), target.position.x(), target.position.y(), target.position.z());
        }

        try
        {
            this->vat_params.v_flock = config["VAT_mission_flocking_parameters"]["v_flock"].as<double>();
            this->vat_params.v_max = config["VAT_mission_flocking_parameters"]["v_max"].as<double>();
            this->vat_params.a_frict = config["VAT_mission_flocking_parameters"]["a_frict"].as<double>();
            this->vat_params.p_frict = config["VAT_mission_flocking_parameters"]["p_frict"].as<double>();
            this->vat_params.r_0_frict = config["VAT_mission_flocking_parameters"]["r_0_frict"].as<double>();
            this->vat_params.C_frict = config["VAT_mission_flocking_parameters"]["C_frict"].as<double>();
            this->vat_params.v_frict = config["VAT_mission_flocking_parameters"]["v_frict"].as<double>();
            this->vat_params.p_att = config["VAT_mission_flocking_parameters"]["p_att"].as<double>();
            this->vat_params.r_0_att = config["VAT_mission_flocking_parameters"]["r_0_att"].as<double>();
            this->vat_params.p_rep = config["VAT_mission_flocking_parameters"]["p_rep"].as<double>();
            this->vat_params.r_0_rep = config["VAT_mission_flocking_parameters"]["r_0_rep"].as<double>();
            this->vat_params.a_shill = config["VAT_mission_flocking_parameters"]["a_shill"].as<double>();
            this->vat_params.p_shill = config["VAT_mission_flocking_parameters"]["p_shill"].as<double>();
            this->vat_params.r_0_shill = config["VAT_mission_flocking_parameters"]["r_0_shill"].as<double>();
            this->vat_params.v_shill = config["VAT_mission_flocking_parameters"]["v_shill"].as<double>();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            std::cerr << "Failed to read at least one VAT flocking parameter, using ALL default VAT params." << '\n';
        }

        this->phy_use_uds = config["phy_use_uds"].as<bool>();
        this->phy_uds_server_address = config["phy_uds_server_address"].as<std::string>();
        this->phy_ip_server_address = config["phy_ip_server_address"].as<std::string>();
        this->phy_ip_server_port = config["phy_ip_server_port"].as<unsigned int>();

        rclcpp::QoS qos_persistent = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

        rclcpp::QoS reliable_qos(10); // depth of 10
        reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        reliable_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        reliable_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);


        /* ----------- Publishers ----------- */
        this->obstacles_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", qos_persistent);
        this->pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("agent_poses", 10);
        if (this->publish_agent_structs)
        {
            this->agent_structs_pub_ = this->create_publisher<dancers_msgs::msg::AgentStructArray>("agent_structs", 10);
        }
        this->id_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("id_markers", 10);
        this->desired_velocities_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("desired_velocities", 10);
        if (this->publish_network_markers)
        {
            this->network_mission_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_mission_links", 10);
            this->network_potential_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_potential_links", 10);
            this->network_idle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_idle_links", 10);
        }
        this->target_areas_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_areas", 10);
        this->base_station_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("base_station", 10);

        /* ----------- Subscribers ----------- */
        std::string events_namespace = "/events/";
        this->targets_sub_ = this->create_subscription<dancers_msgs::msg::Target>(
            events_namespace+"update_target", 
            reliable_qos, 
            std::bind(&MiniDancers::update_target_clbk, this, _1));
        this->spawn_uav_sub_ = this->create_subscription<dancers_msgs::msg::AgentStruct>(
            events_namespace+"spawn_uav", 
            reliable_qos, 
            std::bind(&MiniDancers::spawn_uav_clbk, this, _1));
        this->uav_failure_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            events_namespace+"uav_failure", 
            reliable_qos, 
            std::bind(&MiniDancers::uav_failure_clbk, this, _1));


        /* ----------- Service client ----------- */
        command_client_ = this->create_client<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities");

        this->InitObstacles();

        this->InitUavs();

        // Colours are fun: "\x1b[32m" enables red color ; "\x1b[0m" set default back
        RCLCPP_INFO(this->get_logger(), "\x1b[32mMiniDancers connector initialized. Ready to start simulation !\x1b[0m");

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&MiniDancers::Loop, this);
    }

    ~MiniDancers()
    {
        if (loop_thread_.joinable())
        {
            loop_thread_.join();
            RCLCPP_INFO(this->get_logger(), "MiniDancers Node Destroyed");
        }
    }

private:
    /* Time and iteration counters */
    double sync_window;             // s
    double step_size;               // s
    double simulation_length;       // s
    uint64_t it;                    // -
    uint64_t it_end_sim;            // -

    /* Obstacles */
    std::vector<obstacle_t> obstacles;
    
    /* UAV-related */
    int n_uavs;
    std::map<uint32_t, agent_t> uavs;
    std::mutex uavs_mutex;
    std::map<uint32_t, Eigen::Vector3d> desired_velocities;
    double communication_range;
    std::tuple<double, double, double> start_position;
    double K_penetration_force;
    double K_restitution_force;
    double K_agent_penetration_force;
    double K_agent_restitution_force;
    double agent_radius;
    VAT_params_t vat_params;
    std::map<int, target_t> target_areas;
    double target_altitude;
    
    /* Software machinery */
    bool print_sim_advancement_;
    bool phy_use_uds;
    std::string phy_uds_server_address;
    std::string phy_ip_server_address;
    unsigned int phy_ip_server_port;
    std::string ros_ws_path;
    bool cosim_mode;
    std::string mode;

    /* Network related */
    uint32_t mission_flow_id;
    uint32_t potential_flow_id;
    uint32_t routing_flow_id;

    /* Thread running the co-simulation Loop */
    std::thread loop_thread_;

    /* Methods */
    void InitObstacles();
    void InitUavs();
    void ApplyObstacleCollisions();
    void ApplyUavCollisions();
    void DisplayRviz();
    void UpdateCmds();
    void GetNeighbors(dancers_update_proto::DancersUpdate &network_update_msg);
    void GetCommands(dancers_update_proto::DancersUpdate &network_update_msg);
    int GetAgentIndex(uint32_t agent_id);
    target_t* GetAgentTarget(uint32_t agent_id);
    void GenerateRolesAndNeighbors();
    std::string GenerateResponseProtobuf();
    void Loop();

    void MakeStep(uint32_t agent_id);

    /* Publishers */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<dancers_msgs::msg::AgentStructArray>::SharedPtr agent_structs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr id_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr desired_velocities_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_mission_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_potential_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_idle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_areas_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr base_station_pub_;

    /* Subscribers */
    rclcpp::Subscription<dancers_msgs::msg::Target>::SharedPtr targets_sub_;
    void update_target_clbk(dancers_msgs::msg::Target msg);

    rclcpp::Subscription<dancers_msgs::msg::AgentStruct>::SharedPtr spawn_uav_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr uav_failure_sub_;
    void spawn_uav_clbk(dancers_msgs::msg::AgentStruct msg);
    void uav_failure_clbk(std_msgs::msg::UInt32 msg);

    /* Service client*/
    rclcpp::Client<dancers_msgs::srv::GetAgentVelocities>::SharedPtr command_client_;

    /* Collisions number saving */
    bool save_collisions_number;
    std::string m_collisions_file;

    /* Compute time saving */
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;

    /* Publish options */
    bool publish_agent_structs;
    bool publish_network_markers;
};

void MiniDancers::update_target_clbk(dancers_msgs::msg::Target msg)
{
    target_t target;
    target.position = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    target.is_sink = msg.is_sink;
    target.assigned_agents = msg.concerned_agents;

    this->target_areas[msg.target_id] = target;

    for (int agent_id : msg.concerned_agents)
    {
        if (agent_id == -1)
        {
            for (auto &agent : this->uavs)
            {
                agent.second.secondary_objective = target.position;
            }
            break;
        }
        else if (this->uavs.find(agent_id) != this->uavs.end())
        {
            this->uavs[agent_id].secondary_objective = target.position;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Agent %d does not exist, skipping assignment of its secondary objective.", agent_id);
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Updated target %d at (%f,%f,%f).", msg.target_id, target.position.x(), target.position.y(), target.position.z());
}

void MiniDancers::spawn_uav_clbk(dancers_msgs::msg::AgentStruct msg)
{
    std::lock_guard<std::mutex> lock(this->uavs_mutex);

    // Check that there is not already a UAV with the same ID
    if (this->uavs.find(msg.agent_id) != this->uavs.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Requested UAV spawn Agent with ID %d rejected: already exists.", msg.agent_id);
        return;
    }
    
    // Create a new agent_t from the received AgentStruct message
    agent_t new_agent;
    new_agent.id = msg.agent_id;
    new_agent.uav_system = UavSystem(MultirotorModel::ModelParams(), Eigen::Vector3d(msg.state.position.x, msg.state.position.y, msg.state.position.z), msg.state.velocity_heading.heading);
    new_agent.neighbors = std::vector<NeighborInfo_t>();
    // Is this agent assigned to a target ?
    target_t* target = this->GetAgentTarget(msg.agent_id);
    if (target != nullptr)
    {
        new_agent.secondary_objective = target->position;
    }
    this->desired_velocities[msg.agent_id] = Eigen::Vector3d::Zero();
    this->uavs[msg.agent_id] = new_agent;
    this->n_uavs += 1;

    RCLCPP_INFO(this->get_logger(), "Spawned a new UAV with ID %d", new_agent.id);
}

void MiniDancers::uav_failure_clbk(std_msgs::msg::UInt32 msg)
{
    // CTU's uav_system has a "crash" feature, use it in case of failure.
    std::lock_guard<std::mutex> lock(this->uavs_mutex);
    // verify that the UAV is known:
    if (this->uavs.find(msg.data) == this->uavs.end())
    {
        RCLCPP_ERROR(this->get_logger(), "UAV with ID %d does not exist, cannot crash it.", msg.data);
        return;
    }
    this->uavs[msg.data].uav_system.crash();
    RCLCPP_INFO(this->get_logger(), "UAV %d has crashed.", msg.data);
}

/**
 * @brief Initialize the obstacles in Rviz
 *
 * The obstacles are only displayed once, at the beginning of the simulation. If Rviz was not started at this time, the obstacles will not appear.
 */
void MiniDancers::InitObstacles()
{
    // Display obstacles (once)
    for (obstacle_t obs : this->obstacles)
    {
        std::vector<triangle> triangles = obstacle_to_triangles_list(obs);
        visualization_msgs::msg::Marker obstacle_marker{};
        obstacle_marker.header.frame_id = "map";
        obstacle_marker.id = obs.id;
        obstacle_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
        obstacle_marker.scale.x = 1.0; // Set the desired width of the lines
        obstacle_marker.scale.y = 1.0;
        obstacle_marker.scale.z = 1.0;
        obstacle_marker.pose.position.x = 0.0;
        obstacle_marker.pose.position.y = 0.0;
        obstacle_marker.pose.position.z = 0.0;
        // Set the color (RGBA)
        obstacle_marker.color.r = 0.0;
        obstacle_marker.color.g = 1.0; // Green color
        obstacle_marker.color.b = 0.0;
        obstacle_marker.color.a = 0.8; // Fully opaque

        // Set the lifetime of the marker (0 means it stays until removed)
        obstacle_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        for (triangle tri : triangles)
        {
            geometry_msgs::msg::Point p1, p2, p3;
            p1.x = tri.p1.x();
            p1.y = tri.p1.y();
            p1.z = tri.p1.z();
            p2.x = tri.p2.x();
            p2.y = tri.p2.y();
            p2.z = tri.p2.z();
            p3.x = tri.p3.x();
            p3.y = tri.p3.y();
            p3.z = tri.p3.z();
            obstacle_marker.points.push_back(p1);
            obstacle_marker.points.push_back(p2);
            obstacle_marker.points.push_back(p3);
        }
        this->obstacles_pub_->publish(obstacle_marker);
    }
}

/**
 * @brief Initialize the UAV models
 *
 * This function initializes the vectors related to the UAVs, including their dynamic model (UavSystem) with initial positions in a grid, their associated struct (agent_t), and their initial desired velocity (0)
 */
void MiniDancers::InitUavs()
{
    int n_columns = (int)sqrt(this->n_uavs);
    double spacing = 5.0;
    double grid_x_init = std::get<0>(this->start_position);
    double grid_y_init = std::get<1>(this->start_position) - ((n_columns - 1) * spacing) / 2;
    double grid_z_init = std::get<2>(this->start_position);
    for (int i = 0; i < this->n_uavs; i++)
    {
        // Grid spawn
        double spawn_x = grid_x_init + i / n_columns * spacing;
        double spawn_y = grid_y_init + spacing * (i % n_columns);
        double spawn_z = grid_z_init;
        double spawn_heading = 0.0;

        // Already in line robots
        // double spawn_x = (rand() % 200 - 100) / 100.0;
        // double spawn_y = i * 50.0;
        // double spawn_z = 1.0;
        // double spawn_heading = 0.0;

        // Default ModelParams is the x500 configuration
        MultirotorModel::ModelParams x500_params = MultirotorModel::ModelParams();
        // Enable ground
        x500_params.ground_enabled = true;
        UavSystem uav_system = UavSystem(x500_params, Eigen::Vector3d(spawn_x, spawn_y, spawn_z), spawn_heading);

        agent_t agent;
        agent.id = i;
        agent.uav_system = uav_system;
        agent.neighbors = std::vector<NeighborInfo_t>();
        target_t* assigned_target = this->GetAgentTarget(i);
        if (assigned_target != nullptr)
        {
            agent.secondary_objective = assigned_target->position;
        }
        this->desired_velocities[i] = Eigen::Vector3d::Zero();
        this->uavs[i] = agent;
    }
}

void MiniDancers::ApplyUavCollisions()
{
    const double EPSILON = 1e-9;
    const double restitution_coefficient = 1.0;

    for (size_t i = 0; i < this->uavs.size(); i++)
    {
        for (size_t j = i+1 ; j < this->uavs.size(); j++)
        {
            agent_t& agent1 = this->uavs[i];
            agent_t& agent2 = this->uavs[j];

            const double agent_mass = agent1.uav_system.getParams().mass;

            // 1. Calculate the vector connecting the centers of the two agents.
            Eigen::Vector3d delta_pos = agent2.uav_system.getState().x - agent1.uav_system.getState().x;

            double distance = delta_pos.norm();
            double minDist = this->agent_radius + this->agent_radius;

            if (distance >= minDist) return; // No collision

            // --- Determine the Collision Normal ---
            // The collision normal points from agent2 to agent1.
            Eigen::Vector3d collision_normal;
            if (distance < EPSILON) {
                // If agents are exactly at the same position, use a default normal
                // For simplicity, we'll use a unit X vector as a fallback.
                collision_normal = Eigen::Vector3d::UnitX();
            } else {
                collision_normal = delta_pos.normalized(); // Normalize the vector to get the direction
            }

            // --- Calculate Penetration Depth ---
            // How far the agents are overlapping.
            double penetration_depth = minDist - distance;

            Eigen::Vector3d penetration_force_1 = Eigen::Vector3d::Zero();
            Eigen::Vector3d penetration_force_2 = Eigen::Vector3d::Zero();

            // --- Component 1: Penetration Force (Spring Force Model) ---
            // Applies a force to push agents out of overlap, proportional to penetration depth.
            // This replaces the direct position correction. The force is applied equally
            // and oppositely, and the effect on each agent's acceleration will naturally
            // be inversely proportional to its mass via agent.applyForce().
            if (penetration_depth > EPSILON) {
                Eigen::Vector3d penetration_force = K_agent_penetration_force * penetration_depth * collision_normal;
                penetration_force_1 += penetration_force;
                penetration_force_2 -= penetration_force;
            }
    
            double agent1_velocity_along_normal = agent1.uav_system.getState().v.dot(collision_normal);
            double agent2_velocity_along_normal = agent2.uav_system.getState().v.dot(collision_normal);
            
            std::cout << "Agent1:\n  Velocity: " << agent1.uav_system.getState().v.transpose() << "\n  Velocity(normal): "  << agent1_velocity_along_normal << std::endl;
            std::cout << "Agent2:\n  Velocity: " << agent2.uav_system.getState().v.transpose() << "\n  Velocity(normal): "  << agent2_velocity_along_normal << std::endl;

            // Only process if spheres are moving toward each other
            if (agent1_velocity_along_normal >= agent2_velocity_along_normal) return;

            // Compute impulse assuming unit mass
            double impulseMag = this->K_agent_restitution_force * (agent2_velocity_along_normal - agent1_velocity_along_normal);
            Eigen::Vector3d impulse = impulseMag * collision_normal;

            // Convert impulse to force: F = J / dt
            Eigen::Vector3d force = impulse / this->step_size;

            agent1.uavs_collisions += 1;
            agent2.uavs_collisions += 1;

            RCLCPP_INFO(this->get_logger(), "Agents %d and %d collide! (dist=%f)\n    [%d]: %d\n    [%d]: %d\n    (%f,%f,%f)", agent1.id, agent2.id, distance,
            agent1.id, agent1.uavs_collisions, 
            agent2.id, agent2.uavs_collisions,
            force.x(), force.y(), force.z());
            // Apply the impulse forces.
            // Agent2 gets an equal and opposite force.
            agent1.uav_system.applyForce(force  + penetration_force_1);
            agent2.uav_system.applyForce(-force + penetration_force_2); // Newton's third law: equal and opposite reaction
        }
    }
}

void MiniDancers::ApplyObstacleCollisions()
{
    const double EPSILON = 1e-9;
    const double AGENT_RADIUS = this->agent_radius;
    const double K_penetration_force = this->K_penetration_force; // Force to push out of penetration
    const double K_restitution_force = this->K_restitution_force; // Force to achieve elastic bounce

    for (auto& [agent_id, agent] : this->uavs)
    {

        Eigen::Vector3d agent_position = agent.uav_system.getState().x;
        Eigen::Vector3d agent_velocity = agent.uav_system.getState().v;

        for (auto const& obstacle : this->obstacles)
        {
            // 1. Calculate the min and max corners of the current AABB obstacle.
            Eigen::Vector3d min_corner = obstacle.getMinCorner();
            Eigen::Vector3d max_corner = obstacle.getMaxCorner();

            // 2. Find the closest point on the AABB to the agent's center.
            // This is done by clamping the agent's position coordinates to the AABB's bounds.
            Eigen::Vector3d closest_point;
            closest_point.x() = std::max(min_corner.x(), std::min(agent_position.x(), max_corner.x()));
            closest_point.y() = std::max(min_corner.y(), std::min(agent_position.y(), max_corner.y()));
            closest_point.z() = std::max(min_corner.z(), std::min(agent_position.z(), max_corner.z()));

            // 3. Calculate the vector from the closest point on the AABB to the agent's center.
            // This vector points from the obstacle surface towards the agent.
            Eigen::Vector3d delta = agent_position - closest_point;

            // 4. Calculate the squared distance between the agent's center and the closest point.
            double distance_sq = delta.squaredNorm();
            // Calculate the squared radius for faster comparison, avoiding sqrt if not necessary.
            double agent_radius_sq = AGENT_RADIUS * AGENT_RADIUS;

            // 5. Check for collision: A collision occurs if the squared distance is less than
            //    the squared radius of the agent.
            if (distance_sq < agent_radius_sq) {
                double distance = std::sqrt(distance_sq); // Actual distance for penetration calculation

                // --- Determine the Collision Normal ---
                // The collision normal is the direction perpendicular to the collision surface.
                // It points from the obstacle towards the agent.
                Eigen::Vector3d collision_normal;
                if (distance < EPSILON) {
                    // If the agent is exactly at the closest point (or very deeply inside),
                    // 'delta' will be near zero, making normalization problematic.
                    // In such cases, we approximate the normal. A robust solution might
                    // involve finding the closest AABB face or using the agent's incoming velocity.
                    // Here, if moving, use opposite velocity. If stuck, use +Z as a fallback.
                    if (agent_velocity.norm() > EPSILON) {
                        collision_normal = -agent_velocity.normalized();
                    } else {
                        // Fallback: If agent is completely stuck and not moving, push it up.
                        // This might cause jitter but prevents division by zero.
                        collision_normal = Eigen::Vector3d::UnitZ();
                    }
                } else {
                    collision_normal = delta.normalized(); // Normalize the delta vector to get the direction
                }

                // --- Calculate Penetration Depth ---
                // How far the agent has "sunk" into the obstacle.
                double penetration_depth = AGENT_RADIUS - distance;

                // --- Calculate Total Elastic Collision Force ---
                // The total force is a sum of a repulsion force (to push the agent out)
                // and a restitution force (to make it bounce).
                Eigen::Vector3d total_collision_force = Eigen::Vector3d::Zero();

                // Component 1: Repulsion force
                // This force is proportional to the penetration depth and acts along the collision normal.
                // It pushes the agent out of the obstacle.
                total_collision_force += K_penetration_force * penetration_depth * collision_normal;

                // Component 2: Restitution force (for elastic bounce)
                // This force reverses the component of the agent's velocity that is directed
                // towards the obstacle. Only apply if the agent is actually moving into the obstacle.
                double normal_velocity_component = agent_velocity.dot(collision_normal);

                // If the agent's normal velocity component is negative, it means it's moving towards
                // (or into) the obstacle.
                if (normal_velocity_component < 0) {
                    // For a perfect elastic collision (coefficient of restitution = 1),
                    // the change in normal velocity should be -2 * normal_velocity_component.
                    // The force needed to achieve this velocity change over time 'dt' is:
                    // F = mass * (delta_v / dt)
                    // Here, we use K_restitution_force as a scaling factor that absorbs 'mass/dt'
                    // for a simpler formulation, effectively making it an impulse-like force.
                    total_collision_force += -2.0 * K_restitution_force * normal_velocity_component * collision_normal;
                }

                agent.obstacles_collisions += 1;
                RCLCPP_INFO(this->get_logger(), "[%d] Obstacle collision! #collisions=%d", agent.id, agent.obstacles_collisions);
                agent.uav_system.applyForce(total_collision_force);
            }
        }
    }

}



/**
 * @brief Display every dynamic object in Rviz (UAVs, targets, network links)
 *
 * Displaying an object in Rviz is done by publishing a marker. The marker is created here and published here, every time this function is called.
 */
void MiniDancers::DisplayRviz()
{
    // Display robots
    geometry_msgs::msg::PoseArray pose_array{};
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->get_clock()->now();
    visualization_msgs::msg::MarkerArray id_marker_array{};
    visualization_msgs::msg::MarkerArray desired_velocities_marker_array{};
    for (auto agent : this->uavs)
    {
        uint32_t agent_id = agent.first;
        agent_t agent_struct = agent.second;

        // Display poses
        MultirotorModel::State uav_state = agent_struct.uav_system.getState();
        geometry_msgs::msg::Pose pose{};
        pose.position.x = uav_state.x.x();
        pose.position.y = uav_state.x.y();
        pose.position.z = uav_state.x.z();
        Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(uav_state.R);
        pose.orientation.w = quat.w();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose_array.poses.push_back(pose);

        // Display ID markers
        visualization_msgs::msg::Marker id_marker{};
        id_marker.id = agent_id;
        id_marker.header.frame_id = "map";
        id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        id_marker.action = visualization_msgs::msg::Marker::ADD;
        id_marker.color.r = 0.0;
        id_marker.color.g = 0.0;
        id_marker.color.b = 1.0;
        id_marker.color.a = 1.0; // Fully opaque
        id_marker.scale.z = 2.0;
        id_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        id_marker.text = std::to_string(agent_id);
        id_marker.pose.position.x = uav_state.x.x() - 1;
        id_marker.pose.position.y = uav_state.x.y();
        id_marker.pose.position.z = uav_state.x.z();
        id_marker_array.markers.push_back(id_marker);

        // Display desired velocities
        visualization_msgs::msg::Marker desired_velocity_marker{};
        desired_velocity_marker.header.frame_id = "map";
        desired_velocity_marker.id = 2000 + agent_id;
        desired_velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
        desired_velocity_marker.action = visualization_msgs::msg::Marker::ADD;
        desired_velocity_marker.scale.x = 0.3;
        desired_velocity_marker.scale.y = 0.5;
        desired_velocity_marker.scale.z = 0.0;
        desired_velocity_marker.color.r = 0.0;
        desired_velocity_marker.color.g = 1.0;
        desired_velocity_marker.color.b = 0.0;
        desired_velocity_marker.color.a = 0.8;
        desired_velocity_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        desired_velocity_marker.pose.position.x = 0.0;
        desired_velocity_marker.pose.position.y = 0.0;
        desired_velocity_marker.pose.position.z = 0.0;
        geometry_msgs::msg::Point p1, p2;
        p1.x = uav_state.x.x();
        p1.y = uav_state.x.y();
        p1.z = uav_state.x.z();
        p2.x = uav_state.x.x() + this->desired_velocities[agent_id].x();
        p2.y = uav_state.x.y() + this->desired_velocities[agent_id].y();
        p2.z = uav_state.x.z() + this->desired_velocities[agent_id].z();
        desired_velocity_marker.points.push_back(p1);
        desired_velocity_marker.points.push_back(p2);
        desired_velocities_marker_array.markers.push_back(desired_velocity_marker);
    }
    this->pose_array_pub_->publish(pose_array);
    this->id_markers_pub_->publish(id_marker_array);
    this->desired_velocities_markers_pub_->publish(desired_velocities_marker_array);

    // Display Sources
    visualization_msgs::msg::Marker sources_marker{};
    sources_marker.header.frame_id = "map";
    sources_marker.id = 1000;
    sources_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sources_marker.action = visualization_msgs::msg::Marker::ADD;
    sources_marker.scale.x = 5.0;
    sources_marker.scale.y = 5.0;
    sources_marker.scale.z = 5.0;
    sources_marker.pose.position.x = 0.0;
    sources_marker.pose.position.y = 0.0;
    sources_marker.pose.position.z = 0.0;
    sources_marker.color.r = 0.0;
    sources_marker.color.g = 0.1;
    sources_marker.color.b = 1.0;
    sources_marker.color.a = 0.5;
    sources_marker.lifetime = rclcpp::Duration::from_seconds(0);
    for (auto target : this->target_areas)
    {
        target_t secondary_objective = target.second;
        if (secondary_objective.is_sink == false)
        {
            geometry_msgs::msg::Point p{};
            p.x = secondary_objective.position.x();
            p.y = secondary_objective.position.y();
            p.z = secondary_objective.position.z();
            sources_marker.points.push_back(p);
        }
    }
    this->target_areas_pub_->publish(sources_marker);

    // Display Sink
    visualization_msgs::msg::Marker sink_marker{};
    sink_marker.header.frame_id = "map";
    sink_marker.id = 1001;
    sink_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sink_marker.action = visualization_msgs::msg::Marker::ADD;
    sink_marker.scale.x = 5.0;
    sink_marker.scale.y = 5.0;
    sink_marker.scale.z = 5.0;
    sink_marker.color.r = 1.0;
    sink_marker.color.g = 1.0;
    sink_marker.color.b = 0.0;
    sink_marker.color.a = 0.5;
    sink_marker.lifetime = rclcpp::Duration::from_seconds(0);
    for (auto target : this->target_areas)
    {
        target_t secondary_objective = target.second;
        if (secondary_objective.is_sink == true)
        {
            sink_marker.pose.position.x = secondary_objective.position.x();
            sink_marker.pose.position.y = secondary_objective.position.y();
            sink_marker.pose.position.z = secondary_objective.position.z();

            this->base_station_pub_->publish(sink_marker);
        }
    }

    if (this->publish_network_markers)
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

        for (auto agent : this->uavs)
        {
            uint32_t agent_id = agent.first;
            agent_t agent_struct = agent.second;

            Eigen::Vector3d agent_pose(agent_struct.uav_system.getState().x);
            for (size_t j = 0; j < agent_struct.neighbors.size(); j++)
            {
                int neighbor_id = agent_struct.neighbors[j].id;
                if (agent_id == neighbor_id)
                    continue; // (should never happen as we check this in GetNeighbors)
                // Get the "real" position of the neighbor, not the "last position heard" otherwise the visualization is scuffed
                Eigen::Vector3d other_agent_pose(this->uavs[neighbor_id].uav_system.getState().x);

                geometry_msgs::msg::Point p1{};
                p1.x = agent_pose.x();
                p1.y = agent_pose.y();
                p1.z = agent_pose.z();
                geometry_msgs::msg::Point p2{};
                p2.x = other_agent_pose.x();
                p2.y = other_agent_pose.y();
                p2.z = other_agent_pose.z();

                switch (agent_struct.role)
                {
                case (AgentRoleType::Mission):
                    network_marker_mission.points.push_back(p1);
                    network_marker_mission.points.push_back(p2);
                    break;
                case (AgentRoleType::Potential):
                    network_marker_potential.points.push_back(p1);
                    network_marker_potential.points.push_back(p2);
                    break;
                case (AgentRoleType::Idle):
                    network_marker_idle.points.push_back(p1);
                    network_marker_idle.points.push_back(p2);
                    break;
                }
            }
        }

        this->network_mission_pub_->publish(network_marker_mission);
        this->network_potential_pub_->publish(network_marker_potential);
        this->network_idle_pub_->publish(network_marker_idle);
    }
}

/**
 * @brief Updates the internal state of the agents with the new input commands (called at every timestep)
 *
 * This is where the controller code goes ! It uses a ROS2 service to fetch the command from the controller.
 */
void MiniDancers::UpdateCmds()
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

    auto request = std::make_shared<dancers_msgs::srv::GetAgentVelocities::Request>();

    dancers_msgs::msg::AgentStructArray agent_structs;

    for (auto agent : this->uavs)
    {
        dancers_msgs::msg::AgentStruct agent_struct;
        agent_t &self_agent = agent.second;

        agent_struct.agent_id = self_agent.id;
        agent_struct.state.position.x = self_agent.uav_system.getState().x[0];
        agent_struct.state.position.y = self_agent.uav_system.getState().x[1];
        agent_struct.state.position.z = self_agent.uav_system.getState().x[2];

        agent_struct.state.velocity_heading.velocity.x = self_agent.uav_system.getState().v[0];
        agent_struct.state.velocity_heading.velocity.y = self_agent.uav_system.getState().v[1];
        agent_struct.state.velocity_heading.velocity.z = self_agent.uav_system.getState().v[2];

        // Role conversion
        if (self_agent.role == AgentRoleType::Undefined)
        {
            agent_struct.agent_role = agent_struct.AGENT_ROLE_UNDEFINED;
        }
        else if (self_agent.role == AgentRoleType::Mission)
        {
            agent_struct.agent_role = agent_struct.AGENT_ROLE_MISSION;
        }
        else if (self_agent.role == AgentRoleType::Potential)
        {
            agent_struct.agent_role = agent_struct.AGENT_ROLE_POTENTIAL;
        }
        else if (self_agent.role == AgentRoleType::Idle)
        {
            agent_struct.agent_role = agent_struct.AGENT_ROLE_IDLE;
        }

        for (NeighborInfo_t &neighbor : self_agent.neighbors)
        {
            // Get the neighbor's position and velocity 
            // (When MiniDancers invokes the controller, the "real" position of the neighbors are used, there is no notion of erroneous neighbor position knowledge)
            agent_t &neighbor_agent = this->uavs[neighbor.id];
            Eigen::Vector3d neighbor_position(neighbor_agent.uav_system.getState().x);
            Eigen::Vector3d neighbor_velocity(neighbor_agent.uav_system.getState().v);

            dancers_msgs::msg::Neighbor neighbor_msg;
            neighbor_msg.agent_id = neighbor.id;
            neighbor_msg.link_quality = neighbor.link_quality;
            neighbor_msg.agent_role = static_cast<uint8_t>(neighbor.role);
            neighbor_msg.position.x = neighbor_position.x();
            neighbor_msg.position.y = neighbor_position.y();
            neighbor_msg.position.z = neighbor_position.z();
            neighbor_msg.velocity.x = neighbor_velocity.x();
            neighbor_msg.velocity.y = neighbor_velocity.y();
            neighbor_msg.velocity.z = neighbor_velocity.z();

            agent_struct.neighbor_array.neighbors.push_back(neighbor_msg);
        }

        request->agent_structs.push_back(agent_struct);

        if (this->publish_agent_structs)
        {
            agent_structs.structs.emplace_back(std::move(agent_struct));
        }
    }
    this->agent_structs_pub_->publish(agent_structs);

    auto result = command_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // Can't get the reference directly of velocity heading directly since result becomes invalid after the get() call.
        std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> request_msg = result.get();
        std::vector<dancers_msgs::msg::VelocityHeading> &velocity_headings = request_msg->velocity_headings.velocity_heading_array;

        if (velocity_headings.size() != this->uavs.size())
        {
            RCLCPP_WARN(this->get_logger(), "Controller answered with a number of commands (%d) that does not match the number of current UAVs (%d). Trying to modify only the UAVs for which we have a command.", velocity_headings.size(), this->uavs.size());
        }

        for (int i = 0; i < velocity_headings.size(); i++)
        {
            uint32_t agent_id = velocity_headings[i].agent_id;

            if (this->uavs.find(agent_id) != this->uavs.end())
            {
                reference::VelocityHdg controller;

                controller.velocity[0] = velocity_headings[i].velocity.x;
                controller.velocity[1] = velocity_headings[i].velocity.y;
                controller.velocity[2] = velocity_headings[i].velocity.z;
                controller.heading = velocity_headings[i].heading;

                this->desired_velocities[agent_id] = controller.velocity;
                this->uavs[agent_id].uav_system.setInput(controller);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Controller answered with a command for an agent that is not in the current UAVs. Skipping this command.");
            }
        }
    }
    else
    {
        command_client_->remove_pending_request(result);
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't call the command service: " << command_client_->get_service_name() << ". Skipping control step");
        return;
    }
}

/**
 * @brief Updates the internal state of the agents with neighbors lists received from the outside (typically received from the network simulator)
 *
 * @param A Protobuf message of type PhysicsUpdate containing the neighbors lists for all agents
 *
 * This function is not used together with the following function "GetCommands".
 */
void MiniDancers::GetNeighbors(dancers_update_proto::DancersUpdate &network_update_msg)
{
    if (!network_update_msg.payload().empty())
    {
        dancers_update_proto::OrderedNeighborsList neighbors_list_msg;
        neighbors_list_msg.ParseFromString(gzip_decompress(network_update_msg.payload()));

        // std::cout << neighbors_list_msg.DebugString() << std::endl;

        for (auto agent : this->uavs)
        {
            agent.second.neighbors.clear();
        }

        for (int i = 0; i < neighbors_list_msg.ordered_neighbors_size(); i++)
        {
            uint32_t agent_id = neighbors_list_msg.ordered_neighbors(i).agentid();

            if (this->uavs.find(agent_id) == this->uavs.end())
            {
                RCLCPP_WARN(this->get_logger(), "Received update infos for agent %d but we do not know the agent with this ID. Skipping this agent.", agent_id);
                continue;
            }

            // Select the right role
            if (neighbors_list_msg.ordered_neighbors(i).role() == dancers_update_proto::OrderedNeighbors::UNDEFINED)
            {
                this->uavs[agent_id].role = AgentRoleType::Undefined;
            }
            else if (neighbors_list_msg.ordered_neighbors(i).role() == dancers_update_proto::OrderedNeighbors::MISSION)
            {
                this->uavs[agent_id].role = AgentRoleType::Mission;
            }
            else if (neighbors_list_msg.ordered_neighbors(i).role() == dancers_update_proto::OrderedNeighbors::POTENTIAL)
            {
                this->uavs[agent_id].role = AgentRoleType::Potential;
            }
            else if (neighbors_list_msg.ordered_neighbors(i).role() == dancers_update_proto::OrderedNeighbors::IDLE)
            {
                this->uavs[agent_id].role = AgentRoleType::Idle;
            }

            dancers_update_proto::OrderedNeighbors my_neighbors = neighbors_list_msg.ordered_neighbors().at(i);

            if (my_neighbors.neighborid().empty())
            {
                if (this->uavs[agent_id].role == AgentRoleType::Undefined)
                {
                    // Skip this agent, we don't want to erase its neighbors list
                    continue;
                }
                else
                {
                    RCLCPP_FATAL(this->get_logger(), "Agent %d has no neighbors but has a role different of Undefined !", agent_id);
                    exit(EXIT_FAILURE);
                }
            }

            assert(my_neighbors.neighborid_size() == my_neighbors.linkquality_size());
            for (int j = 0; j < my_neighbors.neighborid_size(); j++)
            {
                if (agent_id == my_neighbors.neighborid(j))
                {
                    RCLCPP_FATAL(this->get_logger(), "Agent %d is its own neighbor !", agent_id);
                    exit(EXIT_FAILURE);
                }
                else
                {
                    NeighborInfo_t neighbor_info;
                    neighbor_info.id = my_neighbors.neighborid(j);
                    neighbor_info.link_quality = my_neighbors.linkquality(j);
                    switch (my_neighbors.neighbortype(j))
                    {
                    case dancers_update_proto::OrderedNeighbors::UNDEFINED:
                        neighbor_info.role = AgentRoleType::Undefined;
                        break;
                    case dancers_update_proto::OrderedNeighbors::MISSION:
                        neighbor_info.role = AgentRoleType::Mission;
                        break;
                    case dancers_update_proto::OrderedNeighbors::POTENTIAL:
                        neighbor_info.role = AgentRoleType::Potential;
                        break;
                    case dancers_update_proto::OrderedNeighbors::IDLE:
                        neighbor_info.role = AgentRoleType::Idle;
                        break;
                    default:
                        neighbor_info.role = AgentRoleType::Undefined;
                        break;
                    }

                    this->uavs[agent_id].neighbors.push_back(neighbor_info);
                }
            }
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "No neighbors received !");
    }
}

/**
 * @brief Updates the internal state of the agents with the new input commands (called at every timestep)
 *
 * @param A Protobuf message of type DancersUpdate containing the desired velocity vectors for each agent
 *
 * This function is not used together with "GetNeighbors". Mini-dancers either receives neighborhood information and computes the desired velocities,
 * or receives the desired velocities directly
 */
void MiniDancers::GetCommands(dancers_update_proto::DancersUpdate &network_update_msg)
{
    if (!network_update_msg.payload().empty())
    {
        dancers_update_proto::VelocityHeadingVector velocity_headings_msg;
        velocity_headings_msg.ParseFromString(gzip_decompress(network_update_msg.payload()));

        // std::cout << velocity_headings_msg.DebugString() << std::endl;

        std::vector<uint32_t> updated_agents;
        for (auto &velocity_heading : velocity_headings_msg.velocity_heading())
        {
            uint32_t agent_id = velocity_heading.agentid();

            if (this->uavs.find(agent_id) == this->uavs.end())
            {
                RCLCPP_WARN(this->get_logger(), "MiniDancers received a velocity & heading command for agent %d but it does not know this agent! Discarding this command.", agent_id);
                continue;
            }

            reference::VelocityHdg controller;

            controller.velocity[0] = velocity_heading.vx();
            controller.velocity[1] = velocity_heading.vy();
            controller.velocity[2] = velocity_heading.vz();
            controller.heading = velocity_heading.heading();

            this->desired_velocities[agent_id] = controller.velocity;
            this->uavs[agent_id].uav_system.setInput(controller);
            updated_agents.push_back(agent_id);
        }
        // Check if some agents have no command
        for (auto agent : this->uavs)
        {
            if (std::find(updated_agents.begin(), updated_agents.end(), agent.first) == updated_agents.end())
            {
                reference::VelocityHdg controller;
                agent.second.uav_system.setInput(controller);
                RCLCPP_WARN(this->get_logger(), "Did not receive a command for UAV %d ! Default command is Zero.", agent.first);
            }
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Received an empty DancersUpdate message !");
    }
}

/**
 * @brief Searches a target associated with an agent and returns a pointer to it, or a nullptr otherwise. It also verifies that this agent is assigned to a single objective, and crashes otherwise.  
 */
target_t* MiniDancers::GetAgentTarget(uint32_t agent_id)
{
    int nb_assigned_targets = 0;
    target_t* assigned_target = nullptr;
    for (auto target : this->target_areas)
    {
        for (uint32_t assigned_agent : target.second.assigned_agents)
        {
            if (assigned_agent == agent_id || assigned_agent == -1)
            {
                assigned_target = &target.second;
                nb_assigned_targets += 1;
            }
        }
    }
    if (nb_assigned_targets > 1)
    {
        RCLCPP_ERROR(this->get_logger(), "Agent %d is assigned to more than one target area !", agent_id);
        exit(EXIT_FAILURE);
    }
    return assigned_target;
}


/**
 * \brief Generates the final protobuf message of protobuf_msgs/PhysicsUpdate.
 *
 * It will fill the message with the (compressed) robots_positions for the current simulation window,
 * change the message-type to END, string-serialize the protobuf message, compress it, and return it.
 *
 */
std::string MiniDancers::GenerateResponseProtobuf()
{
    // Change message's type to END
    dancers_update_proto::DancersUpdate physics_update_msg;
    physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

    dancers_update_proto::PoseVector robots_positions_msg;
    for (auto agent : this->uavs)
    {
        agent_t &agent_struct = agent.second;
        dancers_update_proto::Pose *robot_pose_msg = robots_positions_msg.add_pose();
        Eigen::Vector3d agent_position = agent_struct.uav_system.getState().x;
        Eigen::Vector3d agent_velocity = agent_struct.uav_system.getState().v;
        robot_pose_msg->set_agent_id(agent_struct.id);
        robot_pose_msg->set_x(agent_position.x());
        robot_pose_msg->set_y(agent_position.y());
        robot_pose_msg->set_z(agent_position.z());
        robot_pose_msg->set_vx(agent_velocity.x());
        robot_pose_msg->set_vy(agent_velocity.y());
        robot_pose_msg->set_vz(agent_velocity.z());
    }

    std::string robots_positions_string;
    robots_positions_msg.SerializeToString(&robots_positions_string);

    // Fill the channel_data field with the compressed data from the physics simulation
    physics_update_msg.set_payload(gzip_compress(robots_positions_string));

    // Transform the response [protobuf] --> [string]
    std::string str_response;
    physics_update_msg.SerializeToString(&str_response);
    str_response = gzip_compress(str_response);

    return str_response;
}

/**
 * @brief Locally generate neighbors information and role for agents based only on their positions. It ignores the network.
 *
 * To be used when not in co-simulation mode, it defines neighborhood based on sphere-like communication range.
 * Every agent is "Idle" and only have "Idle" neighbors.
 */
void MiniDancers::GenerateRolesAndNeighbors()
{
    for (auto agent : this->uavs)
    {
        agent.second.neighbors.clear();
        agent.second.role = AgentRoleType::Idle;
    }

    for (auto agent : this->uavs)
    {
        for (auto other_agent : this->uavs)
        {
            if (agent.first != other_agent.first)
            {
                double distance = (agent.second.uav_system.getState().x - other_agent.second.uav_system.getState().x).norm();
                if (distance < this->communication_range)
                {
                    NeighborInfo_t neighbor_info;
                    neighbor_info.id = other_agent.first;
                    neighbor_info.link_quality = distance;
                    neighbor_info.role = AgentRoleType::Idle;
                    agent.second.neighbors.push_back(neighbor_info);
                }
            }
        }
    }
}

/**
 * @brief Main simulation loop
 *
 * This function should be called only once at the end of the ROS2 node's constructor. If the co-simulation mode is used, it connects to the Coordinator node via an UDS Socket and them loop until the end off the
 */
void MiniDancers::Loop()
{
    if (cosim_mode) // co-simulation mode
    {
        /* ---- Create socket (server) with Coordinator ---- */
        Socket *socket_coord;
        boost::asio::io_context io_context;
        if (this->phy_use_uds)
        {
            socket_coord = new UDSSocket(io_context);
            socket_coord->accept(this->phy_uds_server_address, 0);
        }
        else
        {
            socket_coord = new TCPSocket(io_context);
            socket_coord->accept(this->phy_ip_server_address, this->phy_ip_server_port);
        }
        RCLCPP_INFO(this->get_logger(), "\x1b[32mSocket connected with Coordinator \x1b[0m");

        // Main simulation loop
        while (rclcpp::ok() && (this->it < this->it_end_sim || this->simulation_length == 0.0))
        {
            std::string received_data = gzip_decompress(socket_coord->receive_one_message());

            // Initialize empty protobuf message type [DancersUpdate]
            dancers_update_proto::DancersUpdate network_update_msg;

            // Transform the message received from the UDS socket (string -> protobuf)
            network_update_msg.ParseFromString(received_data);

            if (this->save_compute_time)
            {
                this->probe.start();
            }

            if (this->mode == "exchange_neighbors")
            {
                this->GetNeighbors(network_update_msg);

                // TODO: keep both ways of updating the commands, internally or from a ROS2 publisher
                this->UpdateCmds();
            }
            else if (this->mode == "exchange_commands")
            {
                this->GetCommands(network_update_msg);
            }
            else
            {
                RCLCPP_FATAL(this->get_logger(), "Unknown mode: %s", this->mode.c_str());
                exit(EXIT_FAILURE);
            }

            // We step the dynamics of each agents in separated thread, actually not 100% sure this is an optimization.
            // std::vector<std::thread> workers;
            // for (int i = 0; i < this->uavs.size(); i++)
            // {
            //     workers.push_back(std::thread(&MiniDancers::MakeStep, this, i));
            // }
            // for (int i = 0; i < this->uavs.size(); i++)
            // {
            //     workers[i].join();
            // }
            // workers.clear();
            
            // reset the external force
            for (auto& uav : this->uavs)
            {        
                uav.second.uav_system.applyForce(Eigen::Vector3d::Zero());
            }

            this->ApplyObstacleCollisions();

            this->ApplyUavCollisions();

            for (auto agent : this->uavs)
            {
                this->MakeStep(agent.first);
            }

            this->DisplayRviz();

            this->it++;

            if (this->save_compute_time)
            {
                this->probe.stop();
            }

            // Generate the response message
            std::string response = this->GenerateResponseProtobuf();

            // Send the response in the UDS socket
            socket_coord->send_one_message(response);

            if (this->print_sim_advancement_)
            {
                RCLCPP_INFO(this->get_logger(), "Iteration %ld/%ld", this->it, this->it_end_sim);
            }
        }
    }
    else // mini-dancers only mode
    {
        while (rclcpp::ok() && this->it < this->it_end_sim)
        {
            if (this->save_compute_time)
            {
                this->probe.start();
            }

            this->GenerateRolesAndNeighbors();

            // Carefull, in "mini-dancers only" mode, we have no neighbor, so we need to have a command algorithm that does not use neighbor information in UpdateCmds !
            this->UpdateCmds();

            std::vector<std::thread> workers;
            for (auto agent : this->uavs)
            {
                workers.push_back(std::thread(&MiniDancers::MakeStep, this, agent.first));
            }
            for (auto agent : this->uavs)
            {
                workers[agent.first].join();
            }
            workers.clear();

            this->DisplayRviz();

            this->it++;

            if (this->save_compute_time)
            {
                this->probe.stop();
            }

            if (this->print_sim_advancement_)
            {
                RCLCPP_INFO(this->get_logger(), "Iteration %d/%d", this->it, this->it_end_sim);
            }
        }
    }

    // record total number of collisions for each agent
    std::ofstream file;
    file.open(this->m_collisions_file.c_str(), std::ios::app);
    for (const auto& agent : this->uavs)
    {
        file << agent.first << "," << agent.second.obstacles_collisions << "," << agent.second.uavs_collisions << std::endl;
    }
    file.close();

    exit(EXIT_SUCCESS);
}

/**
 * @brief Wrapper around UavSystem::makeStep() to be used in a thread
 */
void MiniDancers::MakeStep(uint32_t agent_id)
{
    std::lock_guard<std::mutex> lock(this->uavs_mutex);
    this->uavs[agent_id].uav_system.makeStep(this->step_size);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniDancers>());
    rclcpp::shutdown();
    return 0;
}
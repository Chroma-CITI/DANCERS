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

#include <protobuf_msgs/physics_update.pb.h>
#include <protobuf_msgs/network_update.pb.h>
#include <protobuf_msgs/robots_positions.pb.h>
#include <protobuf_msgs/ordered_neighbors.pb.h>

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
            this->n_uavs = config["robots_number"].as<int>();
            this->step_size = config["phy_step_size"].as<int>() / 1000000.0f; // us to s
            this->it = 0;
            this->simulation_length = config["simulation_length"].as<double>();
            this->it_end_sim = uint64_t(simulation_length / this->step_size);
            this->mission_flow_id = config["mission_flow"]["flow_id"].as<uint32_t>();
            this->potential_flow_id = config["broadcast_flow"]["flow_id"].as<uint32_t>();
            this->radius = 10.0;
            this->omega = 0.005;
            for (auto goal : config["secondary_objectives"])
            {
                this->secondary_objectives[goal.first.as<int>()].first = Eigen::Vector3d(goal.second[0].as<double>(), goal.second[1].as<double>(), goal.second[2].as<double>());
                this->secondary_objectives[goal.first.as<int>()].second = goal.second[3].as<bool>();
                std::cout << "agent " << goal.first.as<int>() << " : " << this->secondary_objectives[goal.first.as<int>()].first.transpose() << std::endl;
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
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                std::cerr << "Failed to read at least one VAT flocking parameter, using ALL default VAT params." << '\n';
            }
            

            try
            {
                this->circle_params.gain = config["circle_parameters"]["gain"].as<double>();
                this->circle_params.omega = config["circle_parameters"]["omega"].as<double>();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                std::cerr << "Failed to read at least one circle parameter, using ALL default circle params." << '\n';
            }
            

            this->phy_uds_server_address = config["phy_uds_server_address"].as<std::string>();

            rclcpp::QoS qos_persistent = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

            /* ----------- Publishers ----------- */
            this->obstacles_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", qos_persistent);
            this->pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("agent_poses", 10);
            if (this->publish_agent_structs)
            {
                this->agent_structs_pub_ = this->create_publisher<dancers_msgs::msg::AgentStructArray>("agent_structs", 10);
            }
            this->id_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("id_markers", 10);
            this->desired_velocities_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("desired_velocities", 10);
            this->network_mission_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_mission_links", 10);
            this->network_potential_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_potential_links", 10);
            this->network_idle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_idle_links", 10);
            this->target_areas_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_areas", 10);
            this->base_station_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("base_station", 10);


            /* ----------- Subscribers ----------- */
            this->targets_sub_ = this->create_subscription<dancers_msgs::msg::Target>("targets", 10, std::bind(&MiniDancers::targets_clbk, this, _1));

            /* ----------- Service client ----------- */
            command_client_ = this->create_client<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities");

            this->InitObstacles();
            
            this->InitUavs();

            RCLCPP_INFO(this->get_logger(), "MiniDancers simulator initialized");

            this->Loop();
        }
    private:
        std::vector<obstacle_t> obstacles;
        int n_uavs;
        std::vector<agent_t> uavs;                          
        std::vector<Eigen::Vector3d> desired_velocities;
        double step_size;
        double simulation_length;
        uint64_t it;
        uint64_t it_end_sim;
        std::string phy_uds_server_address;
        std::map<int, std::pair<Eigen::Vector3d, bool>> secondary_objectives;
        std::string ros_ws_path;
        VAT_params_t vat_params;
        circle_params_t circle_params;
        uint32_t mission_flow_id;
        uint32_t potential_flow_id;
        uint32_t routing_flow_id;
        bool cosim_mode;

        double radius;
        double omega;
        double target_altitude;
        
        /* Methods */
        void InitObstacles();
        void InitUavs();
        void DisplayRviz();
        void UpdateCmds();
        void GetNeighbors(network_update_proto::NetworkUpdate &network_update_msg);
        std::string GenerateResponseProtobuf();
        void Loop();

        void MakeStep(int i);

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
        void targets_clbk(dancers_msgs::msg::Target msg);

        /* Service client*/
        rclcpp::Client<dancers_msgs::srv::GetAgentVelocities>::SharedPtr command_client_;

        /* Compute time saving */
        bool save_compute_time;
        std::string m_computation_time_file;
        WallTimeProbe probe;

        /* Publish agent Structs */
        bool publish_agent_structs;
};

void MiniDancers::targets_clbk(dancers_msgs::msg::Target msg)
{
    std::pair<Eigen::Vector3d, bool> p;
    p.first = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    p.second = msg.target_type;

    std::cout << "Agent " << msg.id << " has a new target : " << p.first.transpose() << std::endl;

    this->secondary_objectives[msg.id] = p;
    for (auto agent = this->uavs.begin(); agent != this->uavs.end(); ++agent)
    {
        if (agent->id == msg.id)
        {
            agent->secondary_objective = p.first;
            std::cout << "Agent " << agent->id << " has a secondary objective : " << p.first.transpose() << std::endl;
        }
    }
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
        obstacle_marker.scale.x = 1.0;  // Set the desired width of the lines
        obstacle_marker.scale.y = 1.0;
        obstacle_marker.scale.z = 1.0;
        obstacle_marker.pose.position.x = 0.0;
        obstacle_marker.pose.position.y = 0.0;
        obstacle_marker.pose.position.z = 0.0;
        // Set the color (RGBA)
        obstacle_marker.color.r = 0.0;
        obstacle_marker.color.g = 1.0;  // Green color
        obstacle_marker.color.b = 0.0;
        obstacle_marker.color.a = 0.8;  // Fully opaque

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
    double grid_x_init = 0.0;
    double grid_y_init = -((n_columns-1) * spacing)/2;
    for (int i = 0; i < this->n_uavs; i++)
    {
        // Grid spawn
        double spawn_x = grid_x_init + i / n_columns * spacing;
        double spawn_y = grid_y_init + spacing * (i% n_columns);
        double spawn_z = 1.0;
        double spawn_heading = 0.0;

        // Already in line robots
        // double spawn_x = (rand() % 200 - 100) / 100.0;
        // double spawn_y = i * 50.0;
        // double spawn_z = 1.0;
        // double spawn_heading = 0.0;

        // Default ModelParams is the x500 configuration
        MultirotorModel::ModelParams x500_params = MultirotorModel::ModelParams();
        UavSystem uav_system = UavSystem(x500_params, Eigen::Vector3d(spawn_x, spawn_y, spawn_z), spawn_heading);

        agent_t agent;
        agent.id = i;
        agent.uav_system = uav_system;
        agent.neighbors = std::vector<NeighborInfo_t>();
        if (this->secondary_objectives.find(i) != this->secondary_objectives.end())
        {
            agent.secondary_objective = this->secondary_objectives[i].first;
        }
        this->desired_velocities.push_back(Eigen::Vector3d::Zero());

        this->uavs.push_back(agent);
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
    for (int i = 0; i < n_uavs ; i++)
    {
        // Display poses
        MultirotorModel::State uav_state = uavs[i].uav_system.getState();
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
        id_marker.id = i;
        id_marker.header.frame_id = "map";
        id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        id_marker.action = visualization_msgs::msg::Marker::ADD;
        id_marker.color.r = 0.0;
        id_marker.color.g = 0.0;
        id_marker.color.b = 1.0;
        id_marker.color.a = 1.0;  // Fully opaque
        id_marker.scale.z = 2.0;
        id_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        id_marker.text = std::to_string(i);
        id_marker.pose.position.x = uav_state.x.x()-1;
        id_marker.pose.position.y = uav_state.x.y();
        id_marker.pose.position.z = uav_state.x.z();
        id_marker_array.markers.push_back(id_marker);

        // Display desired velocities
        visualization_msgs::msg::Marker desired_velocity_marker{};
        desired_velocity_marker.header.frame_id = "map";
        desired_velocity_marker.id = 2000+i;
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
        p2.x = uav_state.x.x() + this->desired_velocities[i].x();
        p2.y = uav_state.x.y() + this->desired_velocities[i].y();
        p2.z = uav_state.x.z() + this->desired_velocities[i].z();
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
    sources_marker.color.r = 1.0;
    sources_marker.color.g = 0.0;
    sources_marker.color.b = 1.0;
    sources_marker.color.a = 0.5;
    sources_marker.lifetime = rclcpp::Duration::from_seconds(0);
    for (auto secondary_objective : this->secondary_objectives)
    {
        if (secondary_objective.second.second == false)
        {
            geometry_msgs::msg::Point p{};
            p.x = secondary_objective.second.first.x();
            p.y = secondary_objective.second.first.y();
            p.z = secondary_objective.second.first.z();
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
    for (auto secondary_objective : this->secondary_objectives)
    {
        if (secondary_objective.second.second == true)
        {
            sink_marker.pose.position.x = secondary_objective.second.first.x();
            sink_marker.pose.position.y = secondary_objective.second.first.y();
            sink_marker.pose.position.z = secondary_objective.second.first.z();

            this->base_station_pub_->publish(sink_marker);
        }
    }



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

    for (int i=0; i < this->n_uavs; i++)
    {
        Eigen::Vector3d agent_pose(this->uavs[i].uav_system.getState().x);
        for (size_t j=0; j < this->uavs[i].neighbors.size(); j++)
        {
            int neighbor_id = this->uavs[i].neighbors[j].id;
            if (i == neighbor_id)
                continue; // (should never happen as we check this in GetNeighbors) 
            Eigen::Vector3d other_agent_pose(this->uavs[neighbor_id].uav_system.getState().x);

            geometry_msgs::msg::Point p1{};
            p1.x = agent_pose.x();
            p1.y = agent_pose.y();
            p1.z = agent_pose.z();
            geometry_msgs::msg::Point p2{};
            p2.x = other_agent_pose.x();
            p2.y = other_agent_pose.y();
            p2.z = other_agent_pose.z();

            switch (this->uavs[i].role)
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

    // // routing neighbors
    // visualization_msgs::msg::Marker network_marker_routing{};
    // network_marker_routing.header.frame_id = "map";
    // network_marker_routing.id = 1003;
    // network_marker_routing.type = visualization_msgs::msg::Marker::LINE_LIST;
    // network_marker_routing.action = visualization_msgs::msg::Marker::ADD;
    // network_marker_routing.scale.x = 0.3;
    // network_marker_routing.color.r = 0.0;  
    // network_marker_routing.color.g = 1.0;   // green color
    // network_marker_routing.color.b = 0.0;
    // network_marker_routing.color.a = 1.0;   // Fully opaque
    // network_marker_routing.lifetime = rclcpp::Duration::from_seconds(0.1);
    // for (int i=0; i < this->n_uavs; i++)
    // {
    //     Eigen::Vector3d agent_pose(this->uavs[i].uav_system.getState().x);
    //     for (size_t j=0; j < this->uavs[i].neighbors_routing.size(); j++)
    //     {
    //         int neighbor_id = this->uavs[i].neighbors_routing[j];
    //         if (i == neighbor_id)
    //             continue; // (should never happen as we check this in GetNeighbors) 
    //         Eigen::Vector3d other_agent_pose(this->uavs[neighbor_id].uav_system.getState().x);

    //         geometry_msgs::msg::Point p1{};
    //         p1.x = agent_pose.x();
    //         p1.y = agent_pose.y();
    //         p1.z = agent_pose.z();
    //         network_marker_routing.points.push_back(p1);
    //         geometry_msgs::msg::Point p2{};
    //         p2.x = other_agent_pose.x();
    //         p2.y = other_agent_pose.y();
    //         p2.z = other_agent_pose.z();
    //         network_marker_routing.points.push_back(p2);
    //     }
    // }
    // this->network_routing_pub_->publish(network_marker_routing);

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

    for (int i=0; i < this->n_uavs; i++)
    {
        dancers_msgs::msg::AgentStruct agent_struct;
        agent_t& self_agent = this->uavs[i];

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


        for (NeighborInfo_t& neighbor: self_agent.neighbors)
        {
            dancers_msgs::msg::Neighbor neighbor_msg;
            neighbor_msg.agent_id = neighbor.id;
            neighbor_msg.link_quality = neighbor.link_quality;
            
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
        std::vector<dancers_msgs::msg::VelocityHeading>& velocity_headings = request_msg->velocity_headings.velocity_heading_array;
        
        assert(velocity_headings.size() == this->n_uavs);

        for (int i=0; i < this->n_uavs; i++)
        {
            reference::VelocityHdg controller;

            controller.velocity[0] = velocity_headings[i].velocity.x;
            controller.velocity[1] = velocity_headings[i].velocity.y;
            controller.velocity[2] = velocity_headings[i].velocity.z;
            controller.heading = velocity_headings[i].heading;

            this->desired_velocities[i] = controller.velocity;
            this->uavs[i].uav_system.setInput(controller);
        }
    }
    else
    {
        command_client_->remove_pending_request(result);
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't call the command service: " << command_client_->get_service_name()<< ". Skipping control step");
        return;
    }
}

/**
 * @brief Updates the internal state of the agents with neighbors lists received from the outside (typically received from the network simulator)
 * 
 * @param A Protobuf message of type PhysicsUpdate containing the neighbors lists for all agents
 */
void MiniDancers::GetNeighbors(network_update_proto::NetworkUpdate &network_update_msg)
{
    if(!network_update_msg.ordered_neighbors().empty())
    {
        ordered_neighbors_proto::OrderedNeighborsList neighbors_list_msg;
        neighbors_list_msg.ParseFromString(gzip_decompress(network_update_msg.ordered_neighbors()));

        // std::cout << neighbors_list_msg.DebugString() << std::endl;

        for (int i=0; i < this->n_uavs; i++)
        {
            this->uavs[i].neighbors.clear();
        }
        
        for (int i=0; i < neighbors_list_msg.ordered_neighbors_size(); i++)
        {
            int agent_id = neighbors_list_msg.ordered_neighbors(i).agentid();

            // Select the right role
            if (neighbors_list_msg.ordered_neighbors(i).role() == ordered_neighbors_proto::OrderedNeighbors::UNDEFINED)
            {
                this->uavs[i].role = AgentRoleType::Undefined;
            }
            else if (neighbors_list_msg.ordered_neighbors(i).role() == ordered_neighbors_proto::OrderedNeighbors::MISSION)
            {
                this->uavs[i].role = AgentRoleType::Mission;
            }
            else if (neighbors_list_msg.ordered_neighbors(i).role() == ordered_neighbors_proto::OrderedNeighbors::POTENTIAL)
            {
                this->uavs[i].role = AgentRoleType::Potential;
            }
            else if (neighbors_list_msg.ordered_neighbors(i).role() == ordered_neighbors_proto::OrderedNeighbors::IDLE)
            {
                this->uavs[i].role = AgentRoleType::Idle;
            }

            ordered_neighbors_proto::OrderedNeighbors my_neighbors = neighbors_list_msg.ordered_neighbors().at(i);

            if (my_neighbors.neighborid().empty())
            {
                if (this->uavs[i].role == AgentRoleType::Undefined)
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
            for (int j=0; j < my_neighbors.neighborid_size(); j++)
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
                        case ordered_neighbors_proto::OrderedNeighbors::UNDEFINED:
                            neighbor_info.role = AgentRoleType::Undefined;
                            break;
                        case ordered_neighbors_proto::OrderedNeighbors::MISSION:
                            neighbor_info.role = AgentRoleType::Mission;
                            break;
                        case ordered_neighbors_proto::OrderedNeighbors::POTENTIAL:
                            neighbor_info.role = AgentRoleType::Potential;
                            break;
                        case ordered_neighbors_proto::OrderedNeighbors::IDLE:
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
 * \brief Generates the final protobuf message of protobuf_msgs/PhysicsUpdate.
 *
 * It will fill the message with the (compressed) robots_positions for the current simulation window,
 * change the message-type to END, string-serialize the protobuf message, compress it, and return it.
 *
 */
std::string MiniDancers::GenerateResponseProtobuf()
{
    // Change message's type to END
    physics_update_proto::PhysicsUpdate physics_update_msg;
    physics_update_msg.set_msg_type(physics_update_proto::PhysicsUpdate::END);

    robots_positions_proto::RobotsPositions robots_positions_msg;
    for (int i=0; i < this->n_uavs; i++)
    {
        robots_positions_proto::RobotPose *robot_pose_msg = robots_positions_msg.add_robot_pose();
        Eigen::Vector3d agent_position = this->uavs[i].uav_system.getState().x;
        robot_pose_msg->set_x(agent_position.x());
        robot_pose_msg->set_y(agent_position.y());
        robot_pose_msg->set_z(agent_position.z());
    }

    std::string robots_positions_string;
    robots_positions_msg.SerializeToString(&robots_positions_string);

    // Fill the channel_data field with the compressed data from the physics simulation
    physics_update_msg.set_robots_positions(gzip_compress(robots_positions_string));

    // Transform the response [protobuf] --> [string]
    std::string str_response;
    physics_update_msg.SerializeToString(&str_response);
    str_response = gzip_compress(str_response);

    return str_response;
}


/**
 * @brief Main simulation loop
 * 
 * This function should be called only once at the end of the ROS2 node's constructor. If the co-simulation mode is used, it connects to the Coordinator node via an UDS Socket and them loop until the end off the   
 */
void MiniDancers::Loop()
{
    if (cosim_mode)         // co-simulation mode
    {
        /* ---- Create socket (server) with Coordinator ---- */
        Socket *socket_coord;
        boost::asio::io_context io_context;
        socket_coord = new UDSSocket(io_context);
        socket_coord->accept(this->phy_uds_server_address, 0);
        RCLCPP_INFO(this->get_logger(), "\x1b[32mSocket connected with Coordinator \x1b[0m");

        // Main simulation loop
        while (rclcpp::ok() && (this->it < this->it_end_sim || this->simulation_length == 0.0))
        {
            std::string received_data = gzip_decompress(socket_coord->receive_one_message());

            // Initialize empty protobuf message type [NetworkUpdate]
            network_update_proto::NetworkUpdate network_update_msg;
            // Transform the message received from the UDS socket (string -> protobuf)
            network_update_msg.ParseFromString(received_data);

            if (this->save_compute_time)
            {
                this->probe.start();
            }

            this->GetNeighbors(network_update_msg);

            // TODO: keep both ways of updating the commands, internally or from a ROS2 publisher 
            this->UpdateCmds();

            // We step the dynamics of each agents in separated thread, actually not 100% sure this is an optimization.
            std::vector<std::thread> workers;
            for (int i=0; i < this->n_uavs; i++)
            {
                workers.push_back(std::thread(&MiniDancers::MakeStep, this, i));
            }
            for (int i=0; i < this->n_uavs; i++)
            {
                workers[i].join();
            }
            workers.clear();

            // for (int i=0; i < this->n_uavs; i++)
            // {
            //     this->MakeStep(i);
            // }

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

        }
    }
    else            // mini-dancers only mode
    {
        while(rclcpp::ok() && this->it < this->it_end_sim)
        {
            if (this->save_compute_time)
            {
                this->probe.start();
            }

            // Carefull, in "mini-dancers only" mode, we have no neighbor, so we need to have a command algorithm that does not use neighbor information in UpdateCmds !
            // this->UpdateCmds();

            std::vector<std::thread> workers;
            for (int i=0; i < this->n_uavs; i++)
            {
                workers.push_back(std::thread(&MiniDancers::MakeStep, this, i));
            }
            for (int i=0; i < this->n_uavs; i++)
            {
                workers[i].join();
            }
            workers.clear();


            this->DisplayRviz();
            
            this->it++;

            if (this->save_compute_time)
            {
                this->probe.stop();
            }
        }
    }
}

/**
 * @brief Wrapper around UavSystem::makeStep() to be used in a thread
 */
void MiniDancers::MakeStep(int i)
{
    this->uavs[i].uav_system.makeStep(this->step_size);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniDancers>());
    rclcpp::shutdown();
    return 0;
}
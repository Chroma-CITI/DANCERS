#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <uav_system.hpp>
#include <VAT_flocking_controller.hpp>
#include <udp_tcp_socket.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <protobuf_msgs/physics_update.pb.h>
#include <protobuf_msgs/robots_positions.pb.h>
#include <protobuf_msgs/ordered_neighbors.pb.h>

#include "boost/filesystem.hpp"

#include <Eigen/Core>

#include <util.hpp>
#include <time_probe.hpp>

#include <yaml-cpp/yaml.h>

using namespace mrs_multirotor_simulator;

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
            this->n_uavs = config["robots_number"].as<int>();
            this->step_size = config["phy_step_size"].as<int>() / 1000000.0f; // us to s
            this->it = 0;
            this->radius = 10.0;
            this->omega = 0.005;
            this->target_altitude = config["target_altitude"].as<double>();
            for (auto goal : config["secondary_objectives"])
            {
                this->secondary_objectives[goal.first.as<int>()] = Eigen::Vector3d(goal.second[0].as<double>(), goal.second[1].as<double>(), goal.second[2].as<double>());
                std::cout << "agent " << goal.first.as<int>() << " : " << this->secondary_objectives[goal.first.as<int>()].transpose() << std::endl;
            }
            this->vat_params.v_flock = config["VAT_flocking_parameters"]["v_flock"].as<double>();
            this->vat_params.v_max = config["VAT_flocking_parameters"]["v_max"].as<double>();
            this->vat_params.a_frict = config["VAT_flocking_parameters"]["a_frict"].as<double>();
            this->vat_params.p_frict = config["VAT_flocking_parameters"]["p_frict"].as<double>();
            this->vat_params.r_0_frict = config["VAT_flocking_parameters"]["r_0_frict"].as<double>();
            this->vat_params.C_frict = config["VAT_flocking_parameters"]["C_frict"].as<double>();
            this->vat_params.v_frict = config["VAT_flocking_parameters"]["v_frict"].as<double>();
            this->vat_params.p_att = config["VAT_flocking_parameters"]["p_att"].as<double>();
            this->vat_params.r_0_att = config["VAT_flocking_parameters"]["r_0_att"].as<double>();
            this->vat_params.p_rep = config["VAT_flocking_parameters"]["p_rep"].as<double>();
            this->vat_params.r_0_rep = config["VAT_flocking_parameters"]["r_0_rep"].as<double>();
            this->vat_params.a_shill = config["VAT_flocking_parameters"]["a_shill"].as<double>();
            this->vat_params.p_shill = config["VAT_flocking_parameters"]["p_shill"].as<double>();
            this->vat_params.r_0_shill = config["VAT_flocking_parameters"]["r_0_shill"].as<double>();
            this->vat_params.v_shill = config["VAT_flocking_parameters"]["v_shill"].as<double>();

            this->phy_uds_server_address = config["phy_uds_server_address"].as<std::string>();

            rclcpp::QoS qos_persistent = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

            /* ----------- Publishers ----------- */
            this->obstacles_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", qos_persistent);
            this->pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("agent_poses", 10);
            this->id_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("id_markers", 10);
            this->desired_velocities_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("desired_velocities", 10);
            this->network_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("network_links", 10);
            this->secondary_objectives_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("secondary_objectives", 10);

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
        uint64_t it;
        std::string phy_uds_server_address;
        std::map<int, Eigen::Vector3d> secondary_objectives;
        std::string ros_ws_path;
        VAT_params_t vat_params;

        double radius;
        double omega;
        double target_altitude;
        
        /* Methods */
        void InitObstacles();
        void InitUavs();
        void DisplayRviz();
        void UpdateCmds();
        void GetNeighbors(physics_update_proto::PhysicsUpdate &PhysicsUpdate_msg);
        std::string GenerateResponseProtobuf(bool targets_reached);
        void Loop();

        /* Publishers */
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacles_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr id_markers_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr desired_velocities_markers_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr network_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr secondary_objectives_pub_;

        /* Compute time saving */
        bool save_compute_time;
        std::string m_computation_time_file;
        WallTimeProbe probe;

};

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
        obstacle_marker.color.a = 0.7;  // Fully opaque

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

void MiniDancers::InitUavs()
{
    int n_columns = (int)sqrt(this->n_uavs);
    double spacing = 3.0;
    for (int i = 0; i < this->n_uavs; i++)
    {
        double spawn_x = i / n_columns * spacing;
        double spawn_y = spacing * (i% n_columns);
        double spawn_z = 1.0;
        double spawn_heading = 0.0;

        // Default ModelParams is the x500 configuration
        MultirotorModel::ModelParams x500_params = MultirotorModel::ModelParams();
        UavSystem uav_system = UavSystem(x500_params, Eigen::Vector3d(spawn_x, spawn_y, spawn_z), spawn_heading);

        // TODO: check if usefull
        uav_system.makeStep(0.01);
        uav_system.makeStep(0.01);

        agent_t agent;
        agent.id = i;
        agent.uav_system = uav_system;
        agent.neighbors = std::vector<int>();
        agent.link_qualities = std::vector<double>();
        if (this->secondary_objectives.find(i) != this->secondary_objectives.end())
        {
            agent.secondary_objective = this->secondary_objectives[i];
        }
        this->desired_velocities.push_back(Eigen::Vector3d::Zero());

        this->uavs.push_back(agent);
    }
}

void MiniDancers::DisplayRviz()
{
    // Display robots
    geometry_msgs::msg::PoseArray pose_array{};
    pose_array.header.frame_id = "map";
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
        id_marker.pose.position.x = uav_state.x.x();
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

    // Display secondary objectives
    visualization_msgs::msg::Marker secondary_objectives_marker{};
    secondary_objectives_marker.header.frame_id = "map";
    secondary_objectives_marker.id = 1000;
    secondary_objectives_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    secondary_objectives_marker.action = visualization_msgs::msg::Marker::ADD;
    secondary_objectives_marker.scale.x = 5.0;
    secondary_objectives_marker.scale.y = 5.0;
    secondary_objectives_marker.scale.z = 5.0;
    secondary_objectives_marker.pose.position.x = 0.0;
    secondary_objectives_marker.pose.position.y = 0.0;
    secondary_objectives_marker.pose.position.z = 0.0;
    secondary_objectives_marker.color.r = 1.0;
    secondary_objectives_marker.color.g = 0.0;
    secondary_objectives_marker.color.b = 1.0;
    secondary_objectives_marker.color.a = 0.5;
    secondary_objectives_marker.lifetime = rclcpp::Duration::from_seconds(0);
    for (auto secondary_objective : this->secondary_objectives)
    {
        geometry_msgs::msg::Point p{};
        p.x = secondary_objective.second.x();
        p.y = secondary_objective.second.y();
        p.z = secondary_objective.second.z();
        secondary_objectives_marker.points.push_back(p);
    }
    this->secondary_objectives_pub_->publish(secondary_objectives_marker);

    // Display edges between neighbors
    visualization_msgs::msg::Marker network_marker{};
    network_marker.header.frame_id = "map";
    network_marker.id = 1001;
    network_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    network_marker.action = visualization_msgs::msg::Marker::ADD;
    network_marker.scale.x = 0.2;
    network_marker.color.r = 1.0;  // Red color
    network_marker.color.g = 0.0;
    network_marker.color.b = 0.0;
    network_marker.color.a = 1.0;  // Fully opaque
    network_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    for (int i=0; i < this->n_uavs; i++)
    {
        Eigen::Vector3d agent_pose(this->uavs[i].uav_system.getState().x);
        for (size_t j=0; j < this->uavs[i].neighbors.size(); j++)
        {
            int neighbor_id = this->uavs[i].neighbors[j];
            if (i == neighbor_id)
                continue; // (should never happen as we check this in GetNeighbors) 
            Eigen::Vector3d other_agent_pose(this->uavs[neighbor_id].uav_system.getState().x);

            geometry_msgs::msg::Point p1{};
            p1.x = agent_pose.x();
            p1.y = agent_pose.y();
            p1.z = agent_pose.z();
            network_marker.points.push_back(p1);
            geometry_msgs::msg::Point p2{};
            p2.x = other_agent_pose.x();
            p2.y = other_agent_pose.y();
            p2.z = other_agent_pose.z();
            network_marker.points.push_back(p2);
        }
    }
    this->network_pub_->publish(network_marker);    

}

/**
 * This is where the controller code goes !
 */
void MiniDancers::UpdateCmds()
{
    std::vector<reference::VelocityHdg> controllers;
    // Compute flocking commands using the VAT algorithm
    controllers = ComputeVATFlockingDesiredVelocities(this->uavs, this->obstacles, this->vat_params);

    for (int i=0; i < this->n_uavs; i++)
    {

        // Dumb proportional altitude controller (z up)
        Eigen::Vector3d agent_position = this->uavs[i].uav_system.getState().x;
        if (agent_position[2] > this->target_altitude + 0.5)
        {
            controllers[i].velocity[2] = 0.1 * (this->target_altitude - agent_position[2]);
        }
        else if (agent_position[2] < this->target_altitude - 0.5)
        {
            controllers[i].velocity[2] = 0.1 * (this->target_altitude - agent_position[2]);
        }
        else
        {
            controllers[i].velocity[2] = 0.0;
        }

        this->desired_velocities[i] = controllers[i].velocity;
        this->uavs[i].uav_system.setInput(controllers[i]);
    }
}


/**
 * @brief Updates the internal state of the agents with neighbors lists received from the outside (typically received from the network simulator)
 * 
 * @param A Protobuf message of type PhysicsUpdate containing the neighbors lists for all agents
 */
void MiniDancers::GetNeighbors(physics_update_proto::PhysicsUpdate &PhysicsUpdate_msg)
{
    if(!PhysicsUpdate_msg.ordered_neighbors().empty())
    {
        ordered_neighbors_proto::OrderedNeighborsList neighbors_list_msg;
        neighbors_list_msg.ParseFromString(gzip_decompress(PhysicsUpdate_msg.ordered_neighbors()));

        for (int i=0; i < this->n_uavs; i++)
        {
            this->uavs[i].neighbors.clear();
            this->uavs[i].link_qualities.clear();
        }
        
        for (int i=0; i < neighbors_list_msg.ordered_neighbors_size(); i++)
        {
            int agent_id = neighbors_list_msg.ordered_neighbors(i).agentid();

            std::cout << neighbors_list_msg.DebugString() << std::endl;

            ordered_neighbors_proto::OrderedNeighbors my_neighbors = neighbors_list_msg.ordered_neighbors().at(i);
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
                    this->uavs[agent_id].neighbors.push_back(my_neighbors.neighborid(j));
                    this->uavs[agent_id].link_qualities.push_back(my_neighbors.linkquality(j));
                }
            }
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No neighbors received !");
    }
}

/**
 * \brief Generates the final protobuf message of protobuf_msgs/PhysicsUpdate.
 *
 * Usually, this function will be passed an empty [PhysicsUpdate] protobuf message with message-type BEGIN.
 * It will fill the message with the (compressed) robots_positions for the current simulation window,
 * change the message-type to END, string-serialize the protobuf message and return it.
 *
 * \param robots_positions A std::string which is a string-serialized ChannelData protobuf message.
 * \param PhysicsUpdate_msg A protobuf message of type PhysicsUpdate. Usually it is an empty message with type BEGIN, this function will fill the message and
 */
std::string MiniDancers::GenerateResponseProtobuf(bool targets_reached)
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
    physics_update_msg.set_targets_reached(targets_reached);

    // Transform the response [protobuf] --> [string]
    std::string str_response;
    physics_update_msg.SerializeToString(&str_response);
    str_response = gzip_compress(str_response);

    return str_response;
}

void MiniDancers::Loop()
{
    /* ---- Create socket (server) with Coordinator ---- */
    Socket *socket_coord;
    boost::asio::io_context io_context;
    socket_coord = new UDSSocket(io_context);
    socket_coord->accept(this->phy_uds_server_address, 0);
    RCLCPP_INFO(this->get_logger(), "\x1b[32mSocket connected with Coordinator \x1b[0m");

    // Main simulation loop
    while (rclcpp::ok())
    {
        std::string received_data = gzip_decompress(socket_coord->receive_one_message());

        // Initialize empty protobuf message type [PhysicsUpdate]
        physics_update_proto::PhysicsUpdate PhysicsUpdate_msg;
        // Transform the message received from the UDS socket (string -> protobuf)
        PhysicsUpdate_msg.ParseFromString(received_data);

        if (this->save_compute_time)
        {
            this->probe.start();
        }

        this->GetNeighbors(PhysicsUpdate_msg);

        this->UpdateCmds();
        for (int i=0; i < this->n_uavs; i++)
        {
            this->uavs[i].uav_system.makeStep(this->step_size);
        }

        this->DisplayRviz();
        
        this->it++;

        bool target_reached = false;

        if (this->save_compute_time)
        {
            this->probe.stop();
        }

        // Generate the response message
        std::string response = this->GenerateResponseProtobuf(target_reached);

        // Send the response in the UDS socket
        socket_coord->send_one_message(response);

    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniDancers>());
    rclcpp::shutdown();
    return 0;
}
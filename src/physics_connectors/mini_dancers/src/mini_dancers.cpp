#include <connector_core/connector.hpp>

// UAV System from CTU Prague
#include <uav_system.hpp>

// Common DANCERS libs
#include <agent.hpp>
#include <yaml_util.hpp>

// C++ libs
#include <shared_mutex>

// Costum ROS2 messages
#include <dancers_msgs/msg/agent_struct.hpp>

// Rviz ROS2 messages
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

// Protobuf messages
#include <protobuf_msgs/pose_vector.pb.h>


using namespace mrs_multirotor_simulator;

class MiniDancers : public Connector
{
public:
    MiniDancers()
    : Connector("mini_dancers")
    {
        RCLCPP_INFO(this->get_logger(), "MiniDancers (PHY) started");

        this->ReadConfigFile();

        this->InitAgents();

        // Initialize publishers
        rclcpp::QoS reliable_qos(10); // depth of 10
        reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        reliable_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        reliable_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        this->pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("agent_poses", 10);
        this->id_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("id_markers", 10);
        this->desired_velocities_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("desired_velocities", 10);

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&MiniDancers::Loop, this);
    }

private:
    /**
     * @brief Read config file and set variables
     * 
     * It is mandatory to set the socket variables to enable the connector
     * to communicate with the simulator !
     */
    void ReadConfigFile() override
    {
        this->use_uds = getYamlValue<bool>(this->config_, "phy_use_uds");
        this->uds_server_address = getYamlValue<std::string>(this->config_, "phy_uds_server_address");
        this->ip_server_address = getYamlValue<std::string>(this->config_, "phy_ip_server_address");
        this->ip_server_port = getYamlValue<unsigned int>(this->config_, "phy_ip_server_port");

        // Read the synchronization interval and the step length as ints to easily verify that they are compatible
        unsigned int sync_window_int = config_["sync_window"].as<unsigned int>();
        unsigned int step_size_int = config_["phy_step_size"].as<unsigned int>();
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
            exit(EXIT_FAILURE);
        }
        this->step_size = step_size_int / 1000000.0f;
        this->it_end_sim = uint64_t(this->simulation_length / this->step_size);


    }

    /**
     * @brief Function called at each step, the connector is responsible to read the update message, step the simulator, 
     * and return the repsonse update message
     */
    dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate update_msg) override
    {
        if (!update_msg.payload().empty())
        {
            dancers_update_proto::PoseVector waypoint_messages;
            waypoint_messages.ParseFromString(gzip_decompress(update_msg.payload()));
            for (const auto& waypoint : waypoint_messages.pose())
            {
                std::unique_lock lock(this->agents_mutex_);
                auto agent = this->agents_.find(waypoint.agent_id());
                if (agent != this->agents_.end())
                {
                    reference::Position controller;
                    controller.position[0] = waypoint.x();
                    controller.position[1] = waypoint.y();
                    controller.position[2] = waypoint.z();
                    agent->second->uav_system->setInput(controller);
                    RCLCPP_DEBUG(this->get_logger(), "Received waypoint for agent %d: (%f, %f, %f)", waypoint.agent_id(), waypoint.x(), waypoint.y(), waypoint.z());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Received waypoint for unknown agent %d", waypoint.agent_id());
                }
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Received empty update message");
        }
        
        // Multithread the step computations. Each thread is dedicated to a single agent, so we don't need to lock anything
        std::vector<std::thread> workers;
        for (auto& [id, agent] : this->agents_)
        {
            // // Lock guard ?
            workers.push_back(std::thread([&agent, this]() {
                agent->uav_system->makeStep(this->step_size);
            }));
        }
        for (auto& worker : workers)
        {
            worker.join();
        }
        workers.clear();

        this->DisplayRviz();

        RCLCPP_DEBUG(this->get_logger(), "Finished iteration %ld", this->it);
        
        return this->GenerateResponseProtobuf();
    }

    /**
     * @brief Generate the response protobuf message to send to the coordinator
     */
    dancers_update_proto::DancersUpdate GenerateResponseProtobuf()
    {
        dancers_update_proto::DancersUpdate response_msg;
        response_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

        std::shared_lock lock(this->agents_mutex_);
        dancers_update_proto::PoseVector robots_positions_msg;
        for (auto& [agent_id, agent_ptr] : this->agents_)
        {
            dancers_update_proto::Pose *robot_pose_msg = robots_positions_msg.add_pose();
            Eigen::Vector3d agent_position = agent_ptr->uav_system->getState().x;
            Eigen::Vector3d agent_velocity = agent_ptr->uav_system->getState().v;
            robot_pose_msg->set_agent_id(agent_ptr->id);
            robot_pose_msg->set_x(agent_position.x());
            robot_pose_msg->set_y(agent_position.y());
            robot_pose_msg->set_z(agent_position.z());
            robot_pose_msg->set_vx(agent_velocity.x());
            robot_pose_msg->set_vy(agent_velocity.y());
            robot_pose_msg->set_vz(agent_velocity.z());
        }

        std::string serialized_msg;
        robots_positions_msg.SerializeToString(&serialized_msg);

        response_msg.set_payload(gzip_compress(serialized_msg));

        return response_msg;
    }

    void InitAgents();
    void DisplayRviz();

    // ROS Callbacks
    void SpawnUavClbk(dancers_msgs::msg::AgentStruct msg);

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr id_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr desired_velocities_markers_pub_;

    std::map<uint32_t, std::unique_ptr<Agent>> agents_;
    mutable std::shared_mutex agents_mutex_;
};

void MiniDancers::InitAgents()
{
    int n_columns = (int)sqrt(getYamlValue<int>(this->config_, "robots_number"));
    double spacing = getYamlValue<double>(this->config_, "initial_spacing");
    Eigen::Vector3d grid_center = Eigen::Vector3d(getYamlValue<double>(this->config_, "initial_x"), 
                                                  getYamlValue<double>(this->config_, "initial_y"), 
                                                  getYamlValue<double>(this->config_, "initial_z"));
    double initial_heading = getYamlValue<double>(this->config_, "initial_heading");

    for (unsigned int i=0; i < getYamlValue<unsigned int>(this->config_, "robots_number"); i++)
    {
        dancers_msgs::msg::AgentStruct agent_ros_msg;
        
        // Agent's ID
        agent_ros_msg.agent_id = i;

        // Find the starting position of this robot
        int row = i / n_columns;
        int col = i % n_columns;
        agent_ros_msg.state.position.x = grid_center.x() + (col - n_columns/2) * spacing;
        agent_ros_msg.state.position.y = grid_center.y() + (row - n_columns/2) * spacing;
        agent_ros_msg.state.position.z = grid_center.z();
        agent_ros_msg.state.heading = initial_heading;
        
        // Agent's initial velocity is zero
        agent_ros_msg.state.velocity.x = 0.0;
        agent_ros_msg.state.velocity.y = 0.0;
        agent_ros_msg.state.velocity.z = 0.0;


        this->SpawnUavClbk(agent_ros_msg);
    }
}

void MiniDancers::SpawnUavClbk(dancers_msgs::msg::AgentStruct msg)
{
    // Abort if the agent already exists.
    // Otherwise, add it to agents_
    {
        std::shared_lock lock(this->agents_mutex_);
        if (this->agents_.find(msg.agent_id) != this->agents_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Agent %d already exists, skipping spawn.", msg.agent_id);
            return;
        }
    }

        // Finally, create the Agent object and add it to the map
    {
        std::unique_lock lock(this->agents_mutex_);
        auto [it, success] = this->agents_.emplace(msg.agent_id, std::make_unique<Agent>(AgentFromRosMsg(msg)));
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert new agent %d in the map. Something went wrong, aborting.", msg.agent_id);
            exit(EXIT_FAILURE);
        }
        auto& [id, agent_ptr] = *it;

        // Default ModelParams is the x500 configuration
        MultirotorModel::ModelParams x500_params = MultirotorModel::ModelParams();
        // Enable ground
        x500_params.ground_enabled = true;

        agent_ptr->uav_system = std::make_shared<UavSystem>(x500_params, agent_ptr->position, agent_ptr->heading);
    }
}

void MiniDancers::DisplayRviz()
{
    // Display robots
    geometry_msgs::msg::PoseArray pose_array{};
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->get_clock()->now();
    visualization_msgs::msg::MarkerArray id_marker_array{};
    visualization_msgs::msg::MarkerArray desired_velocities_marker_array{};
    std::shared_lock lock(this->agents_mutex_);
    for (auto& [agent_id, agent_struct] : this->agents_)
    {
        // Display poses
        MultirotorModel::State uav_state = agent_struct->uav_system->getState();
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
        p2.x = uav_state.x.x() + agent_struct->cmd_velocity.x();
        p2.y = uav_state.x.y() + agent_struct->cmd_velocity.y();
        p2.z = uav_state.x.z() + agent_struct->cmd_velocity.z();
        desired_velocity_marker.points.push_back(p1);
        desired_velocity_marker.points.push_back(p2);
        desired_velocities_marker_array.markers.push_back(desired_velocity_marker);
    }
    this->pose_array_pub_->publish(pose_array);
    this->id_markers_pub_->publish(id_marker_array);
    this->desired_velocities_markers_pub_->publish(desired_velocities_marker_array);

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniDancers>());
    rclcpp::shutdown();
    return 0;
}
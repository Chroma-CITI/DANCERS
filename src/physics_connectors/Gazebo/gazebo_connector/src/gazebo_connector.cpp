#include <connector_core/connector.hpp>

// Common DANCERS libs
#include <agent.hpp>
#include <yaml_util.hpp>

// Gazebo libs
#include <gz/sim.hh>
#include <gz/msgs.hh>

// C++ libs
#include <shared_mutex>
#include <filesystem>

// Costum ROS2 messages
#include <dancers_msgs/msg/agent_struct.hpp>

// ROS2 messages
#include <geometry_msgs/msg/point.hpp>

// Protobuf messages
#include <protobuf_msgs/pose_vector.pb.h>

using namespace mrs_multirotor_simulator;

class GazeboConnector : public Connector
{
public:
    GazeboConnector()
    : Connector("gazebo_connector")
    {
        RCLCPP_INFO(this->get_logger(), "GazeboConnector (PHY) started");

        this->ReadConfigFile();

        // Check that the PX4 Autopilot exists at config_["path_to_px4_autopilot"] and has been built
        this->path_to_px4_autopilot = getYamlValue<std::string>(this->config_, "path_to_px4_autopilot");
        if (!std::filesystem::exists(this->path_to_px4_autopilot))
        {
            RCLCPP_FATAL(this->get_logger(), "PX4 Autopilot not found at %s, aborting.", this->path_to_px4_autopilot.c_str());
            exit(EXIT_FAILURE);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "PX4 Autopilot found at %s", this->path_to_px4_autopilot.c_str());
            std::string px4_executable_path = this->path_to_px4_autopilot + "/build/px4_sitl_default/bin/px4";
            if (!std::filesystem::exists(px4_executable_path))
            {
                RCLCPP_FATAL(this->get_logger(), "PX4 SITL executable not found at %s, is it built? Aborting.", px4_executable_path.c_str());
                exit(EXIT_FAILURE);
            }
        }

        // Since we are using the PX4 Autopilot, we need to check some environment variables before launching Gazebo
        // Set the env. var. GZ_SIM_RESOURCE_PATH and GZ_SIM_SERVER_CONFIG_PATH to find models and server.config
        if (getenv("GZ_SIM_RESOURCE_PATH") == NULL)
        {
            setenv("GZ_SIM_RESOURCE_PATH", (this->path_to_px4_autopilot+"/Tools/simulation/gz/models").c_str(), 0);
            RCLCPP_INFO(this->get_logger(), "Using GZ_SIM_RESOURCE_PATH=%s", (this->path_to_px4_autopilot+"/Tools/simulation/gz/server.config").c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using already set GZ_SIM_RESOURCE_PATH=%s", getenv("GZ_SIM_RESOURCE_PATH"));
        }
        if (getenv("GZ_SIM_SERVER_CONFIG_PATH") == NULL)
        {
            setenv("GZ_SIM_SERVER_CONFIG_PATH", (this->path_to_px4_autopilot+"/Tools/simulation/gz/server.config").c_str(), 0);
            RCLCPP_INFO(this->get_logger(), "Using GZ_SIM_SERVER_CONFIG_PATH=%s", (this->path_to_px4_autopilot+"/Tools/simulation/gz/server.config").c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using already set GZ_SIM_SERVER_CONFIG_PATH=%s", getenv("GZ_SIM_RESOURCE_PATH"));
        }

        // Starts the Gazebo server with the right world
        this->LaunchGazebo();

        // Initializes the agents present at the start of the simulation
        this->InitAgents();

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&GazeboConnector::Loop, this);
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
        // Receive a list of waypoints and control the UAVs in position
        if (!update_msg.payload().empty())
        {
            dancers_update_proto::PoseVector waypoint_messages;
            waypoint_messages.ParseFromString(gzip_decompress(update_msg.payload()));
            // For each waypoint received, update the position controller of the associated agent
            for (const auto& waypoint : waypoint_messages.pose())
            {
                std::unique_lock lock(this->agents_mutex_);
                auto agent = this->agents_.find(waypoint.agent_id());
                if (agent != this->agents_.end())
                {
                    geometry_msgs::msg::Point wp;
                    wp.set__x(waypoint.x() + agent->second->initial_position.x());
                    wp.set__y(waypoint.y() + agent->second->initial_position.y());
                    wp.set__z(waypoint.z() + agent->second->initial_position.z());
                    this->waypoint_publishers_[agent->first]->publish(wp);
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

        // Arguments are blocking, iterations, continue after
        this->server_ptr->Run(true, 1, false);
        
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
            robot_pose_msg->set_agent_id(agent_ptr->id);
            robot_pose_msg->set_x(agent_ptr->position.x());
            robot_pose_msg->set_y(agent_ptr->position.y());
            robot_pose_msg->set_z(agent_ptr->position.z());
            robot_pose_msg->set_vx(agent_ptr->velocity.x());
            robot_pose_msg->set_vy(agent_ptr->velocity.y());
            robot_pose_msg->set_vz(agent_ptr->velocity.z());
        }

        std::string serialized_msg;
        robots_positions_msg.SerializeToString(&serialized_msg);

        response_msg.set_payload(gzip_compress(serialized_msg));

        return response_msg;
    }

    void InitAgents();

    void LaunchGazebo();

    // Gazebo callbacks
    void GzPoseInfoClbk(const gz::msgs::Pose_V &pose_msg);

    // ROS Callbacks
    void SpawnUavClbk(const dancers_msgs::msg::AgentStruct msg);

    std::string path_to_px4_autopilot;

    std::unique_ptr<gz::sim::Server> server_ptr;

    std::map<uint32_t, std::unique_ptr<Agent>> agents_;
    mutable std::shared_mutex agents_mutex_;

    // publishers
    std::map<uint32_t, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> waypoint_publishers_;
};

/**
 * @brief Initializes the agents present at the start of the simulation 
 * 
 * It works by calling a callback that actually spawns the robots, with a custom ROS2 message of type AgentStruct
 */
void GazeboConnector::InitAgents()
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

/**
 * @brief Callback that adds a UAV to the simulation
 * 
 * With the PX4 Autopilot and Gazebo, spawning a UAV consists in starting the autopilot in a separate thread. This methods requires the PX4 Autopilot to be pre-built.  
 */
void GazeboConnector::SpawnUavClbk(const dancers_msgs::msg::AgentStruct msg)
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

    int airframe = 4001;

    std::string cmd =
        "PX4_SYS_AUTOSTART=" + std::to_string(airframe) + " " +
        "PX4_SIM_MODEL=gz_" + getYamlValue<std::string>(this->config_, "robot_model") + " " +
        "PX4_GZ_MODEL_POSE=" + std::to_string(msg.state.position.x) + "," + std::to_string(msg.state.position.y) + "," + std::to_string(msg.state.position.z) + ",0,0,0,0 " +
        "PX4_PARAM_NAV_DLL_ACT=0 " +
        this->path_to_px4_autopilot + "/build/px4_sitl_default/bin/px4 " +
        "-i " + std::to_string(msg.agent_id) +
        " > dancers_data/"+getYamlValue<std::string>(this->config_, "experiment_name")+"/instance_"+getYamlValue<std::string>(this->config_, "instance_id")+"/results/px4_" + std::to_string(msg.agent_id) + ".log 2>&1";

    RCLCPP_INFO(this->get_logger(), "Spawning PX4-powered UAV %d with model %s", msg.agent_id, getYamlValue<std::string>(this->config_, "robot_model").c_str());
    std::thread([cmd]() {
        // Redirect output to log files to avoid stdout clutter
        int ret = std::system(cmd.c_str());
        if (ret != 0)
            std::cerr << "[PX4-Launcher] PX4 exited with code " << ret << std::endl;
    }).detach();

    // Create a publisher for the waypoint of this agent
    this->waypoint_publishers_[msg.agent_id] = 
        this->create_publisher<geometry_msgs::msg::Point>(
            "px4_" + std::to_string(msg.agent_id) + "/waypoint",
            10
    );

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
        agent_ptr->initial_position = agent_ptr->position;
        agent_ptr->initial_position_saved = true;
    }
}

/**
 * @brief Configures and starts te Gazebo server
 * 
 * We use the Gazebo libraries to configure the server and starting. It is useful to keep a pointer to the gz::sim::Server object for future use.
 * Interactions with the server can also happen with the gz::transport library, because not everything can be done with the ServerConfig.
 */
void GazeboConnector::LaunchGazebo()
{
    gz::sim::ServerConfig serverConfig;
    serverConfig.SetSdfFile(getYamlValue<std::string>(this->config_, "world_file"));
    serverConfig.SetSeed(getYamlValue<unsigned int>(this->config_, "seed"));
    serverConfig.SetUpdateRate(std::numeric_limits<double>::infinity());

    // Instantiate server
    this->server_ptr = std::make_unique<gz::sim::Server>(serverConfig);
    RCLCPP_INFO(this->get_logger(), "Gazebo server started.");

    gz::transport::Node node;

    unsigned int serviceTimeoutMs = 2000;
    
    // Request the world's name to verify it has started correctly (blocking)
    std::string world_name;
    {
        gz::msgs::StringMsg_V worldsResp;
        bool result = false;
        bool executed = node.Request("/gazebo/worlds", serviceTimeoutMs, worldsResp, result);
        if (executed && result && worldsResp.data_size() > 0)
        {
            world_name = worldsResp.data(0);
            RCLCPP_INFO(this->get_logger(), "World found with name : %s", world_name.c_str());
        }
    }

    // Set the MaxStepSize parameter
    {
        std::string service = "/world/" + world_name + "/set_physics";

        gz::msgs::Physics request;
        request.set_max_step_size(getYamlValue<double>(this->config_, "phy_step_size") / 1000000.0); // us to s

        gz::msgs::Boolean reply;
        bool result = false;
        bool executed = node.Request(service, request, serviceTimeoutMs, reply, result);

        if (!executed)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out for: %s", service.c_str());
            exit(EXIT_FAILURE);
        }
        if (!result || !reply.data())
        {
            RCLCPP_ERROR(this->get_logger(), "Service returned failure for: %s", service.c_str());
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(this->get_logger(), "Successfully set the MaxStepSize parameter to: %f", request.max_step_size());
    }

    if (node.Subscribe("/world/" + world_name + "/pose/info", &GazeboConnector::GzPoseInfoClbk, this))
    {
        RCLCPP_INFO(this->get_logger(), "Subscribed to /world/%s/pose/info", world_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to /world/%s/pose/info", world_name.c_str());
        exit(EXIT_FAILURE);
    }

}

/**
 * @brief Callback function for the /world/<world_name>/pose/info topic
 * 
 * Saves the latest robots' positions' in the agents_ map.
 */
void GazeboConnector::GzPoseInfoClbk(const gz::msgs::Pose_V &pose_msg)
{
    std::unique_lock lock(this->agents_mutex_);
    for (auto& [agent_id, agent_ptr] : this->agents_)
    {
        std::string robot_name_entity = getYamlValue<std::string>(this->config_, "robot_model") + "_" + std::to_string(agent_id);
        // Find in the protobuf message if an entity has that name
        bool robot_found = false;
        int i = 0;
        for (; i < pose_msg.pose_size(); i++)
        {
            if (pose_msg.pose(i).name() == robot_name_entity)
            {
                robot_found = true;
                break;
            }
        }
        if (!robot_found)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find Gazebo position for entity %s, skipping.", robot_name_entity.c_str());
            break;
        }
        const auto& pose = pose_msg.pose(i);
        agent_ptr->position[0] = pose.position().x();
        agent_ptr->position[1] = pose.position().y();
        agent_ptr->position[2] = pose.position().z();
        // Convert quaternion to yaw
        double qw = pose.orientation().w();
        double qx = pose.orientation().x();
        double qy = pose.orientation().y();
        double qz = pose.orientation().z();
        agent_ptr->heading = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboConnector>());
    rclcpp::shutdown();
    return 0;
}
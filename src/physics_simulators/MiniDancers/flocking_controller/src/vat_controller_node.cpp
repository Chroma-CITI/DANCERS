#include <functional>
#include <memory>
#include <memory>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "vat_controller.hpp"

#include <flocking_controller/agent_util.hpp>
#include <flocking_controller/cuboid_obstacle_util.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dancers_msgs/msg/velocity_heading_array.hpp"
#include "dancers_msgs/srv/get_agent_velocities.hpp"
#include "dancers_msgs/msg/target.hpp"
#include "dancers_msgs/msg/agent_struct.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

#include "occupancy_grid.hpp"
#include "grid_path_planner.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class VATControllerNode : public rclcpp::Node
{
public:
    VATControllerNode() : Node("vat_controller"), obstacles_(std::make_shared<std::vector<cuboid::obstacle_t>>())
    {
        // Get the config file
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);

        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Parse the config file
        this->config = YAML::LoadFile(config_file_path);

        this->simEndTime = rclcpp::Time(config["simulation_length"].as<double>(), 0);

        // Initialize obstacles
        obstaclesInitialization(config);

        this->secondary_objectives = getAgentSecondaryObjective(config);

        // Initialize the
        occupancyGridAndPlannerInitialization(config, obstacles_, secondary_objectives);

        for (auto i : this->secondary_objectives)
        {
            RCLCPP_WARN(this->get_logger(), "Secondary objective %d: (%f,%f,%f)", i.first, i.second.x(), i.second.y(), i.second.z());
        }

        // Initialize the controllers
        agentControllerInitialization(config, secondary_objectives);

        should_controller_publish_cmd_ = config["controller_publish_cmd"].as<bool>();

        if (should_controller_publish_cmd_)
        {
            cmd_publisher_ = this->create_publisher<dancers_msgs::msg::VelocityHeadingArray>("velocity_cmd", 10);
        }

        rclcpp::QoS reliable_qos(10); // depth of 10
        reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        reliable_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        reliable_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        // Initialize subscribers
        std::string events_namespace = "/events/";
        this->clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&VATControllerNode::clockCallback, this, _1));
        this->target_position_subscriber_ = this->create_subscription<dancers_msgs::msg::Target>(
            events_namespace+"update_target", 
            reliable_qos, 
            std::bind(&VATControllerNode::targetPositionCallback, this, _1));
        this->spawn_uav_sub_ = this->create_subscription<dancers_msgs::msg::AgentStruct>(
            events_namespace+"spawn_uav", 
            reliable_qos,
            std::bind(&VATControllerNode::spawnUavCallback, this, _1));
        this->uav_failure_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            events_namespace+"uav_failure", 
            reliable_qos,
            std::bind(&VATControllerNode::UavFailureCallback, this, _1));

        // Create the service
        service_ = this->create_service<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities",
                                                                               std::bind(&VATControllerNode::commandCallback, this, _1, _2));

        RCLCPP_DEBUG(this->get_logger(), "Flocking controller's service initialized.");
    }

private:
    /**
     * @brief Structure combining the a path planner to its ROS publisher.
     */
    struct PlannerAndPublisher
    {
        std::shared_ptr<GridPathPlanner> path_planner_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    };

    /**
     * @brief Flag that indicates if the controller should publish its service response on a topic.
     * Mainly used for debugging.
     */
    bool should_controller_publish_cmd_ = false;

    /**
     * @brief The YAML object holding the configuration of the simulation.
     */
    YAML::Node config;

    /**
     * @brief The secondary objectives of the agents. The key is the agent's id, the value is a 3D vector giving the global position off the target area.
     */
    std::map<int, Eigen::Vector3d> secondary_objectives;

    rclcpp::Time simEndTime;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Subscription<dancers_msgs::msg::AgentStruct>::SharedPtr spawn_uav_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr uav_failure_sub_;

    bool shutdown_triggered_ = false;
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void spawnUavCallback(const dancers_msgs::msg::AgentStruct agent);
    void UavFailureCallback(const std_msgs::msg::UInt32 agent_id);

    VATController::VAT_params_t default_vat_params_ = {
        .v_flock = 1.5,
        .v_max = 1.0,
        .v_sec_max = 1.0,
        .a_frict = 4.16,
        .p_frict = 3.2,
        .r_0_frict = 85.3,
        .C_frict = 0.8,
        .v_frict = 0.6,
        .p_att = 0.08,
        .r_0_att = 15,
        .p_rep = 0.13,
        .r_0_rep = 15,
        .a_shill = 53,
        .p_shill = 3.55,
        .r_0_shill = 0.3,
        .v_shill = 13.622};

    /**
     * @brief Service server used to return agent_velocities when called.
     */
    rclcpp::Service<dancers_msgs::srv::GetAgentVelocities>::SharedPtr service_;

    /**
     * @brief Publisher that publishes the response of the its service computing the velocities command.
     */
    rclcpp::Publisher<dancers_msgs::msg::VelocityHeadingArray>::SharedPtr cmd_publisher_;

    /**
     * @brief Publisher that publishes the occupancy grid of the obstacles.
     */
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;

    /**
     * @brief Publisher that publishes the pose of the waypoints of the path planners.
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_publisher_;

    /**
     * @brief List of all the planners and their corresponding publisher.
     */
    std::vector<PlannerAndPublisher> path_planners_and_pub_;

    /**
     * @brief Map containing the controllers for each agent. The key is the agent's id.
     */
    std::map<uint32_t, std::unique_ptr<VATController>> controllers_;

    /**
     * @brief Obstacle list to avoid. Used in the shillTerm().
     */
    std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles_;

    /**
     * @brief 2D Occupancy grid of the obstacles.
     */
    std::shared_ptr<OccupancyGrid2D> occupancy_grid_ptr_;

    /**
     * @brief Path planner in the associated occupancy grid.
     */
    std::shared_ptr<GridPathPlanner> path_planner_;

    /**
     * @brief Timer used to periodically call the publisher of the occupancy grid.
     */
    rclcpp::TimerBase::SharedPtr occupancy_grid_pub_timer_;

    /**
     * @brief Subscriber to get updates of the target positions
     */
    rclcpp::Subscription<dancers_msgs::msg::Target>::SharedPtr target_position_subscriber_;

    void targetPositionCallback(dancers_msgs::msg::Target msg)
    {
        for (auto concerned_agent : msg.concerned_agents)
        {
            if (concerned_agent == -1)
            {
                for (auto &&controller : controllers_)
                {
                    controller.second->SetSecondaryObjective(Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z));
                }
                break;
            }
            else
            {
                controllers_[concerned_agent]->SetSecondaryObjective(Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z));
            }
        }
    }

    /**
     * @brief Callback that computes all the desired velocities of a group of agent given their self states, neighbor states and obstacles.
     */
    void commandCallback(const std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Request> request,
                         std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response)
    {
        RCLCPP_DEBUG(this->get_logger(), "flocking_controller received a request to compute agent velocities.");

        if (request->agent_structs.size() != controllers_.size())
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "The flocking_controller received a request with a agent number(" << request->agent_structs.size() << ") that is different from its expected number (" << controllers_.size() << "). Won't compute velocities.");
        }

        // for debug only
        // this->printCommandRequest(request);

        for (const auto &agent_struct : request->agent_structs)
        {
            agent_util::AgentState_t self_agent_state = agent_util::create_agent_state_from_ROS_message(agent_struct);
            if (controllers_[agent_struct.agent_id])
            {
                dancers_msgs::msg::VelocityHeading agent_command = controllers_[agent_struct.agent_id]->getVelocityHeading(self_agent_state, *obstacles_);
                response->velocity_headings.velocity_heading_array.push_back(std::move(agent_command));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "The flocking_controller received a request for agent %d that is not in the list of controllers. Won't compute velocities for this agent.", agent_struct.agent_id);
            }
        }
        if (should_controller_publish_cmd_)
        {
            cmd_publisher_->publish(response->velocity_headings);
        }
    }

    /**
     * @brief Initialize the obstacle list from the a given configuration file.
     * @param config YAML configuration stucture containing the obstacles.
     */
    void obstaclesInitialization(YAML::Node &config)
    {
        if (obstacles_)
        {
            for (auto &&building : config["buildings"])
            {
                double x = building["x"].as<double>();
                double y = building["y"].as<double>();
                double z = building["height"].as<double>() / 2;
                double size_x = building["size_x"].as<double>();
                double size_y = building["size_y"].as<double>();
                double height = building["height"].as<double>();

                cuboid::obstacle_t obs{};
                obs.id = building["id"].as<int>();
                obs.center = Eigen::Vector3d(x, y, z);
                obs.size_x = size_x;
                obs.size_y = size_y;
                obs.size_z = height;
                this->obstacles_->push_back(std::move(obs));
            }
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "The obstacle list is a null ptr");
            exit(EXIT_FAILURE);
        }
    }

    /**
     * @brief Initialize the 2D occupancy grid with obstacles where its size is
     * determined by the obstacles and secondary objectives position plus a buffer.
     * Also initialize a global plath planner using the occupancy grid.
     * @param obstacles List of obstacles to put in the occupancy grid.
     * @param secondary_objectives List of all the secondary objectives
     */
    void occupancyGridAndPlannerInitialization(const YAML::Node &config,
                                               std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles,
                                               std::map<int, Eigen::Vector3d> &secondary_objectives)
    {
        if (obstacles)
        {

            // Determine the size of the occupancy grid

            float min_relevant_x = 0.0f;
            float max_relevant_x = 0.0f;

            float min_relevant_y = 0.0f;
            float max_relevant_y = 0.0f;

            if (obstacles->size() != 0)
            {
                min_relevant_x = (*obstacles)[0].center[0] - (*obstacles)[0].size_x / 2;
                max_relevant_x = (*obstacles)[0].center[0] + (*obstacles)[0].size_x / 2;

                min_relevant_y = (*obstacles)[0].center[1] - (*obstacles)[0].size_y / 2;
                max_relevant_y = (*obstacles)[0].center[1] + (*obstacles)[0].size_y / 2;
            }

            for (auto secondary_objective = secondary_objectives.begin(); secondary_objective != secondary_objectives.end(); secondary_objective++)
            {
                Eigen::Vector3d &objective_position = secondary_objective->second;

                if ((obstacles->size() == 0) && secondary_objective == secondary_objectives.begin())
                {
                    min_relevant_x = objective_position[0];
                    max_relevant_x = objective_position[0];

                    min_relevant_y = objective_position[1];
                    max_relevant_y = objective_position[1];
                }
                else
                {
                    if (objective_position[0] < min_relevant_x)
                    {
                        min_relevant_x = objective_position[0];
                    }
                    if (max_relevant_x < objective_position[0])
                    {
                        max_relevant_x = objective_position[0];
                    }
                    if (objective_position[1] < min_relevant_y)
                    {
                        min_relevant_y = objective_position[1];
                    }
                    if (max_relevant_y < objective_position[1])
                    {
                        max_relevant_y = objective_position[1];
                    }
                }
            }

            for (int obstacle_index = 1; obstacle_index < (*obstacles).size(); obstacle_index++)
            {
                const cuboid::obstacle_t &obstacle = (*obstacles)[obstacle_index];

                if ((obstacle.center[0] - obstacle.size_x / 2) < min_relevant_x)
                {
                    min_relevant_x = obstacle.center[0] - obstacle.size_x / 2;
                }
                if (max_relevant_x < (obstacle.center[0] + obstacle.size_x / 2))
                {
                    max_relevant_x = obstacle.center[0] + obstacle.size_x / 2;
                }
                if ((obstacle.center[1] - obstacle.size_y / 2) < min_relevant_y)
                {
                    min_relevant_y = obstacle.center[1] - obstacle.size_y / 2;
                }
                if (max_relevant_y < (obstacle.center[1] + obstacle.size_y / 2))
                {
                    max_relevant_y = obstacle.center[1] + obstacle.size_y / 2;
                }
            }

            float grid_altitude = 10.0f;
            if (YAML::Node altitude_param = config["grid_altitude"])
            {
                grid_altitude = static_cast<float>(altitude_param.as<double>());
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "The param grid_altitude is not set. The default value of " << grid_altitude << " is used.");
            }

            float obstacle_inflation = 0.0f;
            if (YAML::Node obstacle_inflation_param = config["obstacle_inflation"])
            {
                obstacle_inflation = static_cast<float>(obstacle_inflation_param.as<double>());
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "The param obstacle_inflation is not set. The default value of " << obstacle_inflation << " is used.");
            }

            float map_inflation = 20.0f;
            if (YAML::Node map_inflation_param = config["map_inflation"])
            {
                map_inflation = static_cast<float>(map_inflation_param.as<double>());
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "The param map_inflation is not set. The default value of " << map_inflation << " is used.");
            }

            float map_resolution = 0.5f;
            if (YAML::Node map_resolution_param = config["map_resolution"])
            {
                map_resolution = static_cast<float>(map_resolution_param.as<double>());
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "The param map_resolution is not set. The default value of " << map_resolution << " is used.");
            }

            Eigen::Vector3d origin = {min_relevant_x - map_inflation, min_relevant_y - map_inflation, grid_altitude};
            Eigen::Vector3d opposite_corner = {max_relevant_x + map_inflation, max_relevant_y + map_inflation, grid_altitude};

            occupancy_grid_ptr_ = std::make_shared<OccupancyGrid2D>(origin, opposite_corner, map_resolution, obstacle_inflation, OccupancyGrid2D::CellStatus::Free);
            occupancy_grid_ptr_->populateGridFromObstacles(obstacles);

            path_planner_ = std::make_shared<GridPathPlanner>(occupancy_grid_ptr_);

            occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("flocking_controller_occupancy_grid", 10);
            occupancy_grid_pub_timer_ = this->create_wall_timer(100ms, std::bind(&VATControllerNode::occupancy_grid_and_path_time_callback, this));
        }
    }

    void occupancy_grid_and_path_time_callback()
    {
        if (occupancy_grid_ptr_ && occupancy_grid_publisher_)
        {
            geometry_msgs::msg::PoseArray waypoint_array;
            waypoint_array.header.stamp = this->now();
            waypoint_array.header.frame_id = "map";

            nav_msgs::msg::OccupancyGrid grid_msg = this->occupancy_grid_ptr_->getOccupancyGridMsg();
            // Add missing time in the message
            grid_msg.header.stamp = this->now();
            grid_msg.info.map_load_time = this->now();

            for (PlannerAndPublisher &planner_and_pub : path_planners_and_pub_)
            {
                nav_msgs::msg::Path path = planner_and_pub.path_planner_->getPath();
                path.header.stamp = this->now();
                path.header.frame_id = "map";
                for (geometry_msgs::msg::PoseStamped &pose : path.poses)
                {
                    pose.header.stamp = this->now();
                    pose.header.frame_id = "map";
                }
                planner_and_pub.publisher_->publish(path);

                PathPlanner::Waypoint waypoint = planner_and_pub.path_planner_->getCurrentWaypoint();
                geometry_msgs::msg::Pose waypoint_pose;
                waypoint_pose.position.x = waypoint.position[0];
                waypoint_pose.position.y = waypoint.position[1];
                waypoint_pose.position.z = waypoint.position[2];
                waypoint_array.poses.push_back(waypoint_pose);
            }

            if (occupancy_grid_publisher_)
            {
                occupancy_grid_publisher_->publish(grid_msg);
            }
            if (waypoint_publisher_)
            {
                waypoint_publisher_->publish(waypoint_array);
            }
        }
    }

    /**
     * @brief Gets VAT parameters from a a given YAML list name and populate a VAT_params_t struct.
     * @param config YAML config to fetch the list from
     * @param vat_params_config_list_name Variable name of the YAML list containing the VAT parameters
     * @param vat_params Output parameter struct populated from the YAML file.
     */
    bool populateVATParametersFromConfig(const YAML::Node &config,
                                         const std::string &vat_params_config_list_name,
                                         VATController::VAT_params_t &vat_params)
    {
        try
        {
            vat_params.v_flock = config[vat_params_config_list_name]["v_flock"].as<double>();
            vat_params.v_max = config[vat_params_config_list_name]["v_max"].as<double>();
            vat_params.v_sec_max = config[vat_params_config_list_name]["v_sec_max"].as<double>();
            vat_params.r_0_sec = config[vat_params_config_list_name]["r_0_sec"].as<double>();
            vat_params.v_sec_max_path = config[vat_params_config_list_name]["v_sec_max_path"].as<double>();
            vat_params.r_0_sec_path = config[vat_params_config_list_name]["r_0_sec_path"].as<double>();
            vat_params.a_frict = config[vat_params_config_list_name]["a_frict"].as<double>();
            vat_params.p_frict = config[vat_params_config_list_name]["p_frict"].as<double>();
            vat_params.r_0_frict = config[vat_params_config_list_name]["r_0_frict"].as<double>();
            vat_params.C_frict = config[vat_params_config_list_name]["C_frict"].as<double>();
            vat_params.v_frict = config[vat_params_config_list_name]["v_frict"].as<double>();
            vat_params.r_0_att = config[vat_params_config_list_name]["r_0_att"].as<double>();
            vat_params.p_rep = config[vat_params_config_list_name]["p_rep"].as<double>();
            vat_params.r_0_rep = config[vat_params_config_list_name]["r_0_rep"].as<double>();
            vat_params.a_shill = config[vat_params_config_list_name]["a_shill"].as<double>();
            vat_params.p_shill = config[vat_params_config_list_name]["p_shill"].as<double>();
            vat_params.r_0_shill = config[vat_params_config_list_name]["r_0_shill"].as<double>();
            vat_params.v_shill = config[vat_params_config_list_name]["v_shill"].as<double>();
            vat_params.p_los = config[vat_params_config_list_name]["p_los"].as<double>();
            vat_params.r_los_obst_inflation = config[vat_params_config_list_name]["r_los_obst_inflation"].as<double>();
            vat_params.use_deconnexion_distance_instead_of_p_att = config[vat_params_config_list_name]["use_deconnexion_distance_instead_of_p_att"].as<bool>();
            if (vat_params.use_deconnexion_distance_instead_of_p_att)
            {
                float v_sec_max = (config["use_planner"].as<bool>()) ? vat_params.v_sec_max_path : vat_params.v_sec_max;
                vat_params.expected_deconnexion_distance = config["expected_deconnexion_distance"].as<double>();
                const double relative_distance = vat_params.expected_deconnexion_distance - vat_params.r_0_att;
                if (relative_distance > 0.0)
                {
                    vat_params.p_att = vat_params.v_sec_max / (relative_distance);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Can't compute p_att based on expected_deconnexion_distance if expected_deconnexion_distance is smaller than r_0_att");
                    vat_params.p_att = config[vat_params_config_list_name]["p_att"].as<double>();
                }
            }
            else
            {
                vat_params.p_att = config[vat_params_config_list_name]["p_att"].as<double>();
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            RCLCPP_WARN_STREAM(this->get_logger(), "Failed to read at least one VAT flocking parameter of the list " << vat_params_config_list_name << ", using ALL default VAT params.");
            vat_params = default_vat_params_;
        }
    }

    /**
     * @brief Get the secondary objectives serving as end goal for specified agents from the config file.
     * @param config YAML configuration stucture containing the secondary objectives.
     * @return A map containing the secondary objectives where the key is the agent id and the value the cuboid obstacle.
     */
    std::map<int, Eigen::Vector3d> getAgentSecondaryObjective(YAML::Node &config)
    {
        // Get secondary objectives from config files
        std::map<int, Eigen::Vector3d> secondary_objectives;
        for (auto goal : config["secondary_objectives"])
        {
            Eigen::Vector3d objective = Eigen::Vector3d(goal["position"]["x"].as<double>(), goal["position"]["y"].as<double>(), goal["position"]["z"].as<double>());

            for (auto concerned_agent : goal["assigned_agents"])
            {
                int agent_id = concerned_agent.as<int>();

                if (agent_id == -1)
                {
                    for (int i = 0; i < config["robots_number"].as<int>(); i++)
                    {
                        if (!secondary_objectives.emplace(i, objective).second)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Conflicting objectives for agent " << i << " : " << objective.transpose() << " ! Exiting.");
                            exit(EXIT_FAILURE);
                        }
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "Goal of all agents : " << objective.transpose() << std::endl);
                    break;
                }
                else
                {
                    if (!secondary_objectives.emplace(agent_id, objective).second)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "Conflicting objectives for agent " << agent_id << " : " << objective.transpose() << " ! Exiting.");
                        exit(EXIT_FAILURE);
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "Goal of agent " << agent_id << " : " << secondary_objectives[agent_id].transpose() << std::endl);
                }
            }
        }
        return secondary_objectives;
    }

    /**
     * @brief Initialize the controller of each agent based on a configuration file.
     * @param config YAML configuration stucture containing the obstacles.
     * @param secondary_objectives Map of secondary objective positions for agent specified through their id in the keys of the map.
     */
    void agentControllerInitialization(YAML::Node &config, std::map<int, Eigen::Vector3d> &secondary_objectives)
    {
        // Get the fixed altitude option
        std::optional<float> fixed_altitude;
        if (YAML::Node altitude_param = config["target_altitude"])
        {
            fixed_altitude = altitude_param.as<float>();
        }

        // Get the min altitude option
        std::optional<float> min_altitude;
        if (YAML::Node min_altitude_param = config["min_altitude"])
        {
            min_altitude = min_altitude_param.as<float>();
        }

        // Verify if planner should be used.
        bool use_planner = config["use_planner"].as<bool>();
        // Initialize controllers
        const int number_of_agents = config["robots_number"].as<int>();
        for (int agent_id = 0; agent_id < number_of_agents; agent_id++)
        {
            VATController::ControllerOptions_t options;
            options.id = agent_id;

            // Get the flocking parameters
            populateVATParametersFromConfig(config, "VAT_undefined_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Undefined]);
            populateVATParametersFromConfig(config, "VAT_mission_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Mission]);
            populateVATParametersFromConfig(config, "VAT_potential_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Potential]);
            populateVATParametersFromConfig(config, "VAT_idle_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Idle]);

            // Planner configuration
            options.path_planner_params_.use_planner_ = use_planner;
            if (options.path_planner_params_.use_planner_)
            {
                options.path_planner_params_.goal_radius_tolerance_ = static_cast<float>(config["goal_radius_tolerance"].as<double>());
                options.path_planner_params_.distance_to_path_tolerance_ = static_cast<float>(config["distance_to_path_tolerance"].as<double>());
                options.path_planner_params_.lookup_ahead_pursuit_distance_ = static_cast<float>(config["lookup_ahead_pursuit_distance"].as<double>());
            }

            // Get the altitude parameters
            options.desired_fixed_altitude = fixed_altitude;

            options.desired_min_altitude = min_altitude;

            // Look for a secondary objective for the agent
            if (secondary_objectives.find(agent_id) != secondary_objectives.end())
            {
                options.secondary_objective = secondary_objectives[agent_id];
                RCLCPP_INFO_STREAM(this->get_logger(), "Initial goal of agent " << agent_id << " : " << secondary_objectives[agent_id].transpose() << std::endl);

                if (options.path_planner_params_.use_planner_)
                {
                    PlannerAndPublisher planner_and_publisher;
                    planner_and_publisher.path_planner_ = std::make_shared<GridPathPlanner>(occupancy_grid_ptr_);
                    std::string topic_prefix = "agent_path" + std::to_string(agent_id);
                    planner_and_publisher.publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix, 4);

                    options.path_planner_params_.path_planner_ = planner_and_publisher.path_planner_;
                    path_planners_and_pub_.push_back(planner_and_publisher);
                }
            }

            controllers_[agent_id] = std::make_unique<VATController>(options);
        }
        if (use_planner)
        {
            waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoint_poses", 5);
        }
    }

    void printCommandRequest(const std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Request> request)
    {
        std::string ROLES[4] = {"Undefined", "Mission", "Potential", "Idle"};
        std::cout << "Received command request for " << request->agent_structs.size() << " agents: " << std::endl;
        for (auto agent : request->agent_structs)
        {
            std::cout << agent.agent_id << "\t" << ROLES[static_cast<int>(agent.agent_role)] << std::endl;
            for (const auto &neighbor : agent.neighbor_array.neighbors)
            {
                std::cout << "\tid: " << neighbor.agent_id << "\trole: " << ROLES[static_cast<int>(neighbor.agent_role)] << "\tlinkQual: " << neighbor.link_quality << std::endl;
            }
            std::cout << std::endl;
        }
    }
};

void VATControllerNode::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    rclcpp::Time current_time = msg->clock;
    RCLCPP_DEBUG(this->get_logger(), "Simulation time: %f/%f", current_time.seconds(), this->simEndTime.seconds());

    if (!shutdown_triggered_ && current_time.seconds() >= simEndTime.seconds())
    {
        RCLCPP_WARN(this->get_logger(), "ROS time exceeded shutdown threshold. Shutting down...");
        shutdown_triggered_ = true;
        rclcpp::shutdown(); // This stops the entire ROS node
    }
}

void VATControllerNode::spawnUavCallback(const dancers_msgs::msg::AgentStruct agent)
{
    // Get the fixed altitude option
    std::optional<float> fixed_altitude;
    if (YAML::Node altitude_param = config["target_altitude"])
    {
        fixed_altitude = altitude_param.as<float>();
    }

    // Get the min altitude option
    std::optional<float> min_altitude;
    if (YAML::Node min_altitude_param = config["min_altitude"])
    {
        min_altitude = min_altitude_param.as<float>();
    }

    // Verify if planner should be used.
    bool use_planner = config["use_planner"].as<bool>();

    // Initialize controllers
    VATController::ControllerOptions_t options;
    options.id = agent.agent_id;

    // Get the flocking parameters
    populateVATParametersFromConfig(config, "VAT_undefined_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Undefined]);
    populateVATParametersFromConfig(config, "VAT_mission_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Mission]);
    populateVATParametersFromConfig(config, "VAT_potential_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Potential]);
    populateVATParametersFromConfig(config, "VAT_idle_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Idle]);

    // Planner configuration
    options.path_planner_params_.use_planner_ = use_planner;
    if (options.path_planner_params_.use_planner_)
    {
        options.path_planner_params_.goal_radius_tolerance_ = static_cast<float>(config["goal_radius_tolerance"].as<double>());
        options.path_planner_params_.distance_to_path_tolerance_ = static_cast<float>(config["distance_to_path_tolerance"].as<double>());
        options.path_planner_params_.lookup_ahead_pursuit_distance_ = static_cast<float>(config["lookup_ahead_pursuit_distance"].as<double>());
    }

    // Get the altitude parameters
    options.desired_fixed_altitude = fixed_altitude;

    options.desired_min_altitude = min_altitude;

    // Look for a secondary objective for the agent
    if (secondary_objectives.find(agent.agent_id) != secondary_objectives.end())
    {
        options.secondary_objective = secondary_objectives[agent.agent_id];
        RCLCPP_INFO_STREAM(this->get_logger(), "Goal of agent " << agent.agent_id << " : " << secondary_objectives[agent.agent_id].transpose() << std::endl);

        if (options.path_planner_params_.use_planner_)
        {
            PlannerAndPublisher planner_and_publisher;
            planner_and_publisher.path_planner_ = std::make_shared<GridPathPlanner>(occupancy_grid_ptr_);
            std::string topic_prefix = "agent_path" + std::to_string(agent.agent_id);
            planner_and_publisher.publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix, 4);

            options.path_planner_params_.path_planner_ = planner_and_publisher.path_planner_;
            path_planners_and_pub_.push_back(planner_and_publisher);
        }
    }

    controllers_[agent.agent_id] = std::make_unique<VATController>(options);

    RCLCPP_DEBUG(this->get_logger(), "UAV %d flocking controller correctly initialized.", agent.agent_id);
}

void VATControllerNode::UavFailureCallback(const std_msgs::msg::UInt32 agent_id)
{
    // Find the flocking controller of that agent and purely and simply delete it.
    for (auto it = controllers_.begin(); it != controllers_.end();)
    {
        if (it->first == agent_id.data)
        {
            it = controllers_.erase(it);
            RCLCPP_INFO(this->get_logger(), "UAV %d failure ! Flocking controller correctly deleted.", agent_id.data);
            break;  // We can "break" : there can't be multiple identical keys in controllers_ (it's a map)
        }
        else
        {
            ++it;
        }
    }
    RCLCPP_WARN(this->get_logger(), "UAV %d failure but this UAV has no flocking controller.", agent_id.data);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VATControllerNode>());
    rclcpp::shutdown();
    return 0;
}
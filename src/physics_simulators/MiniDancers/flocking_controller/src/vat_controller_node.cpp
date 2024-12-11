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

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "occupancy_grid.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class VATControllerNode : public rclcpp::Node
{
    public:
        VATControllerNode(): Node("vat_controller"), obstacles_(std::make_shared<std::vector<cuboid::obstacle_t>>())
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
            YAML::Node config = YAML::LoadFile(config_file_path);

            // Initialize obstacles
            obstaclesInitialization(config);

            std::map<int, Eigen::Vector3d> secondary_objectives = getAgentSecondaryObjective(config);

            // Initialize the 
            occupancyGridInitialization(obstacles_, secondary_objectives);

            // Initialize the controllers
            agentControllerInitialization(config, secondary_objectives);

            should_controller_publish_cmd_ = config["controller_publish_cmd"].as<bool>();

            if(should_controller_publish_cmd_)
            {
                cmd_publisher_ = this->create_publisher<dancers_msgs::msg::VelocityHeadingArray>("velocity_cmd", 10);
            }

            // Create the service
            service_ = this->create_service<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities", 
                                                                                  std::bind(&VATControllerNode::commandCallback,this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "Flocking controller's service initialized.");
        }

    private:
        /**
         * @brief Flag that indicates if the controller should publish its service respons on a topic.
         * Mainly used for debugging. 
         */
        bool should_controller_publish_cmd_ = false;

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
            .v_shill = 13.622
        };

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
         * @brief Map containing the controllers for each agent. The key is the agent's id. 
         */
        std::vector<std::unique_ptr<VATController>> controllers_;

        /**
         * @brief Obstacle list to avoid. Used in the shillTerm().
         */
        std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles_;

        /**
         * @brief 2D Occupancy grid of the obstacles.
         */
        std::shared_ptr<OccupancyGrid2D> occupancy_grid_ptr_;

        /**
         * @brief Timer used to periodically call the publisher of the occupancy grid.
         */
        rclcpp::TimerBase::SharedPtr occupancy_grid_pub_timer_;

        /**
         * @brief Callback that computes all the desired velocities of a group of agent given their self states, neighbor states and obstacles.
         */
        void commandCallback(const std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Request> request,
                                   std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response)
        {
           RCLCPP_DEBUG(this->get_logger(), "flocking_controller received a request to compute agent velocities.");
           
           if (request->agent_structs.size() != controllers_.size())
           {
                RCLCPP_ERROR_STREAM(this->get_logger(), "The flocking_controller received a request with a agent number("<< request->agent_structs.size() <<") that is different from its expected number ("<<controllers_.size()<<"). Won't compute velocities.");
           }
           else
           {
                std::vector<std::shared_ptr<const agent_util::AgentState_t>> agent_states;

                // Create a vector list of all the agent states.
                for(const auto& agent_struct: request->agent_structs)
                {
                    agent_util::AgentState_t self_agent_state = agent_util::create_agent_state_from_ROS_message(agent_struct);
                    agent_states.push_back(std::make_shared<agent_util::AgentState_t>(std::move(self_agent_state)));
                }
                
                for(const auto& agent_struct: request->agent_structs)
                {
                    dancers_msgs::msg::VelocityHeading agent_command = controllers_[agent_struct.agent_id]->getVelocityHeading(agent_states, *obstacles_);
                    response->velocity_headings.velocity_heading_array.push_back(std::move(agent_command));
                }
                if(should_controller_publish_cmd_)
                {
                    cmd_publisher_->publish(response->velocity_headings);
                }
           }
        }

        /**
         * @brief Initialize the obstacle list from the a given configuration file. 
         * @param config YAML configuration stucture containing the obstacles.
         */
        void obstaclesInitialization(YAML::Node& config)
        {
            if (obstacles_)
            {
                for (auto&& building : config["buildings"])
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
         * @param obstacles List of obstacles to put in the occupancy grid.
         * @param secondary_objectives List of all the secondary objectives
         */
        void occupancyGridInitialization(std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles,
                                         std::map<int, Eigen::Vector3d>& secondary_objectives)
        {
            if (obstacles)
            {
                float min_relevant_x = 0.0f;
                float max_relevant_x = 0.0f;

                float min_relevant_y = 0.0f;
                float max_relevant_y = 0.0f;

                if (obstacles->size() != 0)
                {
                    min_relevant_x = (*obstacles)[0].center[0] - (*obstacles)[0].size_x/2;
                    max_relevant_x = (*obstacles)[0].center[0] + (*obstacles)[0].size_x/2;

                    min_relevant_y = (*obstacles)[0].center[1] - (*obstacles)[0].size_y/2;
                    max_relevant_y = (*obstacles)[0].center[1] + (*obstacles)[0].size_y/2; 
                }

                for(auto secondary_objective = secondary_objectives.begin(); secondary_objective != secondary_objectives.end();  secondary_objective++)
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

                    if ((obstacle.center[0] - obstacle.size_x/2) < min_relevant_x)
                    {
                        min_relevant_x = obstacle.center[0] - obstacle.size_x/2;
                    }
                    if (max_relevant_x < (obstacle.center[0] + obstacle.size_x/2))
                    {
                        max_relevant_x = obstacle.center[0] + obstacle.size_x/2;
                    }
                    if ((obstacle.center[1] - obstacle.size_y/2) < min_relevant_y)
                    {
                        min_relevant_y = obstacle.center[1] - obstacle.size_y/2;
                    }
                    if (max_relevant_y < (obstacle.center[1] + obstacle.size_y/2))
                    {
                        max_relevant_y = obstacle.center[1] + obstacle.size_y/2;
                    }
                }

                // TODO get from params
                float grid_altitude = 10.0f;
                float map_inflation = 20.0f;

                Eigen::Vector3d origin = {min_relevant_x -map_inflation, min_relevant_y - map_inflation, grid_altitude};
                Eigen::Vector3d opposite_corner = {max_relevant_x + map_inflation, max_relevant_y + map_inflation, grid_altitude};

                occupancy_grid_ptr_ = std::make_shared<OccupancyGrid2D>(origin, opposite_corner, 0.5f, 1.0f, OccupancyGrid2D::CellStatus::Free);
                occupancy_grid_ptr_->populateGridFromObstacles(obstacles);
                
                occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("flocking_controller_occupancy_grid", 10);
                occupancy_grid_pub_timer_ = this->create_wall_timer(5s, std::bind(&VATControllerNode::occupancy_grid_time_callback, this));
            }
        }

        void occupancy_grid_time_callback()
        {
            nav_msgs::msg::OccupancyGrid grid_msg = this->occupancy_grid_ptr_->getOccupancyGridMsg();
            // Add missing time in the message
            grid_msg.header.stamp = this->now();
            grid_msg.info.map_load_time = this->now();

            occupancy_grid_publisher_->publish(grid_msg);
        }

        /**
         * @brief Gets VAT parameters from a a given YAML list name and populate a VAT_params_t struct.
         * @param config YAML config to fetch the list from
         * @param vat_params_config_list_name Variable name of the YAML list containing the VAT parameters
         * @param vat_params Output parameter struct populated from the YAML file.
         */
        bool populateVATParametersFromConfig(const YAML::Node& config, 
                                             const std::string& vat_params_config_list_name,
                                             VATController::VAT_params_t& vat_params)
        {
            try
            {
                vat_params.v_flock = config[vat_params_config_list_name]["v_flock"].as<double>();
                vat_params.v_max = config[vat_params_config_list_name]["v_max"].as<double>();
                vat_params.v_sec_max = config[vat_params_config_list_name]["v_sec_max"].as<double>();
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
                vat_params.use_deconnexion_distance_instead_of_p_att = config[vat_params_config_list_name]["use_deconnexion_distance_instead_of_p_att"].as<bool>();
                if (vat_params.use_deconnexion_distance_instead_of_p_att)
                {
                    vat_params.expected_deconnexion_distance = config["expected_deconnexion_distance"].as<double>();
                    const double relative_distance = vat_params.expected_deconnexion_distance - vat_params.r_0_att;
                    if (relative_distance > 0.0)
                    {
                        vat_params.p_att = vat_params.v_sec_max/(relative_distance);
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
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                RCLCPP_WARN_STREAM(this->get_logger(), "Failed to read at least one VAT flocking parameter of the list " << vat_params_config_list_name <<", using ALL default VAT params.");
                vat_params = default_vat_params_;
            }
        }

        /**
         * @brief Get the secondary objectives serving as end goal for specified agents from the config file.
         * @param config YAML configuration stucture containing the secondary objectives.
         * @return A map containing the secondary objectives where the key is the agent id and the value the cuboid obstacle.
         */
        std::map<int, Eigen::Vector3d> getAgentSecondaryObjective(YAML::Node& config)
        {
            // Get secondary objectives from config files
            std::map<int, Eigen::Vector3d> secondary_objectives;
            for (auto goal : config["secondary_objectives"])
            {
                Eigen::Vector3d objective = Eigen::Vector3d(goal.second[0].as<double>(), goal.second[1].as<double>(), goal.second[2].as<double>());
                secondary_objectives.insert({goal.first.as<int>(), objective});
            }
            return  secondary_objectives;
        }

        /**
         * @brief Initialize the controller of each agent based on a configuration file.
         * @param config YAML configuration stucture containing the obstacles.
         * @param secondary_objectives Map of secondary objective positions for agent specified through their id in the keys of the map.
         */
        void agentControllerInitialization(YAML::Node& config, std::map<int, Eigen::Vector3d>& secondary_objectives)
        {
            // Get the fixed altitude option
            std::optional<float> fixed_altitude;
            if (YAML::Node altitude_param = config["target_altitude"]) 
            {
                fixed_altitude = altitude_param.as<float>();
            }

            // Initialize controllers
            const int number_of_agents = config["robots_number"].as<int>();
            for (int agent_id =0; agent_id < number_of_agents; agent_id++)
            {
                VATController::ControllerOptions_t options;
                options.id = agent_id;

                // Get the flocking parameters
                populateVATParametersFromConfig(config,"VAT_undefined_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Undefined]);
                populateVATParametersFromConfig(config,"VAT_mission_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Mission]);
                populateVATParametersFromConfig(config,"VAT_potential_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Potential]);
                populateVATParametersFromConfig(config,"VAT_idle_flocking_parameters", options.VAT_params[agent_util::AgentRoleType::Idle]);

                // Get the altitude parameter
                options.desired_fixed_altitude = fixed_altitude;

                // Look for a secondary objective for the agent
                if (secondary_objectives.find(agent_id) != secondary_objectives.end())
                {
                    options.secondary_objective = secondary_objectives[agent_id];
                    RCLCPP_INFO_STREAM(this->get_logger(), "Goal of agent " << agent_id << " : " << secondary_objectives[agent_id].transpose() << std::endl);  
                }

                controllers_.push_back(std::make_unique<VATController>(options));
            }
        }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VATControllerNode>());
    rclcpp::shutdown();
    return 0;
}
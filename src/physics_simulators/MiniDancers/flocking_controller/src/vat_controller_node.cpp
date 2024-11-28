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
#include "dancers_msgs/srv/get_agent_velocities.hpp"

using namespace std::placeholders;

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

            // Initialize the controllers
            agentControllerInitialization(config);

            // Create the service
            service_ = this->create_service<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities", 
                                                                                  std::bind(&VATControllerNode::commandCallback,this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "Flocking controller's service initialized.");
        }

    private:

        rclcpp::Service<dancers_msgs::srv::GetAgentVelocities>::SharedPtr service_;

        /**
         * @brief Map containing the controllers for each agent. The key is the agent's id. 
         */
        std::vector<std::unique_ptr<VATController>> controllers_;

        /**
         * @brief Obstacle list to avoid. Used in the shillTerm().
         */
        std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles_;

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
                std::vector<std::shared_ptr<agent_util::AgentState_t>> agent_states;

                // Create a vector list of all the agent states.
                for(const auto& agent_struct: request->agent_structs)
                {
                    agent_util::AgentState_t self_agent_state = agent_util::create_agent_state_from_ROS_message(agent_struct);
                    agent_states.push_back(std::make_shared<agent_util::AgentState_t>(std::move(self_agent_state)));
                }

                
                for(const auto& agent_struct: request->agent_structs)
                {
                    // Create the list of neighbors
                    std::vector<std::shared_ptr<const agent_util::AgentState_t>> neighbors;

                    // Create an array of neighbors
                    for(auto neighbor_id: agent_struct.neighbor_array.neighbors)
                    {
                        if (neighbor_id != agent_struct.agent_id)
                        {
                            neighbors.push_back(agent_states[neighbor_id]);
                        }
                    }

                    dancers_msgs::msg::VelocityHeading agent_command = controllers_[agent_struct.agent_id]->getVelocityHeading(*agent_states[agent_struct.agent_id], neighbors, *obstacles_);
                    response->velocity_headings.velocity_heading_array.push_back(std::move(agent_command));
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
         * @brief Initialize the controller of each agent based on a configuration file.
         * @param config YAML configuration stucture containing the obstacles.
         */
        void agentControllerInitialization(YAML::Node& config)
        {
            // Get secondary objectives from config files
            std::map<int, Eigen::Vector3d> secondary_objectives;
            for (auto goal : config["secondary_objectives"])
            {
                secondary_objectives.insert({goal.first.as<int>(), Eigen::Vector3d(goal.second[0].as<double>(), goal.second[1].as<double>(), goal.second[2].as<double>())});
            }

            // Initialize controllers
            const int number_of_agents = config["robots_number"].as<int>();
            for (int agent_id =0; agent_id < number_of_agents; agent_id++)
            {
                // Look for a secondary objective for the agent
                if (secondary_objectives.find(agent_id) != secondary_objectives.end())
                {
                    std::optional<Eigen::Vector3d> secondary_objective_opt = secondary_objectives[agent_id];
                    controllers_.push_back(std::make_unique<VATController>(agent_id, secondary_objective_opt));
                    
                    RCLCPP_INFO_STREAM(this->get_logger(), "Goal of agent " << agent_id << " : " << secondary_objectives[agent_id].transpose() << std::endl);
                }
                else
                {
                    controllers_.push_back(std::make_unique<VATController>(agent_id));
                }
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
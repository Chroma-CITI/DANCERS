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

        VATController::VAT_params_t default_vat_params_ = {
            .v_flock = 1.5,
            .v_max = 1.0,
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
                vat_params.a_frict = config[vat_params_config_list_name]["a_frict"].as<double>();
                vat_params.p_frict = config[vat_params_config_list_name]["p_frict"].as<double>();
                vat_params.r_0_frict = config[vat_params_config_list_name]["r_0_frict"].as<double>();
                vat_params.C_frict = config[vat_params_config_list_name]["C_frict"].as<double>();
                vat_params.v_frict = config[vat_params_config_list_name]["v_frict"].as<double>();
                vat_params.p_att = config[vat_params_config_list_name]["p_att"].as<double>();
                vat_params.r_0_att = config[vat_params_config_list_name]["r_0_att"].as<double>();
                vat_params.p_rep = config[vat_params_config_list_name]["p_rep"].as<double>();
                vat_params.r_0_rep = config[vat_params_config_list_name]["r_0_rep"].as<double>();
                vat_params.a_shill = config[vat_params_config_list_name]["a_shill"].as<double>();
                vat_params.p_shill = config[vat_params_config_list_name]["p_shill"].as<double>();
                vat_params.r_0_shill = config[vat_params_config_list_name]["r_0_shill"].as<double>();
                vat_params.v_shill = config[vat_params_config_list_name]["v_shill"].as<double>();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                RCLCPP_WARN_STREAM(this->get_logger(), "Failed to read at least one VAT flocking parameter of the list " << vat_params_config_list_name <<", using ALL default VAT params.");
                vat_params = default_vat_params_;
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
                populateVATParametersFromConfig(config,"VAT_iddle_flocking_parameters", options.VAT_params_iddle);
                populateVATParametersFromConfig(config,"VAT_mission_flocking_parameters", options.VAT_params_mission);

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
#include <rclcpp/rclcpp.hpp>

#include <yaml_util.hpp>

// Custom ROS2 messages
#include <dancers_msgs/srv/get_agent_velocities.hpp>

// Custom util libs
#include <target_util.hpp>
#include <agent.hpp>
#include <cuboid_obstacle_util.hpp>

#include "vat_controller.hpp"

using namespace std::placeholders;

class VATControllerNode : public rclcpp::Node
{
public:
    VATControllerNode() : Node("vat_controller"), obstacles_(std::make_shared<std::vector<cuboid::obstacle_t>>())
    {
        // Declare the config_file ROS2 parameter
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        
        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s\nA config file must be given in the launch file.", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Parse the config file
        this->config_ = YAML::LoadFile(config_file_path);

        // If this controller is supposed to publish the agents' velocity commands, initialize a ROS2 publisher
        this->should_controller_publish_cmd_ = getYamlValue<bool>(this->config_, "should_controller_publish_commands");
        if (this->should_controller_publish_cmd_)
        {
            cmd_publisher_ = this->create_publisher<dancers_msgs::msg::VelocityHeadingArray>("velocity_cmd", 10);
        }

        // Create the object that holds the flocking configuration defined in the configuration file

        // Get the fixed altitude option
        if (YAML::Node altitude_param = this->config_["fixed_altitude"])
        {
            this->flocking_options.desired_fixed_altitude = altitude_param.as<float>();
        }

        // Get the min altitude option
        if (YAML::Node min_altitude_param = this->config_["min_altitude"])
        {
            this->flocking_options.desired_min_altitude = min_altitude_param.as<float>();
        }

        // Get the mandatory parameters for the VAT flocking
        this->flocking_options.VAT_params = VATController::VAT_params_t{
            getYamlValue<float>(this->config_, "v_max"),
            getYamlValue<float>(this->config_, "p_tar"),
            getYamlValue<float>(this->config_, "r_0_tar"),
            getYamlValue<float>(this->config_, "a_frict"),
            getYamlValue<float>(this->config_, "p_frict"),
            getYamlValue<float>(this->config_, "r_0_frict"),
            getYamlValue<float>(this->config_, "C_frict"),
            getYamlValue<float>(this->config_, "v_frict"),
            getYamlValue<float>(this->config_, "p_att"),
            getYamlValue<float>(this->config_, "r_0_att"),
            getYamlValue<float>(this->config_, "p_rep"),
            getYamlValue<float>(this->config_, "r_0_rep"),
            getYamlValue<float>(this->config_, "a_shill"),
            getYamlValue<float>(this->config_, "p_shill"),
            getYamlValue<float>(this->config_, "r_0_shill"),
            getYamlValue<float>(this->config_, "v_shill"),
            getYamlValue<float>(this->config_, "p_los"),
            getYamlValue<float>(this->config_, "r_los_obst_inflation"),
            getYamlValue<float>(this->config_, "r_obstacle_perception")
        };

        // Get the obstacles from the configuration file
        if (YAML::Node obstacles = this->config_["obstacles"])
        {
            for (const auto& obstacle : obstacles)
            {
                cuboid::obstacle_t obs{};
                obs.id = obstacle["id"].as<int>();
                obs.center = Eigen::Vector3d(
                    obstacle["x"].as<double>(), 
                    obstacle["y"].as<double>(), 
                    obstacle["z"].as<double>());
                obs.size_x = obstacle["size_x"].as<double>();
                obs.size_y = obstacle["size_y"].as<double>();
                obs.size_z = obstacle["size_z"].as<double>();

                this->obstacles_->push_back(std::move(obs));
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No obstacle defined in config file.");
        }

        // Get the targets from the configuration file
        if (YAML::Node targets_node = this->config_["targets"])
        {
            for (const auto& target_node : targets_node)
            {
                target_t target;
                target.id = getYamlValue<uint32_t>(target_node, "id");
                target.position = Eigen::Vector3d(
                    getYamlValue<float>(target_node, "x"),
                    getYamlValue<float>(target_node, "y"),
                    getYamlValue<float>(target_node, "z")
                );
                target.radius = getYamlValue<float>(target_node, "radius");
                target.global = getYamlValue<bool>(target_node, "global");
                target.assigned_agents = getYamlValue<std::vector<uint32_t>>(target_node, "assigned_agents");
                target.is_sink = getYamlValue<bool>(target_node, "is_sink");

                this->targets_.push_back(target);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No target defined in config file.");
        }
        
        // Create the service
        service_ = this->create_service<dancers_msgs::srv::GetAgentVelocities>("get_agents_velocities",
            std::bind(&VATControllerNode::serviceClbk, this, _1, _2));
            
        RCLCPP_INFO(this->get_logger(), "VATControllerNode initialized.");
    }
private:
    /**
     * @brief Obstacle list to avoid. Used in the shillTerm().
     */
    std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles_;

    /**
     * @brief The YAML object holding the configuration of the simulation.
     */
    YAML::Node config_;

    /**
     * @brief Flocking options used to initialize each VAT controller.
     */
    VATController::VATControllerOptions_t flocking_options;

    /**
     * @brief List of targets.
     */
    std::vector<target_t> targets_;

    /**
     * @brief Service server used to return agent_velocities when called.
     */
    rclcpp::Service<dancers_msgs::srv::GetAgentVelocities>::SharedPtr service_;

    /**
     * @brief Map containing the controllers for each agent. The key is the agent's id.
     */
    std::map<uint32_t, std::unique_ptr<VATController>> controllers_;

    /**
     * @brief Flag that indicates if the controller should publish its service response on a topic.
     * Mainly used for debugging.
     */
    bool should_controller_publish_cmd_ = false;

    /**
     * @brief Publisher that publishes the response of the its service computing the velocities command.
     */
    rclcpp::Publisher<dancers_msgs::msg::VelocityHeadingArray>::SharedPtr cmd_publisher_;

    /**
     * @brief Callback that computes all the desired velocities of a group of agents given their state, neighbors states and obstacles.
     */
    void serviceClbk(const std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Request> request,
                         std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response);

    /**
     * @brief Initialize a VAT controller for the given agent if it does not already exist.
     */
    void initializeController(const dancers_msgs::msg::AgentStruct& msg);
    
};


void VATControllerNode::serviceClbk(const std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Request> request,
                         std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response)
{
    RCLCPP_DEBUG(this->get_logger(), "flocking_controller received a request to compute %ld agent velocities.", request->agent_structs.size());

    if (request->agent_structs.size() != controllers_.size())
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "The flocking_controller received a request for " << request->agent_structs.size() << " agents, which is different from the number of existing controllers (" << controllers_.size() << ").");
    }

    for (const auto &agent_struct : request->agent_structs)
    {
        std::shared_ptr<const Agent> self_agent = std::make_shared<const Agent>(AgentFromRosMsg(agent_struct));
        if (this->controllers_.find(agent_struct.agent_id) != this->controllers_.end())
        {
            dancers_msgs::msg::VelocityHeading agent_command = this->controllers_[agent_struct.agent_id]->getVelocityHeading(self_agent, *obstacles_);
            response->velocity_headings.velocity_heading_array.push_back(std::move(agent_command));
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Adding flocking command to response for agent " << agent_struct.agent_id << " : " << 
                agent_command.velocity.x << "," <<
                agent_command.velocity.y << "," <<
                agent_command.velocity.z);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "The flocking controller node received a request for agent %d that is not in the list of controllers. Creating a new controller for it.", agent_struct.agent_id);
            this->initializeController(agent_struct);
            dancers_msgs::msg::VelocityHeading agent_command = this->controllers_[agent_struct.agent_id]->getVelocityHeading(self_agent, *obstacles_);
            response->velocity_headings.velocity_heading_array.push_back(std::move(agent_command));
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Adding flocking command to response for agent " << agent_struct.agent_id << " : " << 
                agent_command.velocity.x << "," <<
                agent_command.velocity.y << "," <<
                agent_command.velocity.z);
        }
    }
    if (this->should_controller_publish_cmd_)
    {
        cmd_publisher_->publish(response->velocity_headings);
    }
}

void VATControllerNode::initializeController(const dancers_msgs::msg::AgentStruct& msg)
{
    uint32_t agent_id = msg.agent_id;
    if (this->controllers_.find(agent_id) != this->controllers_.end())
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "VAT controller for agent " << agent_id << " already initialized.");
    }
    else
    {
        // Make a copy of the generic parameters
        auto options = this->flocking_options;
    
        // customize with the agent's ID
        options.id = agent_id;
    
        // add the agent's target, if needed
        for (auto& target : this->targets_)
        {
            if (target.global)
            {
                options.target_position = target.position;
                break;
            }
            else if (std::find(target.assigned_agents.begin(), target.assigned_agents.end(), agent_id) != target.assigned_agents.end())
            {
                options.target_position = target.position;
                break;
            }
        }
        
        this->controllers_[agent_id] = std::make_unique<VATController>(options);
        RCLCPP_INFO_STREAM(this->get_logger(), "Initialized VAT controller for agent " << agent_id);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VATControllerNode>());
    rclcpp::shutdown();
    return 0;
}
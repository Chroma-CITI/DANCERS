
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <dancers_msgs/srv/get_agent_velocities.hpp>

using namespace std::placeholders;

class TrajectoryControllerNode : public rclcpp::Node
{
public:
    TrajectoryControllerNode() : Node("trajectory_controller_node")
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

        std::string trajectory_shape_str = config["trajectory_shape"].as<std::string>();

        if (trajectory_shape_str == "CIRCULAR")
        {
            this->trajectory_shape = TrajectoryShape::CIRCULAR;
        }
        else if (trajectory_shape_str == "INFINITY_SYMBOL")
        {
            this->trajectory_shape = TrajectoryShape::INFINITY_SYMBOL;
        }
        else if (trajectory_shape_str == "SQUARE")
        {
            this->trajectory_shape = TrajectoryShape::SQUARE;
        }
        else if (trajectory_shape_str == "TRIANGLE")
        {
            this->trajectory_shape = TrajectoryShape::TRIANGLE;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "The trajectory shape is not valid.");
            exit(EXIT_FAILURE);
        }

        // Create the service
        service_ = this->create_service<dancers_msgs::srv::GetAgentVelocities>("get_agent_velocities",
                                                                               std::bind(&TrajectoryControllerNode::commandCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Flocking controller's service initialized.");
    }

private:
    enum class TrajectoryShape
    {
        CIRCULAR=0,
        INFINITY_SYMBOL,
        SQUARE,
        TRIANGLE
    };

    TrajectoryShape trajectory_shape;

    /**
     * @brief Service server used to return agent_velocities when called.
     */
    rclcpp::Service<dancers_msgs::srv::GetAgentVelocities>::SharedPtr service_;

    /**
     * @brief Callback that computes all the desired velocities of a group of agent given their self states, neighbor states and obstacles.
     */
    void commandCallback(const std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Request> request,
                         std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response)
    {
        RCLCPP_DEBUG(this->get_logger(), "trajectory_controller received a request to compute agent velocities.");

        dancers_msgs::msg::VelocityHeading command;

        double t = this->get_clock()->now().seconds();

        // Circular
        double w = 0.1;
        double r = 10.0;

        // Infinity Symbol
        double a = 5.0;

        if (this->trajectory_shape == TrajectoryShape::CIRCULAR)
        {
            Eigen::Vector3d cmd;
            cmd[0] = cos(w * t);
            cmd[1] = sin(w * t);
            cmd[2] = 0.0;
            cmd.normalize();
            cmd *= r * w;

            command.heading = 0.0;
            command.velocity.x = cmd[0];
            command.velocity.y = cmd[1];
            command.velocity.z = cmd[2];
        }
        else if (this->trajectory_shape == TrajectoryShape::INFINITY_SYMBOL)
        {
            command.heading = 0.0;
            command.velocity.x = -a * sin(w * t);
            command.velocity.y = a * (cos(w * t) * cos(w * t) - sin(w * t) * sin(w * t));
            command.velocity.z = 0.0;
        }

        for (std::size_t i = 0; i < request->agent_structs.size(); i++)
        {
            response->velocity_headings.velocity_heading_array.push_back(command);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryControllerNode>());
    rclcpp::shutdown();
    return 0;
}
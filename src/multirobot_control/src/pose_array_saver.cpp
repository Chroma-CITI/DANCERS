#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

using std::placeholders::_1;

class PoseArraySaver : public rclcpp::Node
{
  public:
    PoseArraySaver()
    : Node("pose_saver")
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

            // ========================= ROBOT POSE SAVING =========================

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
            while (this->out_file_name.empty())
            {
                temp_path = ros_ws_path + "/data/" + experience_name + "/robot_poses_" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    this->out_file_name = temp_path;
                }
            }

            std::ofstream out_file;
            // initialize the output file with headers
            out_file.open(this->out_file_name.c_str(), std::ios::out);
            out_file << "timestamp,";
            for (int i=0 ; i < config["robots_number"].as<int>() ; i++)
            {
                out_file << "x_" << i << ",";
                out_file << "y_" << i << ",";
                out_file << "z_" << i << ",";
            }
            out_file << std::endl;
            out_file.close();

        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        this->pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/agent_poses", 
            10, 
            std::bind(&PoseArraySaver::pose_array_callback, this, _1));

    }

  private:
    void pose_array_callback(const geometry_msgs::msg::PoseArray & msg) const;
    std::string out_file_name;
    std::string ros_ws_path;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;

    std::string m_output_file;
    
};

void PoseArraySaver::pose_array_callback(const geometry_msgs::msg::PoseArray & msg) const
{
    std::ofstream out_file;

    out_file.open(this->out_file_name.c_str(), std::ios_base::app);

    double ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000.0;
    out_file << ts << ",";
    for (auto pose : msg.poses)
    {
        for (int i=0 ; i < msg.poses.size() ; i++)
        {
            out_file << pose.position.x << ",";
            out_file << pose.position.y << ",";
            out_file << pose.position.z << ",";
        }
    }
    out_file << std::endl;
    out_file.close();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseArraySaver>());
  rclcpp::shutdown();
  return 0;
}
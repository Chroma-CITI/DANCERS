#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "dancers_msgs/msg/agent_struct_array.hpp"

#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

using std::placeholders::_1;

class AgentStructSaver : public rclcpp::Node
{
  public:
    AgentStructSaver()
    : Node("agent_struct_saver")
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
            while (this->out_folder_name.empty())
            {
                temp_path = ros_ws_path + "/data/" + experience_name + "/agent_structs_" + std::to_string(i);
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    this->out_folder_name = temp_path;
                }
            }

            // create directory
            boost::filesystem::create_directories(this->out_folder_name);

            for (int i=0; i < config["robots_number"].as<int>(); i++)
            {
                std::string agent_file = this->out_folder_name + "/agent_"+std::to_string(i) + ".csv";
                this->agent_output_files.push_back(agent_file);
                std::ofstream out_file;
                // initialize the output file with headers
                out_file.open(agent_file.c_str(), std::ios::out);
                out_file << "timestamp,agent_role,agent_id,x,y,z,vx,vy,vz,heading,";
                for (int j=0; j < config["robots_number"].as<int>()-1; j++)
                {
                    out_file << "neighId" << j << ",neighPathloss" << j << ",";
                }
                out_file << std::endl;
                out_file.close();
            }

        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        this->agent_struct_array_sub_ = this->create_subscription<dancers_msgs::msg::AgentStructArray>(
            "/agent_structs", 
            10, 
            std::bind(&AgentStructSaver::agent_struct_array_callback, this, _1));

    }

  private:
    void agent_struct_array_callback(const dancers_msgs::msg::AgentStructArray & msg);
    std::string out_folder_name;
    std::string ros_ws_path;
    std::vector<std::string> agent_output_files;

    rclcpp::Subscription<dancers_msgs::msg::AgentStructArray>::SharedPtr agent_struct_array_sub_;

    std::string m_output_file;
    
};

void AgentStructSaver::agent_struct_array_callback(const dancers_msgs::msg::AgentStructArray & msg)
{

    for (auto agent_struct : msg.structs)
    {
        std::ofstream out_file;
        out_file.open(this->agent_output_files[agent_struct.agent_id].c_str(), std::ios_base::app);
        
        std::string role;
        switch (agent_struct.agent_role)
        {
        case dancers_msgs::msg::AgentStruct::AGENT_ROLE_MISSION:
            role = "MISSION";
            break;
        
        case dancers_msgs::msg::AgentStruct::AGENT_ROLE_POTENTIAL:
            role = "POTENTIAL";
            break;

        case dancers_msgs::msg::AgentStruct::AGENT_ROLE_IDLE:
            role = "IDLE";
            break;

        case dancers_msgs::msg::AgentStruct::AGENT_ROLE_UNDEFINED:
            role = "UNDEFINED";
            break;

        default:
            role = "UNKNOWN";
            break;
        }

        // get current ROS2 time
        rclcpp::Time ts = this->get_clock()->now();
        out_file << ts.seconds() << ",";
        out_file << role << "," << agent_struct.agent_id << ",";
        out_file << agent_struct.state.position.x << "," << agent_struct.state.position.y << "," << agent_struct.state.position.z << ",";
        out_file << agent_struct.state.velocity_heading.velocity.x << "," << agent_struct.state.velocity_heading.velocity.y << "," << agent_struct.state.velocity_heading.velocity.z << ",";
        out_file << agent_struct.state.velocity_heading.heading << ",";

        dancers_msgs::msg::NeighborArray neighbor_array = agent_struct.neighbor_array;
        for (auto neighbor : neighbor_array.neighbors)
        {
            out_file << neighbor.agent_id << "," << neighbor.link_quality << ",";
        }

        out_file << std::endl;
        out_file.close();

    }

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgentStructSaver>());
  rclcpp::shutdown();
  return 0;
}
#include <iostream>
#include <fstream>
#include <optional>
#include <string>

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include "gz/sim/ServerConfig.hh"
#include <gz/sim/components.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>


#include <protobuf_msgs/physics_update.pb.h>
#include <protobuf_msgs/channel_data.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <ros_net_sim_interfaces/msg/pathloss_pair.hpp>
#include <ros_net_sim_interfaces/msg/pathloss_pair_array.hpp>

#include <yaml-cpp/yaml.h>

static std::string m_output_file;

class RoboticsSimulator : public rclcpp::Node
{
    public:
        RoboticsSimulator() : Node("robotics_simulator")
        {
            // Declare two parameters for this ros2 node
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "Path to the YAML configuration file.";
            this->declare_parameter("config_file", "", param_desc);
            this->declare_parameter("verbose", false);

            // Fetch the parameter path to the config file using ros2 parameter
            std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

            // Verify existence of the config file, abort if not found
            if(access(config_file_path.c_str(), F_OK) != 0){
               RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s\nA config file must be given in the launch file.", config_file_path.c_str());
               exit(EXIT_FAILURE);
            }

            // Parse the config file
            YAML::Node config = YAML::LoadFile(config_file_path);

            // Store the names of the robots in gazebo in a vector
            std::vector<std::string> gazebo_models;
            for(int i=1 ; i <= config["robots_number"].as<int>() ; i++){
                std::string robot_name = config["robots_model"].as<std::string>()+"_"+std::to_string(i); //e.g. x500_1, x500_2, etc.
                gazebo_models.push_back(robot_name); 
                RCLCPP_INFO(this->get_logger(), "Tracked robot: %s", robot_name.c_str());
            }

// vvvvvvvvvvvvvvvvvvvvvvvv DATA SAVING vvvvvvvvvvvvvvvvvvvvvvvv

            // Create a folder based on the experience name, if not existant already
            std::string experience_name = config["experience_name"].as<std::string>();
            if(boost::filesystem::create_directories("./data/"+experience_name)){
                RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", experience_name.c_str());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Using existing data folder for this experiment");
            }

            // Define the output file name, based on the existing files in the experience folder (incremental)
            std::string temp_path;
            int j = 1;
            while(m_output_file.empty()){
                temp_path = "./data/"+experience_name+"/robot_sim_data"+std::to_string(j)+".csv";
                if(boost::filesystem::exists(temp_path)){
                    j++;
                } else {
                    m_output_file = temp_path;
                }
            }

            // initialize the output file with headers
            std::ofstream f;
            f.open(m_output_file.c_str(), std::ios::out);
            f << "Time[us]"
            << std::endl;
            f.close();

// ^^^^^^^^^^^^^^^^^^^^^^^^ DATA SAVING ^^^^^^^^^^^^^^^^^^^^^^^^


            // Store the system values
            int sync_window = config["sync_window"].as<int>();
            int step_length = config["phy_step_size"].as<int>(); // us
            int update_rate = config["update_rate"].as<int>();

            if(sync_window%step_length != 0){
                RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
                exit(EXIT_FAILURE);
            }
            uint64_t steps_per_window = sync_window/step_length;
            
            /******** [Step 1] Set-up and start the Gazebo simulation server ********/
            
            // Verbosity level for Gazebo - defaults to 1 if unset
            gz::common::Console::SetVerbosity(4);

            // Object to pass custom configuration to the server
            gz::sim::ServerConfig serverConfig;

            // Populate with some configuration, for example, the SDF file to load
            // modify_sdf_world(config["world_sdf_path"].as<std::string>(), step_length, update_rate);
            serverConfig.SetSdfFile(config["world_sdf_path"].as<std::string>());
            serverConfig.SetUpdateRate(update_rate); // in Hz
            // serverConfig.SetInitialSimTime(std::time(0)); // in seconds

            // Instantiate server
            gz::sim::Server server(serverConfig);

            RCLCPP_INFO(this->get_logger(), "Gazebo server started.");


            /******** [Step 2] Interact with Gazebo Server to modify the world and prepare subscribers ********/

            // Object enabling pub/sub and service calls
            gz::transport::Node node;

            bool executed{false};
            bool result{false};
            unsigned int timeout{5000};

            gz::msgs::StringMsg_V resp;
            
            // Request the /gazebo/worlds service to get the world's name and make sure it is correctly created.
            std::string service{"/gazebo/worlds"};
            executed = node.Request(service, timeout, resp, result);
            this->check_service_results(service, executed, result);
            
            std::string world_name = resp.data(0);
            RCLCPP_INFO(this->get_logger(), "World found with name : %s", world_name.c_str());

            // Call the /world/<world_name>/Create service to create the buildings
            resp.Clear();
            executed = false; 
            result = false;
            service = "/world/"+world_name+"/create";
            gz::msgs::EntityFactory req;
            gz::msgs::Boolean respo;

            for(auto building : config["buildings"]){
                std::string name = building["name"].as<std::string>();
                double x = building["x"].as<double>();
                double y = building["y"].as<double>();
                double z = building["height"].as<double>()/2;
                double size_x = building["size_x"].as<double>();
                double size_y = building["size_y"].as<double>();
                double height = building["height"].as<double>();
                std::string buildingSdf = R"(
                    <?xml version="1.0" ?>
                    <sdf version='1.7'>
                        <model name=')" + name + R"('>
                            <static>true</static>
                            <link name='link'>
                                <pose>)"+std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(z)+R"( 0 0 0</pose>
                                <visual name='visual'>
                                    <geometry>
                                        <box>
                                            <size>)"+std::to_string(size_x)+" "+std::to_string(size_y)+" "+std::to_string(height)+R"(</size>
                                        </box>
                                    </geometry>
                                    <material>
                                        <ambient>0.2 0.2 0.2 0.3</ambient>
                                        <diffuse>0.4 0.4 0.8 1</diffuse>
                                        <specular>0.0 0.0 0.0 1</specular>
                                    </material>
                                </visual>
                                <collision name='collision'>
                                    <geometry>
                                        <box>
                                            <size>)"+std::to_string(size_x)+" "+std::to_string(size_y)+" "+std::to_string(height)+R"(</size>
                                        </box>
                                    </geometry>
                                </collision>
                            </link>
                        </model>
                    </sdf>
                )";

                req.set_sdf(buildingSdf);

                if(this->get_parameter("verbose").get_parameter_value().get<bool>()){
                    RCLCPP_DEBUG(this->get_logger(), "Request creation of entity : \n%s", req.DebugString().c_str());
                }

                executed = node.Request(service, req, timeout, respo, result);
                this->check_service_results(service, executed, result);
                if(respo.data()){std::cout << "Created building " << name << std::endl;}
            }

            // Call the /world/<world_name>/set_physics service to set the Physics of Gazebo simulation
            executed = false; 
            result = false;
            service = "/world/"+world_name+"/set_physics";
            gz::msgs::Physics request;
            gz::msgs::Boolean res;
            request.set_max_step_size((double)step_length*0.000001); // step_length in us, converted to seconds
            executed = node.Request(service, request, timeout, res, result);
            this->check_service_results(service, executed, result);

            // Map holding the robots' names and Pose information
            std::map<std::string, gz::msgs::Pose> robot_poses;
            std::mutex rob_pos_mutex;
            // Map holding the robots' names and Odometry information
            std::map<std::string, gz::msgs::Odometry> robot_odom;
            std::mutex rob_odom_mutex;

            // Callback to the pose/info subscriber. Its role is to fill the robot_poses map
            std::function<void(const gz::msgs::Pose_V &)> cbPoseInfo = 
            [&](const gz::msgs::Pose_V &_msg)
            {
                // We use a mutex because there is a concurrency issue with generate_channel_data() function
                std::lock_guard<std::mutex> guard(rob_pos_mutex);
                for(int i=0 ; i < _msg.pose_size() ; i++){
                    std::string entity_name = _msg.pose(i).name();
                    // filter the entities given by the topic "/world/<world_name>/pose/info" with the gazebo_names given in the config file
                    if(std::find(gazebo_models.begin(), gazebo_models.end(), entity_name) != gazebo_models.end()){
                        robot_poses[entity_name] = _msg.pose(i);
                    }
                }
            };

            // Callback to the pose/info subscriber. Its role is to fill the robot_poses map
            std::function<void(const gz::msgs::Odometry &)> cbOdometry = 
            [&](const gz::msgs::Odometry &_msg)
            {
                for(std::string robot_name : gazebo_models){
                    if(_msg.header().data(0).value(0).find(robot_name) != std::string::npos){
                        std::lock_guard<std::mutex> guard(rob_odom_mutex);
                        robot_odom[robot_name] = _msg;
                        return;
                    }
                }
            };

            std::map< std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr > publishers;
            for(auto robot_name : gazebo_models){
                if(node.Subscribe("/model/"+robot_name+"/odometry", cbOdometry)){
                    RCLCPP_INFO(this->get_logger(), "Subscribed to /model/%s/odometry", robot_name.c_str());
                }
                publishers[robot_name] = this->create_publisher<nav_msgs::msg::Odometry>(robot_name+"/odometry", 10);
            }

            // Create a subscriber to /world/<world_name>/pose/info 
            // (this topic sends the pose of all entities in the world, which is overkill for us, but more robust as it does not depend on a robot system plugin)
            if(node.Subscribe("/world/"+world_name+"/pose/info", cbPoseInfo)){
                RCLCPP_INFO(this->get_logger(), "Subscribed to /world/%s/pose/info", world_name.c_str());
            }

            std::cout << "Gazebo is ready" << std::endl; // This line triggers the spawn of the drones in the python launch file.


            /******** [Step 3] Main simulation loop ********/
            double sim_time = 0.0;

            while(true){

                sim_time += steps_per_window*step_length*0.001; // in seconds

                std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
                // Advance the server in this thread (blocking) for W iterations, pause at the end
                server.Run(true /*blocking*/, steps_per_window /*iterations*/, false /*paused*/);
                std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
                f.open(m_output_file.c_str(), std::fstream::app);
                f << wait << "\n";
                f.close();

                if(this->get_parameter("verbose").get_parameter_value().get<bool>()){
                    // 34 = Blue :)
                    RCLCPP_INFO(this->get_logger(), "\x1b[34m[%f] Advanced %i milliseconds\x1b[0m", sim_time, sync_window / 1000);
                }

                // generate the channel_data message from the pose of the robots
                rob_pos_mutex.lock();
                rob_odom_mutex.lock();
                this->publish_robots_odom(robot_odom, publishers);
                rob_pos_mutex.unlock();
                rob_odom_mutex.unlock();

            }
        }

    private:
        void publish_robots_odom(std::map<std::string, gz::msgs::Odometry> robot_poses, std::map< std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr > publishers);
        void check_service_results(std::string service, bool executed, bool result);
};

/**
 * \brief Prints the result of a call to a Gazebo Service (just for code compactness)
 * 
 * \param service The name of the service
 * \param executed If the service timed out
 * \param result If the call to the service failed
 */
void RoboticsSimulator::check_service_results(std::string service, bool executed, bool result){
    if(!executed){
        std::cerr << std::endl << "Service call to [" << service << "] timed out"
            << std::endl;
        exit(EXIT_FAILURE);
    } 
    if(!result){
        std::cerr << std::endl << "Service call to [" << service << "] failed"
            << std::endl;
        exit(EXIT_FAILURE);
    }
}

void RoboticsSimulator::publish_robots_odom(const std::map<std::string, gz::msgs::Odometry> robot_odom, std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> publishers)
{
    for(const auto& robot : robot_odom){
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "world";
        odom.child_frame_id = robot.first;
        odom.pose.pose.position.x = robot.second.pose().position().x();
        odom.pose.pose.position.y = robot.second.pose().position().y();
        odom.pose.pose.position.z = robot.second.pose().position().z();
        odom.pose.pose.orientation.w = robot.second.pose().orientation().w();
        odom.pose.pose.orientation.x = robot.second.pose().orientation().x();
        odom.pose.pose.orientation.y = robot.second.pose().orientation().y();
        odom.pose.pose.orientation.z = robot.second.pose().orientation().z();

        odom.twist.twist.linear.x = robot.second.twist().linear().x();
        odom.twist.twist.linear.y = robot.second.twist().linear().y();
        odom.twist.twist.linear.z = robot.second.twist().linear().z();
        odom.twist.twist.angular.x = robot.second.twist().angular().x();
        odom.twist.twist.angular.y = robot.second.twist().angular().y();
        odom.twist.twist.angular.z = robot.second.twist().angular().z();
        
        // Ensures the publisher exists for this robot name
        if(publishers.find(robot.first) != publishers.end()){
            publishers[robot.first]->publish(odom);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not find publisher for robot %s", robot.first.c_str());
        }
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboticsSimulator>());
    rclcpp::shutdown();
    return 0;
}
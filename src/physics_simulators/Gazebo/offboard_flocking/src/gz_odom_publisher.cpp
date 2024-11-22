#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>


/**
 * \brief Prints the result of a call to a Gazebo Service (just for code compactness)
 * 
 * \param service The name of the service
 * \param executed If the service timed out
 * \param result If the call to the service failed
 */
void check_service_results(std::string service, bool executed, bool result){
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

class GzOdomPublisher : public rclcpp::Node
{
    public:
        // Constructor
        GzOdomPublisher() : rclcpp::Node("GzOdomPublisher"){

            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "Number of robots.";
            this->declare_parameter("robots_number", 1, param_desc);

            param_desc.description = "Model of the robot.";
            this->declare_parameter("robots_model", "x500", param_desc);

            this->robots_number = this->get_parameter("robots_number").as_int();
            this->robots_model = this->get_parameter("robots_model").as_string();

            // Store the names of the robots in gazebo in a vector
            for(int i=1 ; i <= this->robots_number ; i++){
                std::string robot_name = this->robots_model+"_"+std::to_string(i); //e.g. x500_1, x500_2, etc.
                this->gazebo_models.push_back(robot_name); 
                RCLCPP_INFO(this->get_logger(), "Tracked robot: %s", robot_name.c_str());
            }

            // ------------------- Interaction with Gazebo -------------------

            bool executed{false};
            bool result{false};
            unsigned int timeout{5000};

            gz::msgs::StringMsg_V resp;
            
            // Request the /gazebo/worlds service to get the world's name and make sure it is correctly created.
            std::string service{"/gazebo/worlds"};
            executed = this->node.Request(service, timeout, resp, result);
            check_service_results(service, executed, result);
            
            this->world_name = resp.data(0);
            RCLCPP_INFO(this->get_logger(), "World found with name : %s", this->world_name.c_str());

                        // Prepare the QoS that we will assign to the publishers / subscribers  
            auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
            
            for(int i=0; i < this->robots_number; i++)
            {
                this->odom_publishers[this->gazebo_models[i]] = this->create_publisher<nav_msgs::msg::Odometry>(this->gazebo_models[i]+"/odometry", 10);
            }

            // // Callback to the /odometry subscriber.
            // std::function<void(const gz::msgs::Odometry &)> cbOdometry = 
            // [&](const gz::msgs::Odometry &_msg)
            // {
            //     // Copy the Gazebo odom to the ROS2 odom
            //     nav_msgs::msg::Odometry odom;
            //     odom.header.stamp = this->now(); // This should be the Gazebo time if the use_sim_time ROS parameter is True
            //     odom.header.frame_id = "world";
            //     odom.child_frame_id = this->robot_name;
            //     odom.pose.pose.position.x = _msg.pose().position().x();
            //     odom.pose.pose.position.y = _msg.pose().position().y();
            //     odom.pose.pose.position.z = _msg.pose().position().z();
            //     odom.pose.pose.orientation.x = _msg.pose().orientation().x();
            //     odom.pose.pose.orientation.w = _msg.pose().orientation().w();
            //     odom.pose.pose.orientation.y = _msg.pose().orientation().y();
            //     odom.pose.pose.orientation.z = _msg.pose().orientation().z();

            //     odom.twist.twist.linear.x = _msg.twist().linear().x();
            //     odom.twist.twist.linear.y = _msg.twist().linear().y();
            //     odom.twist.twist.linear.z = _msg.twist().linear().z();
            //     odom.twist.twist.angular.x = _msg.twist().angular().x();
            //     odom.twist.twist.angular.y = _msg.twist().angular().y();
            //     odom.twist.twist.angular.z = _msg.twist().angular().z();

            //     // Publish the ROS2 odom
            //     this->odom_publisher->publish(odom);
            // };

            // Map holding the robots' names and Pose information
            std::map<std::string, gz::msgs::Pose> robot_poses;

            // Callback to the pose/info subscriber. Its role is to fill the robot_poses map
            std::function<void(const gz::msgs::Pose_V &)> cbPoseInfo = 
            [&](const gz::msgs::Pose_V &_msg)
            {
                for(int i=0 ; i < _msg.pose_size() ; i++){
                    std::string entity_name = _msg.pose(i).name();
                    // filter the entities given by the topic "/world/<world_name>/pose/info" with the gazebo_names given in the config file
                    if(std::find(this->gazebo_models.begin(), this->gazebo_models.end(), entity_name) != this->gazebo_models.end()){
                        // robot_poses[entity_name] = _msg.pose(i);
                        // Copy the Gazebo odom to the ROS2 odom
                        nav_msgs::msg::Odometry odom;
                        odom.header.stamp = this->now(); // This should be the Gazebo time if the use_sim_time ROS parameter is True
                        odom.header.frame_id = this->world_name;
                        odom.child_frame_id = entity_name;
                        odom.pose.pose.position.x = _msg.pose(i).position().x();
                        odom.pose.pose.position.y = _msg.pose(i).position().y();
                        odom.pose.pose.position.z = _msg.pose(i).position().z();
                        odom.pose.pose.orientation.x = _msg.pose(i).orientation().x();
                        odom.pose.pose.orientation.w = _msg.pose(i).orientation().w();
                        odom.pose.pose.orientation.y = _msg.pose(i).orientation().y();
                        odom.pose.pose.orientation.z = _msg.pose(i).orientation().z();

                        // Publish the ROS2 odom
                        this->odom_publishers[entity_name]->publish(odom);
                    }
                }
            };

            // Create a subscriber to /world/<world_name>/pose/info 
            // (this topic sends the pose of all entities in the world, which is overkill for us, but more robust as it does not depend on a robot system plugin)
            if(node.Subscribe("/world/"+this->world_name+"/pose/info", cbPoseInfo)){
                RCLCPP_INFO(this->get_logger(), "Subscribed to /world/%s/pose/info", this->world_name.c_str());
            }




            // // Subscribe to the Gazebo "pose/info" topic with gazebo transport
            // if(this->node.Subscribe("/model/"+this->robot_name+"/odometry", cbOdometry)){
            //     RCLCPP_INFO(this->get_logger(), "Subscribed to /model/%s/odometry Gz topic", this->robot_name.c_str());
            // } else {
            //     RCLCPP_FATAL(this->get_logger(), "Failed to subscribe to /model/%s/odometry Gz topic", this->robot_name.c_str());
            // }

        }
    private:
        uint32_t robots_number;
        std::string robots_model;
        std::vector<std::string> gazebo_models;
        std::string world_name;

        std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers;
        gz::transport::Node node;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <Eigen/Dense>

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

void save_gz_pose(gz::msgs::Odometry gz_pose, std::string file_name){
    std::ofstream file;
    file.open(file_name, std::fstream::app);

    Eigen::Quaterniond q(gz_pose.pose().orientation().w(), gz_pose.pose().orientation().x(), gz_pose.pose().orientation().y(), gz_pose.pose().orientation().z());
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);

    file << gz_pose.header().stamp().sec()<<","<<gz_pose.header().stamp().nsec()<< "," << gz_pose.pose().position().x() << "," << gz_pose.pose().position().y() << "," << gz_pose.pose().position().z() << "," << euler_angles.x() << "," << euler_angles.y() << "," << euler_angles.z() << "," << gz_pose.twist().linear().x() << "," << gz_pose.twist().linear().y() << "," << gz_pose.twist().linear().z() << "," << gz_pose.twist().angular().x() << "," << gz_pose.twist().angular().y() << "," << gz_pose.twist().angular().z() << std::endl;
    file.close();

}

class GzOdomPublisher : public rclcpp::Node
{
    public:
        // Constructor
        GzOdomPublisher() : rclcpp::Node("GzOdomPublisher"){

            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "ID of the robot.";
            this->declare_parameter("robot_id", 1, param_desc);

            param_desc.description = "Model of the robot.";
            this->declare_parameter("robot_model", "x500", param_desc);

            param_desc.description = "Name of the file to save robot's pose.";
            this->declare_parameter("file_name", "data/default.csv", param_desc);

            this->robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint32_t>();
            if(this->robot_id <= 0){
                RCLCPP_FATAL(this->get_logger(), "Wrong robot ID ! Robot IDs must start at 1. Received: %i", robot_id);
            }

            this->robot_name = this->get_parameter("robot_model").as_string() + "_" + std::to_string(this->robot_id);

            this->file_name = this->get_parameter("file_name").as_string();

            // ------------------- Interaction with Gazebo -------------------

            bool executed{false};
            bool result{false};
            unsigned int timeout{5000};

            gz::msgs::StringMsg_V resp;
            
            // Request the /gazebo/worlds service to get the world's name and make sure it is correctly created.
            std::string service{"/gazebo/worlds"};
            executed = this->node.Request(service, timeout, resp, result);
            check_service_results(service, executed, result);

            std::ofstream file;
            file.open(file_name, std::ios::out);
            file << "sec,nsec,x,y,z,rx,ry,rz,vx,vy,vz,wx,wy,wz" << std::endl;
            file.close();
            
            std::string world_name = resp.data(0);
            RCLCPP_INFO(this->get_logger(), "World found with name : %s", world_name.c_str());
            
            // Callback to the /odometry subscriber.
            std::function<void(const gz::msgs::Odometry &)> cbOdometry = 
            [&](const gz::msgs::Odometry &_msg)
            {
                save_gz_pose(_msg, this->file_name);
            };

            // Subscribe to the Gazebo "pose/info" topic with gazebo transport
            if(this->node.Subscribe("/model/"+this->robot_name+"/odometry", cbOdometry)){
                RCLCPP_INFO(this->get_logger(), "Subscribed to /model/%s/odometry Gz topic", this->robot_name.c_str());
            } else {
                RCLCPP_FATAL(this->get_logger(), "Failed to subscribe to /model/%s/odometry Gz topic", this->robot_name.c_str());
            }

        }
    private:
        uint32_t robot_id;
        std::string robot_name;
        std::string file_name;

        gz::transport::Node node;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
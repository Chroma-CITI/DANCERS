#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"

#include <Eigen/Dense>

#include <boost/filesystem.hpp>

using std::placeholders::_1;

class Px4PoseSaver : public rclcpp::Node
{
  public:
    Px4PoseSaver()
    : Node("px4_pose_saver")
    {

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "ID of the robot.";
        this->declare_parameter("robot_id", 1, param_desc);

        param_desc.description = "Model of the robot.";
        this->declare_parameter("robot_model", "x500", param_desc);

        param_desc.description = "Name of the file to save robot's pose.";
        this->declare_parameter("file_name", "data/default.csv", param_desc);

        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort().transient_local();

        this->robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint32_t>();
        if(this->robot_id <= 0){
            RCLCPP_FATAL(this->get_logger(), "Wrong robot ID ! Robot IDs must start at 1. Received: %i", robot_id);
        }

        this->robot_name = this->get_parameter("robot_model").as_string() + "_" + std::to_string(this->robot_id);

        this->file_name = this->get_parameter("file_name").as_string();

        this->vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/px4_"+std::to_string(robot_id)+"/fmu/out/vehicle_attitude", 
            sub_qos, 
            std::bind(&Px4PoseSaver::vehicle_attitude_callback, this, _1));

        // initialize the output file with headers
        std::ofstream f;
        f.open(this->file_name.c_str(), std::ios::out);
        f << "timestamp,rx,ry,rz"
        << std::endl;
        f.close();


    }

  private:
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude & msg) const;
    uint32_t robot_id;
    std::string robot_name;
    std::string file_name;

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;

    std::string m_output_file;
    
};

void Px4PoseSaver::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude & msg) const
{
    std::ofstream file;
    file.open(this->file_name, std::fstream::app);

    Eigen::Quaterniond q(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);

    file << msg.timestamp << "," << euler_angles.x() << "," << euler_angles.y() << "," << euler_angles.z() << std::endl;
    file.close();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4PoseSaver>());
  rclcpp::shutdown();
  return 0;
}
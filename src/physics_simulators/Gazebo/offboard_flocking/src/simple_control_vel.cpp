#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


Eigen::Vector3d transform_vector_quat(Eigen::Vector3d v, Eigen::Quaterniond q){
    Eigen::Vector3d v_rot = q * v;
    return v_rot;
}

class SimpleControl : public rclcpp::Node
{
    public:
        // Constructor
        SimpleControl() : rclcpp::Node("SimpleControl"){

            // // Declare the config_file ROS2 parameter
            // auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            // param_desc.description = "Path to the YAML configuration file.";
            // this->declare_parameter("config_file", "", param_desc);

            // // Fetch the parameter path to the config file using ros2 parameter
            // std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

            // // Verify existence of the config file, abort if not found
            // if(access(config_file_path.c_str(), F_OK) != 0){
            //    RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            //    exit(EXIT_FAILURE);
            // }

            //  // Parse the config file
            // YAML::Node config = YAML::LoadFile(config_file_path);

            // Declare the robot_id ROS2 parameter
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "ID of the robot. This is the system id of each robot (instance number + 1)";
            this->declare_parameter("robot_id", 1, param_desc);

            robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint8_t>();

            if(robot_id <= 0){
                RCLCPP_FATAL(this->get_logger(), "Wrong robot ID ! Robot IDs must start at 1. Received: %i", robot_id);
            }

            // Prepare the QoS that we will assign to the publishers / subscribers  
            auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
            
            // Create subscriber 
            this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                            "/px4_"+std::to_string(robot_id)+
                            "/fmu/out/vehicle_status",
                            qos_sub, 
                            std::bind(&SimpleControl::vehicle_status_clbk, this, _1)
            );

            this->vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
                            "/px4_"+std::to_string(robot_id)+
                            "/fmu/out/vehicle_attitude",
                            qos_sub,
                            std::bind(&SimpleControl::vehicle_attitude_clbk, this, _1)
            );


            // Create publishers
            this->offboard_control_mode_publisher_ =
                    this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                        "/px4_"+std::to_string(robot_id)+
                        "/fmu/in/offboard_control_mode", qos_pub);
            this->trajectory_setpoint_publisher_ =
                    this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                        "/px4_"+std::to_string(robot_id)+
                        "/fmu/in/trajectory_setpoint", qos_pub);
            this->vehicle_command_publisher_ = 
                    this->create_publisher<px4_msgs::msg::VehicleCommand>(
                        "/px4_"+std::to_string(robot_id)+
                        "/fmu/in/vehicle_command", qos_pub);

		    this->offboard_setpoint_counter_ = 0;
            this->timer_period = 100000us; // 100 ms

            this->takeoff_speed = 1.0;
            this->radius = 10.0; // m
            this->theta = M_PI/2.0; // rad
            this->omega = 0.5; // rad.s-1

            auto cmdloop_callback = [this]() -> void {
                if(this->offboard_setpoint_counter_ == 10){
                    // Change to Offboard mode after 10 setpoints (1s)
                    this->engage_offboard_mode();

                    // Arm the vehicle
                    this->arm();

                }

                RCLCPP_DEBUG(this->get_logger(), "nav state: %d", this->nav_state);
                RCLCPP_DEBUG(this->get_logger(), "arming state: %d", this->arming_state);

                Eigen::Vector3d cmd;

                if(this->offboard_setpoint_counter_ <= 100){
                    cmd = {0.0, 0.0, -this->takeoff_speed};
                } 
                else {

                    // Circular motion
                    // cmd = {std::sin(0.05*this->offboard_setpoint_counter_)*this->radius, 
                    // std::cos(0.05*this->offboard_setpoint_counter_)*this->radius, 
                    // 0.0};

                    // // Straight line
                    // if((this->offboard_setpoint_counter_ % 400) > 200){
                    //     cmd = {2.0, 0.0, 0.0};
                    // } else {
                    //     cmd = {-2.0, 0.0, 0.0};
                    // }
                    cmd = {1.0, 0.0, 0.0};
                    // this->theta = this->theta + this->omega * (this->timer_period.count()/1000000.0);
                }

                // Publish the offboard control mode at each loop : Mandatory to keep this topic published > 2 Hz to keep the offboard mode
                this->publish_offboard_control_mode();

                // Publish the trajectory setpoint at each loop only if the vehicle is in Offboard mode
                if(this->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
                    this->publish_trajectory_setpoint(cmd, NAN);
                }
                this->offboard_setpoint_counter_++;

            };

            // start publisher timer
            timer_ = this->create_wall_timer(timer_period, cmdloop_callback);
        }

        void arm() ;
	    void disarm() ;
        void engage_offboard_mode();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::chrono::microseconds timer_period;
        uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
        uint8_t nav_state;
	    uint8_t arming_state;
        double takeoff_speed;
        double radius;
        double theta;
        double omega;
        uint8_t robot_id;
        Eigen::Quaterniond q;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;

        // publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

        // callbacks
        void vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg);
        void vehicle_attitude_clbk(const px4_msgs::msg::VehicleAttitude & msg);

        // commands
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint(Eigen::Vector3d target, float yaw);
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);

};

void SimpleControl::vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg)
{
    this->arming_state = msg.arming_state;
    this->nav_state = msg.nav_state;
    // RCLCPP_INFO(this->get_logger(), "ArmingState: %i\nNavState: %i", this->arming_state, this->nav_state);
}

void SimpleControl::vehicle_attitude_clbk(const px4_msgs::msg::VehicleAttitude & msg){
    this->q = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void SimpleControl::arm()
{
    // send the arm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, this->robot_id+1);

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void SimpleControl::disarm()
{
    // send the disarm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, this->robot_id+1);

	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Send a command to set the vehicle to Offboard mode
 */
void SimpleControl::engage_offboard_mode()
{
    // send the arm command in a VehicleCommand message. 1 is offboard, 6 is ???
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, this->robot_id+1);

    RCLCPP_INFO(this->get_logger(), "Set Offboard mode command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void SimpleControl::publish_offboard_control_mode(){
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void SimpleControl::publish_trajectory_setpoint(Eigen::Vector3d target, float yaw){
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);

    msg.position[0] = NAN;
    msg.position[1] = NAN;
    msg.position[2] = NAN; // The Z axis is toward the ground

    msg.velocity[0] = target[0];
    msg.velocity[1] = target[1];
    msg.velocity[2] = target[2];

    msg.acceleration[0] = NAN;
    msg.acceleration[1] = NAN;
    msg.acceleration[2] = NAN;

    msg.jerk[0] = NAN;
    msg.jerk[1] = NAN;
    msg.jerk[2] = NAN;

    msg.yaw = yaw;

    // RCLCPP_INFO(this->get_logger(), "My position: %f, %f, %f", this->robots_data[this->robot_id-1].position[0], this->robots_data[this->robot_id-1].position[1], this->robots_data[this->robot_id-1].position[2]);

    RCLCPP_DEBUG(this->get_logger(), "Sending trajectory setpoint to UAV %i: at position %f, %f, %f", this->robot_id, msg.position[0], msg.position[1], msg.position[2]);
    // RCLCPP_INFO(this->get_logger(), "Sending trajectory setpoint to UAV %i: at velocity %f, %f, %f", this->robot_id, msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    // RCLCPP_INFO(this->get_logger(), "Sending trajectory setpoint to UAV %i: at acceleration %f, %f, %f", this->robot_id, msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]);

    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void SimpleControl::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t system_id)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = system_id;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	this->vehicle_command_publisher_->publish(msg);
}



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleControl>());
    rclcpp::shutdown();
    return 0;
}
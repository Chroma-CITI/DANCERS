#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlTest : public rclcpp::Node
{
    public:
        // Constructor
        ControlTest() : rclcpp::Node("ControlTest"){

            // Declare one parameters for this ros2 node
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "ID of the robot. This is the system id of each robot (instance number + 1)";
            this->declare_parameter("robot_id", 1, param_desc);

            robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint8_t>();
            node_namespace = this->get_namespace();

            // std::cout << "node_namespace: " << node_namespace << std::endl;

            // RCLCPP_INFO(this->get_logger(), node_namespace.c_str(),"/fmu/out/vehicle_status");

            // Prepare the QoS that we will assign to the publishers / subscribers  
            auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
            
            // Create subscriber 
            this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                            node_namespace+"/fmu/out/vehicle_status",
                            qos_sub, 
                            std::bind(&ControlTest::vehicle_status_clbk, this, _1)
            );

            // Create publishers
            this->offboard_control_mode_publisher_ =
                    this->create_publisher<px4_msgs::msg::OffboardControlMode>(node_namespace+"/fmu/in/offboard_control_mode", qos_pub);
            this->trajectory_setpoint_publisher_ =
                    this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(node_namespace+"/fmu/in/trajectory_setpoint", qos_pub);
            this->vehicle_command_publisher_ = 
                    this->create_publisher<px4_msgs::msg::VehicleCommand>(node_namespace+"/fmu/in/vehicle_command", qos_pub);

		    this->offboard_setpoint_counter_ = 0;
            this->radius = 10.0; // m
            this->theta = 0.0; // rad
            this->omega = 0.5; // rad.s-1
            this->timer_period = 100ms;
            this->ax = 1.0;
            this->az = -20.0;

            auto cmdloop_callback = [this]() -> void {
                if(this->offboard_setpoint_counter_ == 10){
                    // Change to Offboard mode after 10 setpoints
                    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, this->robot_id);

                    // Arm the vehicle
                    this->arm();

                }

                this->publish_offboard_control_mode();
                this->publish_trajectory_setpoint();

                // stop the counter after reaching 11
                if (this->offboard_setpoint_counter_ < 11) {
				    this->offboard_setpoint_counter_++;
                }
            };

            // start publisher timer
            timer_ = this->create_wall_timer(timer_period, cmdloop_callback);
        }

        void arm() ;
	    void disarm() ;

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::chrono::milliseconds timer_period;
        uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
        uint8_t nav_state;
	    uint8_t arming_state;
        double radius;
        double theta;
        double omega;
        std::string node_namespace;
        uint8_t robot_id;
        double ax;
        double az;

        // Subscriber
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

        // publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

        // callbacks
        void vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg)
        {
            this->arming_state = msg.arming_state;
            this->nav_state = msg.nav_state;
            // RCLCPP_INFO(this->get_logger(), "ArmingState: %i\nNavState: %i", this->arming_state, this->nav_state);
        }

        // commands
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void ControlTest::arm()
{
    // send the arm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, this->robot_id);

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void ControlTest::disarm()
{
    // send the disarm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, this->robot_id);

	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void ControlTest::publish_offboard_control_mode(){
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void ControlTest::publish_trajectory_setpoint(){
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
    msg.acceleration[0] = this->ax;
    msg.acceleration[1] = 0.0;
    msg.acceleration[2] = this->az;

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void ControlTest::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t system_id)
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
    rclcpp::spin(std::make_shared<ControlTest>());
    rclcpp::shutdown();
    return 0;
}
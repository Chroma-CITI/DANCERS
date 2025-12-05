#include <rclcpp/rclcpp.hpp>

// PX4 messages
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

// ROS2 messages
#include <geometry_msgs/msg/point.hpp>

// Eigen
#include <Eigen/Dense>

using std::placeholders::_1;

class WaypointControl : public rclcpp::Node
{
public:
    WaypointControl() : rclcpp::Node("waypoint_control")
    {
        // Declare the robot_id ROS2 parameter
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Name of the robot. The topics are namespaced by this name, e.g. px4_1.";
        this->declare_parameter("robot_name", "px4_1", param_desc);

        this->robot_name = this->get_parameter("robot_name").get_parameter_value().get<std::string>();
        
        // Find the robot ID in the name (robot name must have the form <...>_<ID>)
        try
        {
            this->robot_id = std::stoi(this->robot_name.substr(this->robot_name.find_last_of("_") + 1));
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "The robot_name parameter must have the form <...>_<ID>, e.g. px4_1, px4_2, etc. The ID must be an integer.");
            throw e;
        }

        // This is a very weird behavior of the PX4 code. The PX4 with instance 0 has no namespace, whereas all other instances are namespaces by px4_1, px4_2, etc.
        std::string px4_namespace = this->robot_name;
        if (this->robot_id == 0)
        {
            px4_namespace = "";
        }

        // Prepare the QoS that we will assign to the publishers / subscribers  
        auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
        
        // Create subscribers
        this->waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            this->robot_name + "/waypoint",
            qos_sub,
            std::bind(&WaypointControl::waypoint_clbk, this, _1));
        this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            px4_namespace + "/fmu/out/vehicle_status_v1",
            qos_sub, 
            std::bind(&WaypointControl::vehicle_status_clbk, this, _1));

        // Create publishers
        this->offboard_control_mode_publisher_ =
                this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                    px4_namespace + "/fmu/in/offboard_control_mode", 
                    qos_pub);
        this->trajectory_setpoint_publisher_ =
                this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                    px4_namespace + "/fmu/in/trajectory_setpoint", 
                    qos_pub);
        this->vehicle_command_publisher_ = 
                this->create_publisher<px4_msgs::msg::VehicleCommand>(
                    px4_namespace + "/fmu/in/vehicle_command",
                    qos_pub);

        // Set a 10 Hz control loop period
        this->timer_period_ = std::chrono::microseconds(100000);

        // Start the command loop
        this->timer_ = this->create_wall_timer(timer_period_, std::bind(&WaypointControl::cmd_loop_clbk, this));
    }

private:

    std::string robot_name;                         //!< name of the robot (topics namespace)
    unsigned int robot_id;                          //!< id of the robot

    rclcpp::TimerBase::SharedPtr timer_;            //!< timer calling the control loop
    std::chrono::microseconds timer_period_;         //!< timer period (time between two calls of the control loop)

    uint8_t nav_state_;                              //!< navigation state of the PX4 robot
    uint8_t arming_state_;                           //!< arming state of the PX4 robot

    geometry_msgs::msg::Point current_waypoint = geometry_msgs::msg::Point();     //!< the currently followed waypoint
    double theta = 0;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub_;

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // callbacks
    void vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg);
    void waypoint_clbk(const geometry_msgs::msg::Point & msg);
    void cmd_loop_clbk();

    // commands
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(Eigen::Vector3d waypoint, float yaw);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);

    // util
    Eigen::Vector3d ENU_to_NED(Eigen::Vector3d vec);

};

/**
 * @brief Rotation of a 3D vector from ENU (East North Up) to NED (North East Down) frame.
 */
Eigen::Vector3d WaypointControl::ENU_to_NED(Eigen::Vector3d vec)
{
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0); // Rotation axis (z-axis)
    Eigen::Quaterniond rot_half_pi_z(Eigen::AngleAxisd(M_PI / 2, z_axis));
    rot_half_pi_z.normalize();

    Eigen::Vector3d y_axis(0.0, 1.0, 0.0); // Rotation axis (x-axis)
    Eigen::Quaterniond rot_pi_y(Eigen::AngleAxisd(M_PI, y_axis));
    rot_pi_y.normalize();

    vec = rot_half_pi_z * vec;
    vec = rot_pi_y * vec;

    return vec;
}

/**
 * @brief Callback for the /fmu/out/vehicle_status topic
 * 
 * Saves only the arming state and the navigation state to internal variables.
 */
void WaypointControl::vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg)
{
    this->arming_state_ = msg.arming_state;
    this->nav_state_ = msg.nav_state;
}

/**
 * @brief Callback for the /waypoint topic
 *
 * Saves the waypoint to internal variables.
 */
void WaypointControl::waypoint_clbk(const geometry_msgs::msg::Point & msg)
{
    this->current_waypoint = msg;
}

/**
 * @brief Main control loop
 * 
 * This function is called at a fixed frequency by a timer.
 * It tries to switch to offboard mode and arm the vehicle if not already done.
 * Then, it publishes the offboard control mode and the current waypoint (must be done at more than 2 Hz. Here we aim for 10 Hz).
 */
void WaypointControl::cmd_loop_clbk()
{
    // Always try to switch to offboard mode and arm the vehicle if in another mode
    // The VehicleCommand (uOrb messages) correspond to the equivalent MAVLink messages
    // See https://docs.px4.io/main/en/msg_docs/VehicleCommand#vehiclecommand-uorb-message
    // and https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
    if(this->nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD || this->arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
        // engage the offboard control mode
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, this->robot_id);
        RCLCPP_INFO(this->get_logger(), "Set Offboard mode command sent");

        // Arm the vehicle
	    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, this->robot_id);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");

        // sleep to let a chance to the robot to receive the messages
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Publish the offboard control mode at each loop : Mandatory to keep this topic published > 2 Hz to keep the offboard mode
    this->publish_offboard_control_mode();

    // Publish the waypoint (position controller). All "trajectory setpoints" are supposed to be in NED convention
    Eigen::Vector3d wp = {this->current_waypoint.x, this->current_waypoint.y, this->current_waypoint.z};
    this->publish_trajectory_setpoint(ENU_to_NED(wp), this->theta);
}

/**
 * @brief Publish the offboard control mode.
 * 
 * For this example, only position and altitude controls are active.
 */
void WaypointControl::publish_offboard_control_mode(){
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 */
void WaypointControl::publish_trajectory_setpoint(Eigen::Vector3d waypoint, float yaw){
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);

    msg.position[0] = waypoint.x();
    msg.position[1] = waypoint.y();
    msg.position[2] = waypoint.z(); 

    msg.velocity[0] = NAN;
    msg.velocity[1] = NAN;
    msg.velocity[2] = NAN;

    msg.acceleration[0] = NAN;
    msg.acceleration[1] = NAN;
    msg.acceleration[2] = NAN;

    msg.jerk[0] = NAN;
    msg.jerk[1] = NAN;
    msg.jerk[2] = NAN;

    msg.yaw = yaw;

    RCLCPP_INFO(this->get_logger(), "Sending trajectory setpoint to UAV %i: at position %f, %f, %f", this->robot_id, msg.position[0], msg.position[1], msg.position[2]);

    trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void WaypointControl::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t system_id)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = system_id+1; // +1 because in simulation the PX4 instance numbering is off by 1...
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	this->vehicle_command_publisher_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointControl>());
    rclcpp::shutdown();
    return 0;
}
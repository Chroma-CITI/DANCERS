#include <iostream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// Network libraries
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// PX4 ROS messages
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

// Standard ROS messages
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

// DANCERS ROS messages
#include <dancers_msgs/msg/neighbor_id_quality_array.hpp>

// Obstacle detector messages
#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Function used for flocking computation, see curve in Vásárhelyi 2018 Fig.6.
 */
static double SigmoidLin(double r, double a, double p)
{
    if (r <= 0)
    {
        return 0;
    }
    else if (r * p > 0 && r * p < a / p)
    {
        return r * p;
    }
    else
    {
        return std::sqrt(2 * a * r - std::pow(a, 2) / std::pow(p, 2));
    }
}

/**
 * @brief Rotation of a 3D vector from ENU (East North Up) to NED (North East Down) frame.
 */
static Eigen::Vector3d ENU_to_NED(Eigen::Vector3d cmd)
{

    Eigen::Vector3d z_axis(0.0, 0.0, 1.0); // Rotation axis (z-axis)
    Eigen::Quaterniond rot_half_pi_z(Eigen::AngleAxisd(M_PI / 2, z_axis));
    rot_half_pi_z.normalize();

    Eigen::Vector3d y_axis(0.0, 1.0, 0.0); // Rotation axis (x-axis)
    Eigen::Quaterniond rot_pi_y(Eigen::AngleAxisd(M_PI, y_axis));
    rot_pi_y.normalize();

    cmd = rot_half_pi_z * cmd;
    cmd = rot_pi_y * cmd;

    return cmd;
}



/**
 * A struct holding position of a neighbor robot.
 */
struct UAV_data
{
    rclcpp::Time timestamp;                         // Timestamp of the last message received by this peer
    std::string name;                           // Name of the peer in the form <robot_model>_<id>
    uint32_t id;                                // ID of the peer
    Eigen::Vector3d initial_position;           // Initial position of the peer in the GLOBAL frame
    Eigen::Vector3d position;                   // The position of a neighbor in the LOCAL frame, i.e. relative positions. OR in "my_data", position relative to launch position.
    Eigen::Quaterniond orientation;             // The quaternion of the orientation of the UAV's frame with respect to its launch orientation
    Eigen::Vector3d velocity;                   // The velocity of a robot in the GLOBAL frame. We don't need the origin of the velocity,
    Eigen::Vector3d acceleration;
    double yaw;
    double distance;
    bool is_neighbor;                              // True if the peer is a neighbor, false otherwise
    double link_quality;                        // Link quality of the peer (of the last received message)
};

class VATPilot : public rclcpp::Node
{
public:
    // Constructor
    VATPilot() : rclcpp::Node("VATPilot")
    {

        // Declare the config_file ROS2 parameter
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);

        // Declare the robot_id ROS2 parameter
        param_desc.description = "ID of the robot. This is the system id of each robot (instance number + 1)";
        this->declare_parameter("robot_id", 1, param_desc);
        this->robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint32_t>();
        if (robot_id <= 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Wrong robot ID ! Robot IDs must start at 1. Received: %i", robot_id);
        }

        // Declare the use_gz_position ROS2 parameter (switch between ground truth positions and SITL sensor fusion)
        param_desc.description = "If true, use the ground truth position from the Gazebo simulation. If false, use the SITL sensor measurements.";
        this->declare_parameter("use_gz_positions", false, param_desc);
        this->use_gz_positions = this->get_parameter("use_gz_positions").get_parameter_value().get<bool>();
        RCLCPP_DEBUG(this->get_logger(), "Using ground truth positions from Gazebo");

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        this->num_robots = config["robots_number"].as<uint16_t>();
        this->offboard_setpoint_counter_ = 0;
        this->timer_period = std::chrono::microseconds(config["cmd_loop_period"].as<uint64_t>());                // 100 ms

        YAML::Node flocking_param = config["VAT_flocking_parameters"];
        this->v_flock = flocking_param["v_flock"].as<double>();
        this->v_max = flocking_param["v_max"].as<double>();
        this->a_frict = flocking_param["a_frict"].as<double>();
        this->p_frict = flocking_param["p_frict"].as<double>();
        this->r_0_frict = flocking_param["r_0_frict"].as<double>();
        this->C_frict = flocking_param["C_frict"].as<double>();
        this->v_frict = flocking_param["v_frict"].as<double>();
        this->p_att = flocking_param["p_att"].as<double>();
        this->r_0_att = flocking_param["r_0_att"].as<double>();
        this->p_rep = flocking_param["p_rep"].as<double>();
        this->r_0_rep = flocking_param["r_0_rep"].as<double>();
        this->a_shill = flocking_param["a_shill"].as<double>();
        this->p_shill = flocking_param["p_shill"].as<double>();
        this->r_0_shill = flocking_param["r_0_shill"].as<double>();
        this->v_shill = flocking_param["v_shill"].as<double>();

        this->target_altitude_ = config["target_altitude"].as<double>();

        this->timeout_value = config["neighbor_timeout_value"].as<uint64_t>() * 1000; // convert from microseconds to nanoseconds 

        this->secondary_objective_flag = flocking_param["secondary_objective_flag"].as<bool>();
        for (auto sec_obj : flocking_param["secondary_objectives"])
        {
            Eigen::Vector3d pos(sec_obj.second[0].as<double>(), sec_obj.second[1].as<double>(), sec_obj.second[2].as<double>());
            this->secondary_objectives.insert(std::pair<uint32_t, Eigen::Vector3d>(sec_obj.first.as<uint32_t>(), pos));
        }


        // Initialize the manual propulsion vector to 0
        this->manual_propulsion = {0.0, 0.0, 0.0};

        // Prepare the QoS that we will assign to the publishers / subscribers
        auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        // Initialize the peers information
        for (int i = 1; i <= num_robots; i++)
        {
            std::string robot_name = config["robots_model"].as<std::string>() + "_" + std::to_string(i); // e.g. x500_1, x500_2, etc.
            UAV_data data;
            data.name = robot_name;
            data.id = i;
            data.is_neighbor = false;
            this->robots_data[i] = data;
            RCLCPP_DEBUG(this->get_logger(), "Tracked robot: %s", robot_name.c_str());
        }

        // SUBSCRIBERS
        // Subscribe to your own .../vehicle_status
        this->my_vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/px4_" + std::to_string(robot_id) + "/fmu/out/vehicle_status",
            qos_sub,
            std::bind(&VATPilot::my_vehicle_status_clbk, this, _1));
        // Subscribe to your own .../raw_obstacles
        this->my_raw_obstacle_sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
            this->robots_data[this->robot_id].name + "/raw_obstacles",
            qos_sub,
            std::bind(&VATPilot::my_raw_obstacle_clbk, this, _1));
        // Subscribe to your own neighbors list .../neighbors
        this->my_neighbors_sub_ = this->create_subscription<dancers_msgs::msg::NeighborIdQualityArray>(
            this->robots_data[this->robot_id].name + "/neighbors",
            qos_sub,
            std::bind(&VATPilot::my_neighbors_clbk, this, _1));

        if(this->use_gz_positions)
        {
            // Subscribe to all of the .../odometry
            for (auto a : this->robots_data)
            {
                UAV_data agent = a.second;
                this->neighbors_odom_subs.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
                    agent.name + "/odometry",
                    qos_sub,
                    [this, agent](const nav_msgs::msg::Odometry &odom)
                    { VATPilot::agent_odom_clbk(odom, agent.id); }));
                RCLCPP_DEBUG(this->get_logger(), "Subscribed to robot %d /odometry topic.", agent.id);
            }
        }
        else 
        {
            // Subscribe to all of the .../vehicle_local_position
            for (auto a : this->robots_data)
            {
                UAV_data agent = a.second;
                this->neighbors_vehicle_local_pos_subs.push_back(this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                    "/px4_" + std::to_string(robot_id) + "/fmu/out/vehicle_local_position",
                    qos_sub,
                    [this, agent](const px4_msgs::msg::VehicleLocalPosition &local_pos)
                    { VATPilot::agent_vehicle_local_position_clbk(local_pos, agent.id); }));
                RCLCPP_DEBUG(this->get_logger(), "Subscribed to robot %d /vehicle_local_position topic.", agent.id);
            }
        }

        // Subscribe to the global /cmd_vel (global command)
        this->cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            qos_sub,
            std::bind(&VATPilot::cmd_vel_clbk, this, _1));


        // PUBLISHERS
        // Always publish on your own .../offboard_control_mode to keep the offboard mode (minimum at 2Hz)
        this->offboard_control_mode_publisher_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_" + std::to_string(robot_id) + "/fmu/in/offboard_control_mode", qos_pub);
        // publish the trajectory setpoints on your own .../trajectory_setpoint
        this->trajectory_setpoint_publisher_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_" + std::to_string(robot_id) + "/fmu/in/trajectory_setpoint", qos_pub);
        // publish commands such as mode change and arming commands to your own .../vehicle_command
        this->vehicle_command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_" + std::to_string(robot_id) + "/fmu/in/vehicle_command", qos_pub);

        // start publisher timer
        timer_ = rclcpp::create_timer(this, this->get_clock(), this->timer_period, std::bind(&VATPilot::cmd_loop_clbk, this));

        RCLCPP_INFO(this->get_logger(), "VATPilot initialized for drone %d", this->robots_data[this->robot_id].id);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::microseconds timer_period;
    bool use_gz_positions;
    uint64_t offboard_setpoint_counter_;                   //!< counter for the number of setpoints sent
    double target_altitude_;
    uint8_t nav_state;
    uint8_t arming_state;
    uint32_t robot_id; //!< convenience local variable
    uint16_t num_robots;
    std::map<uint32_t, UAV_data> robots_data;
    obstacle_detector::msg::Obstacles known_raw_obstacles;
    Eigen::Vector3d manual_propulsion;
    uint64_t timeout_value;

    bool secondary_objective_flag;

    // flocking parameters (set them in the configuration file)
    double v_flock;
    double v_max;
    // Alignment
    double a_frict;
    double p_frict;
    double r_0_frict;
    double C_frict;
    double v_frict;
    // Attraction
    double p_att;
    double r_0_att;
    // repulsion
    double p_rep;
    double r_0_rep;
    // shill
    double a_shill;
    double p_shill;
    double r_0_shill;
    double v_shill;

    std::map<uint32_t, Eigen::Vector3d> secondary_objectives;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr my_vehicle_status_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr my_raw_obstacle_sub_;
    rclcpp::Subscription<dancers_msgs::msg::NeighborIdQualityArray>::SharedPtr my_neighbors_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbors_odom_subs;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> neighbors_vehicle_local_pos_subs;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // callbacks
    void my_vehicle_status_clbk(const px4_msgs::msg::VehicleStatus &msg);
    void my_raw_obstacle_clbk(const obstacle_detector::msg::Obstacles &msg);
    void my_neighbors_clbk(const dancers_msgs::msg::NeighborIdQualityArray &msg);
    void gz_positions_clbk(const geometry_msgs::msg::PoseArray &msg);
    void agent_odom_clbk(const nav_msgs::msg::Odometry &odom, int peer_id);
    void agent_vehicle_local_position_clbk(const px4_msgs::msg::VehicleLocalPosition &local_pos, int peer_id);
    void cmd_vel_clbk(const geometry_msgs::msg::Twist &msg);

    void cmd_loop_clbk();

    // functions
    void arm();
    void disarm();
    void engage_offboard_mode();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(Eigen::Vector3d target, double yaw = M_PI / 2);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);
    Eigen::Vector3d get_shill_agent_position(Eigen::Vector3d start, Eigen::Vector3d end);
    Eigen::Vector3d compute_flocking_command();

    // flocking functions
    Eigen::Vector3d repulsion_term();
    Eigen::Vector3d attraction_term();
    Eigen::Vector3d alignment_term();
    Eigen::Vector3d shill_term();
    Eigen::Vector3d secondary_objective_term();

};

void VATPilot::my_vehicle_status_clbk(const px4_msgs::msg::VehicleStatus &msg)
{
    this->arming_state = msg.arming_state;
    this->nav_state = msg.nav_state;
    RCLCPP_DEBUG(this->get_logger(), "Saved vehicle status data, nav state=%d", msg.nav_state);
}

void VATPilot::my_raw_obstacle_clbk(const obstacle_detector::msg::Obstacles &_msg)
{
    this->known_raw_obstacles = _msg;
    RCLCPP_DEBUG(this->get_logger(), "Saved obstacles data");
}

void VATPilot::my_neighbors_clbk(const dancers_msgs::msg::NeighborIdQualityArray &_msg)
{
    for (auto neighbor : _msg.neighbors_id_quality_array)
    {
        this->robots_data[neighbor.id].is_neighbor = true;
        this->robots_data[neighbor.id].link_quality = neighbor.link_quality;
        this->robots_data[neighbor.id].timestamp = this->get_clock()->now();
    }
    RCLCPP_INFO(this->get_logger(), "[%d] Saved neighbors data. Current neighbors:", this->robot_id);
    for (auto a : this->robots_data)
    {
        UAV_data agent = a.second;
        if (agent.is_neighbor)
        {
            RCLCPP_INFO(this->get_logger(), "[%d] Neighbor %d", this->robot_id, agent.id);
        }
    }
}

// Callback called at each Odometry message received from agents
// Saves the position and velocity information in the UAV_data structure, the position is in the current UAV frame
void VATPilot::agent_odom_clbk(const nav_msgs::msg::Odometry &odom, int peer_id)
{

    Eigen::Vector3d peer_position_global = {odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
    Eigen::Quaterniond peer_orientation_global = {odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z};
    peer_orientation_global.normalize();
    Eigen::Vector3d peer_velocity_in_peer_frame = {odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z};

    this->robots_data[peer_id].velocity = peer_orientation_global * peer_velocity_in_peer_frame;                 // this is the peer velocity in the global frame
    if(peer_id != this->robot_id)
    {
        this->robots_data[peer_id].position = peer_position_global - this->robots_data[this->robot_id].position; // this is peer position relative to our current position
        this->robots_data[peer_id].distance = this->robots_data[peer_id].position.norm();                         // this is the distance to the peer (look two lines up)
    } else {
        this->robots_data[peer_id].position = peer_position_global;
    }

    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i position: %f, %f, %f", peer_index, this->robots_data[peer_index].position[0], this->robots_data[peer_index].position[1], this->robots_data[peer_index].position[2]);
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i velocity: %f, %f, %f", peer_index, this->robots_data[peer_index].velocity[0], this->robots_data[peer_index].velocity[1], this->robots_data[peer_index].velocity[2]);
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i distance: %f", peer_index, this->robots_data[peer_index].distance);
}

void VATPilot::agent_vehicle_local_position_clbk(const px4_msgs::msg::VehicleLocalPosition &local_pos, int peer_id)
{
    Eigen::Vector3d peer_position_global = {local_pos.x, local_pos.y, local_pos.z};
    // Eigen::Quaterniond peer_orientation_global = {local_pos., local_pos.q[1], local_pos.q[2], local_pos.q[3]};
    // peer_orientation_global.normalize();
    Eigen::Vector3d peer_velocity_in_peer_frame = {local_pos.vx, local_pos.vy, local_pos.vz};

    this->robots_data[peer_id].position = peer_position_global - this->robots_data[this->robot_id].position; // this is peer position relative to our current position
    // this->robots_data[peer_index].velocity = peer_orientation_global * peer_velocity_in_peer_frame;                 // this is the peer velocity in the global frame
    this->robots_data[peer_id].distance = this->robots_data[peer_id].position.norm();                         // this is the distance to the peer (look two lines up)

    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i position: %f, %f, %f", peer_index, this->robots_data[peer_index].position[0], this->robots_data[peer_index].position[1], this->robots_data[peer_index].position[2]);
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i velocity: %f, %f, %f", peer_index, this->robots_data[peer_index].velocity[0], this->robots_data[peer_index].velocity[1], this->robots_data[peer_index].velocity[2]);
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i distance: %f", peer_index, this->robots_data[peer_index].distance);
}

/**
 * @brief Callback that handles a global force vector added to the result of the flocking to manually control the fleet
 */
void VATPilot::cmd_vel_clbk(const geometry_msgs::msg::Twist &msg)
{
    this->manual_propulsion = {msg.linear.x, msg.linear.y, msg.linear.z};
}

/**
 * @brief Callback called at each iteration of the timer. It is called each timer_period time based on the ROS2 simulation clock
 */
void VATPilot::cmd_loop_clbk()
{
    RCLCPP_DEBUG(this->get_logger(), "Entered cmd loop drone %d", this->robot_id);
        
    if (this->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
        // Change to Offboard mode
        this->engage_offboard_mode();
    }
    
    if (this->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
        // Arm the vehicle
        this->arm();
    }

    // CMD must be expressed in the body frame with the FRD convention.
    Eigen::Vector3d cmd = {0.0, 0.0, 0.0};
    if (this->robots_data[this->robot_id].position.z() < this->target_altitude_ && this->offboard_setpoint_counter_ < 100)
    {
        cmd = {0.0, 0.0, 4.0};
    }
    else
    {
        cmd = compute_flocking_command();
    }
    // publish to offboard control mode every loop to stay in offboard mode
    this->publish_offboard_control_mode();
    if (this->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
        // The flocking command is in the local frame, so we need to transform it to the global frame.
        this->publish_trajectory_setpoint(ENU_to_NED(this->robots_data[this->robot_id].orientation * cmd), NAN);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Offboard mode not active");
    }

    this->offboard_setpoint_counter_++;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void VATPilot::arm()
{
    // send the arm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, this->robot_id + 1);

    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void VATPilot::disarm()
{
    // send the disarm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, this->robot_id + 1);

    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Send a command to set the vehicle to Offboard mode
 */
void VATPilot::engage_offboard_mode()
{
    // send the arm command in a VehicleCommand message. 1 is offboard, 6 is ???
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, this->robot_id + 1);

    RCLCPP_INFO(this->get_logger(), "Set Offboard mode command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void VATPilot::publish_offboard_control_mode()
{
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
 * @brief Publish a trajectory setpoint from a velocity vector and a yaw angle
 *
 * \param vel_target Velocity vector in the NED frame
 * \param yaw Yaw angle in radians (0 is North, pi/2 is East, etc.)
 */
void VATPilot::publish_trajectory_setpoint(Eigen::Vector3d vel_target, double yaw)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);

    // We obtain undefined behavior if the unused fields are not explicitly set to NaN
    msg.position[0] = NAN;
    msg.position[1] = NAN;
    msg.position[2] = NAN; // The Z axis is toward the ground

    // We obtain undefined behavior if the unused fields are not explicitly set to NaN
    msg.velocity[0] = vel_target[0];
    msg.velocity[1] = vel_target[1];
    msg.velocity[2] = vel_target[2];

    msg.acceleration[0] = NAN;
    msg.acceleration[1] = NAN;
    msg.acceleration[2] = NAN;

    msg.jerk[0] = NAN;
    msg.jerk[1] = NAN;
    msg.jerk[2] = NAN;

    msg.yaw = yaw;

    RCLCPP_DEBUG(this->get_logger(), "My position: %f, %f, %f", this->robots_data[this->robot_id].position[0], this->robots_data[this->robot_id].position[1], this->robots_data[this->robot_id].position[2]);

    // RCLCPP_INFO(this->get_logger(), "Sending trajectory setpoint to UAV %i: at position %f, %f, %f", this->robot_id, msg.position[0], msg.position[1], msg.position[2]);
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
void VATPilot::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t system_id)
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

/**
 * \brief Calculates the position of the shill agent in the segment obstacle (closest point from agent to segment)
 *
 * from https://paulbourke.net/geometry/pointlineplane/
 *
 * \param  start Start point of the segment [frame: world]
 * \param  end   End point of the segment [frame: world]
 * \return       Position of the shill agent [frame: world]
 */
Eigen::Vector3d VATPilot::get_shill_agent_position(Eigen::Vector3d segment_start, Eigen::Vector3d segment_end)
{
    Eigen::Vector3d agent_pose;
    Eigen::Vector3d robot_pose = {0.0, 0.0, 0.0}; // The obstacle is in the lidar's frame, so we set the robot pose to 0
    double u = ((robot_pose.x() - segment_start.x()) * (segment_end.x() - segment_start.x()) + (robot_pose.y() - segment_start.y()) * (segment_end.y() - segment_start.y())) / std::pow((segment_start - segment_end).norm(), 2);
    if (u >= 1.0)
    {
        u = 1.0;
    }
    if (u <= 0.0)
    {
        u = 0.0;
    }
    agent_pose[0] = segment_start.x() + u * (segment_end.x() - segment_start.x());
    agent_pose[1] = segment_start.y() + u * (segment_end.y() - segment_start.y());
    agent_pose[2] = robot_pose[2];

    return agent_pose;
}

/**
 * The computation of the command law is done in the local frame of the robot (we only have relative position estimation),
 * within the Front-Right-Down (FRD) coodrinate system
 */
Eigen::Vector3d VATPilot::compute_flocking_command()
{
    RCLCPP_DEBUG(this->get_logger(), "velocity: %f, %f, %f", this->robots_data[this->robot_id].velocity[0], this->robots_data[this->robot_id].velocity[1], this->robots_data[this->robot_id].velocity[2]);
    Eigen::Vector3d autopropulsion = this->robots_data[this->robot_id].velocity.normalized() * this->v_flock;
    Eigen::Vector3d repulsion = this->repulsion_term();
    Eigen::Vector3d alignment = this->alignment_term();
    Eigen::Vector3d attraction = this->attraction_term();
    Eigen::Vector3d shill = this->shill_term();
    Eigen::Vector3d secondary_objective_vector = this->secondary_objective_term();


    Eigen::Vector3d desired_velocity = 
                alignment + 
                attraction + 
                repulsion + 
                shill + 
                autopropulsion + 
                this->manual_propulsion;

    if (this->secondary_objective_flag)
    {
        desired_velocity += secondary_objective_vector;
    }


    RCLCPP_DEBUG(this->get_logger(), "autopropulsion: %f, %f, %f", autopropulsion[0], autopropulsion[1], autopropulsion[2]);
    RCLCPP_DEBUG(this->get_logger(), "repulsion: %f, %f, %f", repulsion[0], repulsion[1], repulsion[2]);
    RCLCPP_DEBUG(this->get_logger(), "alignment: %f, %f, %f", alignment[0], alignment[1], alignment[2]);
    RCLCPP_DEBUG(this->get_logger(), "attraction: %f, %f, %f", attraction[0], attraction[1], attraction[2]);
    RCLCPP_DEBUG(this->get_logger(), "shill: %f, %f, %f", shill[0], shill[1], shill[2]);

    // Desired velocity saturates at v_max
    if (desired_velocity.norm() > this->v_max)
    {
        desired_velocity = desired_velocity.normalized() * this->v_max;
    }

    // Small dumb Proportional controller for height (Z is towards UP because we are in ENU convention, )
    if (this->robots_data[this->robot_id].position[2] > this->target_altitude_ + 0.5)
    {
        desired_velocity[2] = 0.1 * (this->target_altitude_ - this->robots_data[this->robot_id].position[2]);
    }
    else if (this->robots_data[this->robot_id].position[2] < this->target_altitude_ - 0.5)
    {
        desired_velocity[2] = 0.1 * (this->target_altitude_ - this->robots_data[this->robot_id].position[2]);
    }
    else
    {
        desired_velocity[2] = 0.0;
    }

    return desired_velocity;

}

/** 
 * Notations and equations from Vasarhelyi 2018
*/
Eigen::Vector3d VATPilot::alignment_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (auto r : this->robots_data)
    {
        UAV_data robot = r.second;
        if (robot.id != this->robot_id && robot.position.hasNaN() == false && robot.distance != NAN && robot.is_neighbor)
        {
            double velDiffNorm = (robot.velocity - this->robots_data[this->robot_id].velocity).norm();
            double v_frictmax = std::max(this->v_frict, SigmoidLin(robot.distance - this->r_0_frict, this->a_frict, this->p_frict));
            if (velDiffNorm > v_frictmax)
            {
                result += this->C_frict * (velDiffNorm - v_frictmax) * (robot.velocity - this->robots_data[this->robot_id].velocity) / velDiffNorm;
            }
        }
    }
    return result;
}

/**
 * From model "Vasarhelyi + Attraction", Bonnefond 2021
*/
Eigen::Vector3d VATPilot::attraction_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (auto r : this->robots_data)
    {
        UAV_data robot = r.second;
        if (robot.id != this->robot_id && robot.position.hasNaN() == false && robot.distance != NAN && robot.is_neighbor)
        {
            if (robot.distance > this->r_0_att)
            {
                result += this->p_att * (this->r_0_att - robot.distance) * (-robot.position) / robot.distance;
            }
        }
    }
    return result;
}

/** 
 * Notations and equations from Vasarhelyi 2018
*/
Eigen::Vector3d VATPilot::repulsion_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (auto r : this->robots_data)
    {
        UAV_data robot = r.second;
        if (robot.id != this->robot_id && robot.position.hasNaN() == false && robot.is_neighbor)
        {
            if (robot.distance < this->r_0_rep && robot.distance > 0.0)
            {
                result += this->p_rep * (this->r_0_rep - robot.distance) * (-robot.position) / robot.distance;
            }
        }
    }
    return result;
}

/**
 * @brief Computes the effect of obstacles on the command law, which is called "shill agent" by Vasarhelyi and Viragh
 *
 * @return Eigen::Vector3d
 */
Eigen::Vector3d VATPilot::shill_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (uint32_t i = 0; i < this->known_raw_obstacles.segments.size(); i++)
    {
        Eigen::Vector3d segment_start = {this->known_raw_obstacles.segments[i].first_point.x, this->known_raw_obstacles.segments[i].first_point.y, this->known_raw_obstacles.segments[i].first_point.z};
        Eigen::Vector3d segment_end = {this->known_raw_obstacles.segments[i].last_point.x, this->known_raw_obstacles.segments[i].last_point.y, this->known_raw_obstacles.segments[i].last_point.z};
        Eigen::Vector3d shill_agent_pose = this->get_shill_agent_position(segment_start, segment_end);
        Eigen::Vector3d shill_agent_velocity = this->v_shill * (-shill_agent_pose).normalized();

        double velDiffNorm = (this->robots_data[this->robot_id].velocity - shill_agent_velocity).norm();

        double v_shillmax = SigmoidLin(shill_agent_pose.norm() - this->r_0_shill, this->a_shill, this->p_shill);
        if (velDiffNorm > v_shillmax)
        {
            // without the minus sign, the shill component is TOWARDS the obstacle (it is in the opposite direction of the shill agent velocity, which points toward the agent)
            result += -(velDiffNorm - v_shillmax) * (this->robots_data[this->robot_id].velocity - shill_agent_velocity) / velDiffNorm;
        }
    }
    return result;
}

// Take a secondary objective location in the global frame, and compute the desired velocity toward it
Eigen::Vector3d VATPilot::secondary_objective_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    if (this->secondary_objectives.find(this->robot_id) != this->secondary_objectives.end())
    {
        result = this->secondary_objectives[this->robot_id] - this->robots_data[this->robot_id].position;
        result.normalize();
        result *= this->v_flock;
    }    
    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VATPilot>());
    rclcpp::shutdown();
    return 0;
}
#include <iostream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// Network libraries
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// PX4 messages
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

// ROS messages
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Obstacle detector messages
#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

// RVIZ message
#include <visualization_msgs/msg/marker.hpp>

// Gazebo imports
#include <gz/msgs.hh>
#include <gz/transport.hh>

#define BROADCAST_ADDR "10.0.0.255"
#define PORT 8090

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
 * @brief Wrapper to compute a distance between two points from position vectors
 */
static double calc_distance(Eigen::Vector3d a, Eigen::Vector3d b)
{
    return (a - b).norm();
}

/**
 * A struct holding position of a neighbor robot.
 */
struct UAV_data
{
    rclcpp::Time timestamp; // timestamp of the last information from this peer, in EPOCH nanoseconds
    bool valid;
    std::string name;
    uint32_t id;
    Eigen::Vector3d position;       // The position of a neighbor in the LOCAL frame, i.e. relative positions. OR in "my_data", position relative to launch position.
    Eigen::Quaterniond orientation; // The quaternion of the orientation of the UAV's frame with respect to its launch orientation
    Eigen::Vector3d velocity;       // The velocity of a robot in the GLOBAL frame. We don't need the origin of the velocity,
    Eigen::Vector3d acceleration;
    double yaw;
    double distance;
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

        this->auto_propulsion_flag = flocking_param["auto_propulsion_flag"].as<bool>();
        this->secondary_objective_flag = flocking_param["secondary_objective_flag"].as<bool>();

        for (auto sec_obj : flocking_param["secondary_objectives"])
        {
            Eigen::Vector3d pos(sec_obj.second[0].as<double>(), sec_obj.second[1].as<double>(), sec_obj.second[2].as<double>());
            this->secondary_objectives.insert(std::pair<uint32_t, Eigen::Vector3d>(sec_obj.first.as<uint32_t>(), pos));
        }

        for(const auto &elem : this->secondary_objectives)
        {
            RCLCPP_INFO(this->get_logger(), "Secondary objective %i : (%f, %f, %f)", elem.first, elem.second.x(), elem.second.y(), elem.second.z());
        }

        this->target_altitude_ = config["target_altitude"].as<double>();

        this->timeout_value = config["timeout_value"].as<uint64_t>() * 1000;

        this->rviz_enabled = config["rviz_enabled"].as<bool>();

        // Prepare the QoS that we will assign to the publishers / subscribers
        auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        // Declare the robot_id ROS2 parameter
        param_desc.description = "ID of the robot. This is the system id of each robot (instance number + 1)";
        this->declare_parameter("robot_id", 1, param_desc);
        this->robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint32_t>();
        if (robot_id <= 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Wrong robot ID ! Robot IDs must start at 1. Received: %i", robot_id);
        }

        // Declare the use_gz_position ROS2 parameter
        param_desc.description = "If true, use the ground truth position from the Gazebo simulation. If false, use the SITL sensor measurements.";
        this->declare_parameter("use_gz_positions", false, param_desc);
        this->use_gz_positions = this->get_parameter("use_gz_positions").get_parameter_value().get<bool>();
        RCLCPP_DEBUG(this->get_logger(), "Using ground truth positions from Gazebo");

        this->my_data = UAV_data();
        this->my_data.name = config["robots_model"].as<std::string>() + "_" + std::to_string(this->robot_id);
        this->my_data.id = this->robot_id;

        // Directly subscribe to Gazebo's position :
        // ------------------- Interaction with Gazebo -------------------
        bool executed{false};
        bool result{false};
        unsigned int timeout{5000};

        gz::msgs::StringMsg_V resp;

        // Request the /gazebo/worlds service to get the world's name and make sure it is correctly created.
        std::string service{"/gazebo/worlds"};
        executed = this->node.Request(service, timeout, resp, result);
        check_service_results(service, executed, result);

        std::string world_name = resp.data(0);
        RCLCPP_INFO(this->get_logger(), "World found with name : %s", world_name.c_str());

        // Callback to the /odometry Gazebo subscriber.
        std::function<void(const gz::msgs::Odometry &)> cbOdometry =
            [&](const gz::msgs::Odometry &_msg)
        {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->now(); // This should be the Gazebo time if the use_sim_time ROS parameter is True
            odom.header.frame_id = "world";
            odom.child_frame_id = this->my_data.name;
            odom.pose.pose.position.x = _msg.pose().position().x();
            odom.pose.pose.position.y = _msg.pose().position().y();
            odom.pose.pose.position.z = _msg.pose().position().z();
            odom.pose.pose.orientation.x = _msg.pose().orientation().x();
            odom.pose.pose.orientation.w = _msg.pose().orientation().w();
            odom.pose.pose.orientation.y = _msg.pose().orientation().y();
            odom.pose.pose.orientation.z = _msg.pose().orientation().z();

            odom.twist.twist.linear.x = _msg.twist().linear().x();
            odom.twist.twist.linear.y = _msg.twist().linear().y();
            odom.twist.twist.linear.z = _msg.twist().linear().z();
            odom.twist.twist.angular.x = _msg.twist().angular().x();
            odom.twist.twist.angular.y = _msg.twist().angular().y();
            odom.twist.twist.angular.z = _msg.twist().angular().z();

            // Copy the Gazebo odom to the my_data struct
            this->my_data.position = {odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
            Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
            q.normalize();
            this->my_data.orientation = q;
            this->my_data.velocity = {odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z};

            this->my_data.distance = NAN;
            this->my_data.yaw = this->my_data.orientation.toRotationMatrix().eulerAngles(0, 1, 2).z();

            this->my_current_odom = odom;
            // Send it through UDP broadcast
            // broadcast_odom(odom);
        };

        // Subscribe to the Gazebo "pose/info" topic with gazebo transport
        if (this->node.Subscribe("/model/" + this->my_data.name + "/odometry", cbOdometry))
        {
            RCLCPP_INFO(this->get_logger(), "Subscribed to /model/%s/odometry Gz topic", this->my_data.name.c_str());
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to subscribe to /model/%s/odometry Gz topic", this->my_data.name.c_str());
        }
        // --------------------------------------------------------------

        // SUBSCRIBERS
        // Subscribe to your own vehicle_status
        this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/px4_" + std::to_string(robot_id) +
                "/fmu/out/vehicle_status",
            qos_sub,
            std::bind(&VATPilot::vehicle_status_clbk, this, _1));
        // Subscribe to your own .../raw_obstacles
        this->raw_obstacle_sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
            this->my_data.name + "/raw_obstacles",
            qos_sub,
            std::bind(&VATPilot::raw_obstacle_clbk, this, _1));

        // PUBLISHERS
        this->offboard_control_mode_publisher_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_" + std::to_string(robot_id) + "/fmu/in/offboard_control_mode", qos_pub);
        this->trajectory_setpoint_publisher_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_" + std::to_string(robot_id) + "/fmu/in/trajectory_setpoint", qos_pub);
        this->vehicle_command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_" + std::to_string(robot_id) + "/fmu/in/vehicle_command", qos_pub);

        // Publishers for Rviz2
        if (this->rviz_enabled)
        {
            this->cmd_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::Marker>("/px4_" + std::to_string(robot_id) + "/cmd_marker", 10);
            this->att_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::Marker>("/px4_" + std::to_string(robot_id) + "/att_cmd_marker", 10);
            this->rep_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::Marker>("/px4_" + std::to_string(robot_id) + "/rep_cmd_marker", 10);
            this->ali_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::Marker>("/px4_" + std::to_string(robot_id) + "/ali_cmd_marker", 10);
            this->auto_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::Marker>("/px4_" + std::to_string(robot_id) + "/auto_cmd_marker", 10);
            this->shi_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::Marker>("/px4_" + std::to_string(robot_id) + "/shi_cmd_marker", 10);
        }

        this->offboard_setpoint_counter_ = 0;
        this->timer_period = std::chrono::microseconds(config["cmd_loop_period"].as<uint64_t>());                // 100 ms
        this->timer_odometry_period = std::chrono::microseconds(config["pose_broadcast_period"].as<uint64_t>()); // 1 s

        // Create sockets
        this->my_ip_addr = "10.0.0." + std::to_string(this->robot_id); // clumsy but works : 10.0.0.1 - 10.0.0.4 - 10.0.0.7 etc.

        // start server thread
        server_thread = std::thread(&VATPilot::server_thread_func, this, PORT);

        // start command loop
        timer_ = rclcpp::create_timer(this, this->get_clock(), this->timer_period, std::bind(&VATPilot::cmd_loop_clbk, this));
        timer_odometry = rclcpp::create_timer(this, this->get_clock(), this->timer_odometry_period, std::bind(&VATPilot::odom_loop_clbk, this));

        RCLCPP_INFO(this->get_logger(), "VATPilot initialized");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;    //!< timer for the command loop
    std::chrono::microseconds timer_period; //!< period of the timer for the command loop
    rclcpp::TimerBase::SharedPtr timer_odometry;
    std::chrono::microseconds timer_odometry_period;
    uint64_t offboard_setpoint_counter_;                   //!< counter for the number of setpoints sent
    uint8_t nav_state;                                     //!< state of the navigation, from the PX4
    uint8_t arming_state;                                  //!< state of the arming, from the PX4
    uint32_t robot_id;                                     //!< id of the ego robot, starts at 1
    uint16_t num_robots;                                   //!< number of robots in the simulation
    std::map<uint32_t, UAV_data> robots_data;              //!< map holding the information of the neighbors in the form <robot_id, UAV_data
    UAV_data my_data;                                      //!< information of the ego robot
    nav_msgs::msg::Odometry my_current_odom;               //!< current odometry of the ego robot (usually from Gazebo)
    obstacle_detector::msg::Obstacles known_raw_obstacles; //!< obstacles detected by the lidar and the obstacle_detector software
    bool use_gz_positions;                                 // would be cool to have a config choice for positioning source : use PX4 estimator or Gazebo ground truth
    gz::transport::Node node;
    uint64_t timeout_value;

    // flocking parameters
    double v_flock;
    double v_max;
    // Alignment
    double a_frict;
    double p_frict;
    double r_0_frict;
    double C_frict; // old value 0.05
    double v_frict; // old value 0.63
    // Attraction
    double p_att; // old value 0.006
    double r_0_att;
    // repulsion
    double p_rep;
    double r_0_rep;
    // shill
    double a_shill;
    double p_shill;
    double r_0_shill;
    double v_shill;

    bool auto_propulsion_flag;
    bool secondary_objective_flag;

    std::map<uint32_t, Eigen::Vector3d> secondary_objectives;

    double target_altitude_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr raw_obstacle_sub_;
    // rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // callbacks
    void vehicle_status_clbk(const px4_msgs::msg::VehicleStatus &msg);
    void raw_obstacle_clbk(const obstacle_detector::msg::Obstacles &data);
    void gz_positions_clbk(const geometry_msgs::msg::PoseArray &data);
    void cmd_loop_clbk();
    void odom_loop_clbk();

    // RVIZ related
    bool rviz_enabled;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr att_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rep_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ali_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr auto_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shi_marker_publisher_;
    visualization_msgs::msg::Marker rviz_arrow(Eigen::Vector3d start, Eigen::Vector3d end, int id, std::string frame_id, std::string color);

    // communications
    std::string my_ip_addr;
    std::thread server_thread;
    void server_thread_func(int port_number);
    void broadcast_odom(const nav_msgs::msg::Odometry &odom);
    int create_udp_rcv_socket(uint16_t port);

    // functions
    void arm();
    void disarm();
    void engage_offboard_mode();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(Eigen::Vector3d target, double yaw = M_PI / 2);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);
    Eigen::Vector3d get_shill_agent_position(Eigen::Vector3d start, Eigen::Vector3d end);
    Eigen::Vector3d compute_flocking_command();
    void check_service_results(std::string service, bool executed, bool result);
    void print_valid_neighbors();

    // flocking functions
    Eigen::Vector3d repulsion_term();
    Eigen::Vector3d attraction_term();
    Eigen::Vector3d alignment_term();
    Eigen::Vector3d shill_term();
    Eigen::Vector3d secondary_objective_term();
};

/**
 * \brief Prints the result of a call to a Gazebo Service (just for code compactness)
 *
 * \param service The name of the service
 * \param executed If the service timed out
 * \param result If the call to the service failed
 */
void VATPilot::check_service_results(std::string service, bool executed, bool result)
{
    if (!executed)
    {
        std::cerr << std::endl
                  << "Service call to [" << service << "] timed out"
                  << std::endl;
        exit(EXIT_FAILURE);
    }
    if (!result)
    {
        std::cerr << std::endl
                  << "Service call to [" << service << "] failed"
                  << std::endl;
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Creates a Marker object for visualization in Rviz2
 *
 * \param start The starting point of the arrow
 * \param end The ending point of the arrow
 * \param id The id of the marker
 * \param frame_id The parent frame_id of the marker
 * \param color The color of the arrow, can be "red", "green", "blue", "violet", "cyan" or "yellow"
 * \return The marker object
 */
visualization_msgs::msg::Marker VATPilot::rviz_arrow(Eigen::Vector3d start, Eigen::Vector3d end, int id, std::string frame_id, std::string color)
{
    geometry_msgs::msg::Point start_arrow;
    start_arrow.x = start.x();
    start_arrow.y = start.y();
    start_arrow.z = start.z();
    geometry_msgs::msg::Point end_arrow;
    end_arrow.x = end.x();
    end_arrow.y = end.y();
    end_arrow.z = end.z();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    // marker.header.stamp = rclcpp::Time(0);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.ns = "flocking_cmd";
    marker.id = id;
    marker.points.push_back(start_arrow);
    marker.points.push_back(end_arrow);
    marker.color.a = 1.0;
    if (color == "red")
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else if (color == "green")
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        ;
    }
    else if (color == "blue")
    {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    else if (color == "violet")
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    else if (color == "cyan")
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    }
    else
    {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    }
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;

    return marker;
}

void VATPilot::vehicle_status_clbk(const px4_msgs::msg::VehicleStatus &msg)
{
    this->arming_state = msg.arming_state;
    this->nav_state = msg.nav_state;
}

void VATPilot::raw_obstacle_clbk(const obstacle_detector::msg::Obstacles &_msg)
{
    this->known_raw_obstacles = _msg;
    RCLCPP_DEBUG(this->get_logger(), "Saved obstacles data");
}

void VATPilot::print_valid_neighbors()
{
    std::string line = "";
    for (auto &n : this->robots_data)
    {
        if (n.second.valid)
        {
            line += "\x1b[32m";
        }
        else
        {
            line += "\x1b[31m";
        }
        line += "[" + std::to_string(n.first) + "] \x1b[0m";
    }
    std::cout << line << std::endl;
}

/**
 * @brief Main command loop, called at each timer's tick
 *
 * This is the high level plan for the flight, including the flight mode changes for PX4, arming,
 * a hand-made takeoff and the call to the flocking algorithm
 */
void VATPilot::cmd_loop_clbk()
{
    RCLCPP_DEBUG(this->get_logger(), "Entered cmd loop drone %d", this->robot_id);

    if (this->offboard_setpoint_counter_ <= 10)
    {
        // Change to Offboard mode after 10 setpoints (1s)
        this->engage_offboard_mode();

        // Arm the vehicle
        this->arm();
    }

    // Clean outdated peer positions
    for (auto peer : this->robots_data)
    {
        rclcpp::Duration position_age = this->get_clock()->now() - peer.second.timestamp;
        if (position_age > rclcpp::Duration(std::chrono::nanoseconds(this->timeout_value)) && peer.second.valid)
        {
            this->robots_data[peer.first].valid = false;
            RCLCPP_WARN(this->get_logger(), "\x1b[31m[%d] Peer %d position outdated (%ld) !\x1b[0m", this->robot_id, peer.first, position_age.nanoseconds());
        }
    }

    // this->print_valid_neighbors();

    // CMD must be expressed in the body frame with the FRD convention.
    Eigen::Vector3d cmd = {0.0, 0.0, 0.0};
    if (this->my_data.position.z() < this->target_altitude_ && this->offboard_setpoint_counter_ < 100)
    {
        cmd = {0.0, 0.0, 4.0}; // Takeoff speed is fixed at 4 m/s towards up
    }
    else
    {
        cmd = compute_flocking_command();
    }

    this->publish_offboard_control_mode();
    if (this->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
        this->publish_trajectory_setpoint(ENU_to_NED(this->my_data.orientation * cmd), NAN);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Offboard mode not active");
    }

    this->offboard_setpoint_counter_++;
}

void VATPilot::odom_loop_clbk()
{
    broadcast_odom(this->my_current_odom);
}

/**
 * @brief Handles the reception of the nav_msgs/Odometry messages from the neighbors
 *
 *
 */
void VATPilot::server_thread_func(int port_number)
{

    int sockfd = create_udp_rcv_socket(port_number);
    struct sockaddr_in cliaddr;
    socklen_t len;

    memset(&cliaddr, 0, sizeof(cliaddr));

    len = sizeof(cliaddr); // len is value/result
    nav_msgs::msg::Odometry rcv_odom;

    while (1)
    {
        recvfrom(sockfd, &rcv_odom, sizeof(rcv_odom),
                 MSG_WAITALL, (struct sockaddr *)&cliaddr,
                 &len);
        if (inet_ntoa(cliaddr.sin_addr) != this->my_ip_addr)
        {
            RCLCPP_DEBUG(this->get_logger(), "Received odom from %s : %lf; %lf; %lf", inet_ntoa(cliaddr.sin_addr), rcv_odom.pose.pose.position.x, rcv_odom.pose.pose.position.y, rcv_odom.pose.pose.position.z);
            uint32_t peer_id = inet_lnaof(cliaddr.sin_addr);
            if (this->robot_id == peer_id)
            {
                throw std::runtime_error("Received odom from myself");
            }

            Eigen::Vector3d peer_position_global = {rcv_odom.pose.pose.position.x, rcv_odom.pose.pose.position.y, rcv_odom.pose.pose.position.z};
            Eigen::Vector3d peer_velocity_in_peer_frame = {rcv_odom.twist.twist.linear.x, rcv_odom.twist.twist.linear.y, rcv_odom.twist.twist.linear.z};

            Eigen::Quaterniond peer_orientation = {rcv_odom.pose.pose.orientation.w, rcv_odom.pose.pose.orientation.x, rcv_odom.pose.pose.orientation.y, rcv_odom.pose.pose.orientation.z};
            peer_orientation.normalize();

            this->robots_data[peer_id].position = peer_position_global - this->my_data.position;  // this is peer position relative to our current position
            this->robots_data[peer_id].velocity = peer_orientation * peer_velocity_in_peer_frame; // this is the peer velocity in the global frame
            this->robots_data[peer_id].distance = this->robots_data[peer_id].position.norm();
            this->robots_data[peer_id].timestamp = this->get_clock()->now();
            if (!this->robots_data[peer_id].valid)
            {
                RCLCPP_WARN(this->get_logger(), "\x1b[32m[%d] Regained position of peer %d !\x1b[0m", this->robot_id, peer_id);
            }

            this->robots_data[peer_id].valid = true;
        }
    }
}

/**
 * @brief Send a command to Arm the vehicle
 */
void VATPilot::arm()
{
    // send the arm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, this->robot_id + 1);

    rclcpp::Clock clock;
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 2000, "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void VATPilot::disarm()
{
    // send the disarm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, this->robot_id + 1);

    rclcpp::Clock clock;
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 2000, "Disarm command sent");
}

/**
 * @brief Send a command to set the vehicle to Offboard mode
 */
void VATPilot::engage_offboard_mode()
{
    // send the arm command in a VehicleCommand message. 1 is offboard, 6 is ???
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, this->robot_id + 1);

    rclcpp::Clock clock;
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 2000, "Set Offboard mode command sent");
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

    RCLCPP_DEBUG(this->get_logger(), "My position: %f, %f, %f", this->my_data.position[0], this->my_data.position[1], this->my_data.position[2]);
    RCLCPP_DEBUG(this->get_logger(), "Sending trajectory setpoint to UAV %i: at velocity %f, %f, %f", this->robot_id, msg.velocity[0], msg.velocity[1], msg.velocity[2]);

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
 * @brief Open a UDP socket for reception
 *
 * \param port The port number to listen to
 * \return The socket file descriptor
 */
int VATPilot::create_udp_rcv_socket(uint16_t port)
{
    int sockfd;
    struct sockaddr_in address;

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        return -1;
    }

    memset(&address, 0, sizeof(address));

    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket with the address
    if (bind(sockfd, (const struct sockaddr *)&address,
             sizeof(address)) < 0)
    {
        perror("bind failed");
        return -1;
    }

    return sockfd;
}

/**
 * @brief Broadcasts a nav_msgs/Odometry message to a predefined port and broadcast address
 *
 * The outgoing port number is random
 * \param odom The odometry message to be broadcasted
 */
void VATPilot::broadcast_odom(const nav_msgs::msg::Odometry &odom)
{
    int sock;
    struct sockaddr_in broadcastAddr;
    const char *broadcastIP;
    unsigned short broadcastPort;
    int broadcastPermission;
    int sendOdomLen;

    broadcastIP = BROADCAST_ADDR;
    broadcastPort = PORT;

    if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        fprintf(stderr, "socket error");
        exit(1);
    }

    broadcastPermission = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (void *)&broadcastPermission, sizeof(broadcastPermission)) < 0)
    {
        fprintf(stderr, "setsockopt error");
        exit(1);
    }

    /* Construct local address structure */
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_addr.s_addr = inet_addr(broadcastIP);
    broadcastAddr.sin_port = htons(broadcastPort);

    sendOdomLen = sizeof(odom);

    /* Broadcast odom in datagram to clients */
    if (sendto(sock, &odom, sendOdomLen, 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr)) != sendOdomLen)
    {
        fprintf(stderr, "sendto error");
        exit(1);
    }

    close(sock);
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
    double u = ((robot_pose.x() - segment_start.x()) * (segment_end.x() - segment_start.x()) + (robot_pose.y() - segment_start.y()) * (segment_end.y() - segment_start.y())) / std::pow(calc_distance(segment_start, segment_end), 2);
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

// The computation of the command law is done in the local frame of the robot (we only have relative position estimation), within the Front-Right-Down (FRD) coodrinate system
Eigen::Vector3d VATPilot::compute_flocking_command()
{
    RCLCPP_DEBUG(this->get_logger(), "velocity: %f, %f, %f", this->my_data.velocity[0], this->my_data.velocity[1], this->my_data.velocity[2]);
    Eigen::Vector3d autopropulsion = this->my_data.velocity.normalized() * this->v_flock;
    Eigen::Vector3d repulsion = this->repulsion_term();
    Eigen::Vector3d alignment = this->alignment_term();
    Eigen::Vector3d attraction = this->attraction_term();
    Eigen::Vector3d shill = this->shill_term();
    Eigen::Vector3d secondary_objective_vector = this->secondary_objective_term();

    Eigen::Vector3d desired_velocity = alignment + attraction + repulsion + shill;

    if (this->auto_propulsion_flag)
    {
        desired_velocity += autopropulsion;
    }

    if (this->secondary_objective_flag)
    {
        desired_velocity += secondary_objective_vector;
    }

    // Small propulsion toward front, to apply a forward motion
    // desired_velocity[0] += 0.8;

    RCLCPP_DEBUG(this->get_logger(), "autopropulsion: %f, %f, %f", autopropulsion[0], autopropulsion[1], autopropulsion[2]);
    RCLCPP_DEBUG(this->get_logger(), "repulsion: %f, %f, %f", repulsion[0], repulsion[1], repulsion[2]);
    RCLCPP_DEBUG(this->get_logger(), "alignment: %f, %f, %f", alignment[0], alignment[1], alignment[2]);
    RCLCPP_DEBUG(this->get_logger(), "attraction: %f, %f, %f", attraction[0], attraction[1], attraction[2]);
    RCLCPP_DEBUG(this->get_logger(), "shill: %f, %f, %f", shill[0], shill[1], shill[2]);

    if (desired_velocity.norm() > this->v_max)
    {
        desired_velocity = desired_velocity.normalized() * this->v_max;
    }

    // Small dumb Proportional controller for height (Z is towards UP because we are in ENU convention, )
    if (this->my_data.position[2] > this->target_altitude_ + 0.5)
    {
        desired_velocity[2] = 0.1 * (this->target_altitude_ - this->my_data.position[2]);
    }
    else if (this->my_data.position[2] < this->target_altitude_ - 0.5)
    {
        desired_velocity[2] = 0.1 * (this->target_altitude_ - this->my_data.position[2]);
    }
    else
    {
        desired_velocity[2] = 0.0;
    }

    if (this->rviz_enabled)
    {
        this->rep_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, repulsion, 0, this->my_data.name, "red"));
        this->att_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, attraction, 0, this->my_data.name, "green"));
        this->ali_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, alignment, 0, this->my_data.name, "blue"));
        this->auto_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, autopropulsion, 0, this->my_data.name, "violet"));
        this->shi_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, shill, 0, this->my_data.name, "cyan"));
        this->cmd_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, desired_velocity, 0, this->my_data.name, ""));
    }

    return desired_velocity;
}

// Notations and equations from Vasarhelyi 2018
Eigen::Vector3d VATPilot::alignment_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (auto r : this->robots_data)
    {
        UAV_data robot = r.second;
        if (r.first != this->robot_id && robot.position.hasNaN() == false && robot.distance != NAN && robot.valid)
        {
            double velDiffNorm = (robot.velocity - this->my_data.velocity).norm();
            double v_frictmax = std::max(this->v_frict, SigmoidLin(robot.distance - this->r_0_frict, this->a_frict, this->p_frict));
            if (velDiffNorm > v_frictmax)
            {
                result += this->C_frict * (velDiffNorm - v_frictmax) * (robot.velocity - this->my_data.velocity) / velDiffNorm;
            }
        }
    }
    return result;
}

// From model "Vasarhelyi + Attraction", Bonnefond 2021
Eigen::Vector3d VATPilot::attraction_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (auto r : this->robots_data)
    {
        UAV_data robot = r.second;
        if (r.first != this->robot_id && robot.position.hasNaN() == false && robot.distance != NAN && robot.valid)
        {
            if (robot.distance > this->r_0_att)
            {
                result += this->p_att * (this->r_0_att - robot.distance) * (-robot.position) / robot.distance;
            }
        }
    }
    return result;
}

// Notations and equations from Vasarhelyi 2018
Eigen::Vector3d VATPilot::repulsion_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (auto r : this->robots_data)
    {
        UAV_data robot = r.second;
        if (r.first != this->robot_id && robot.position.hasNaN() == false && robot.valid)
        {
            if (robot.distance < this->r_0_rep && robot.distance > 0.0)
            {
                result += this->p_rep * (this->r_0_rep - robot.distance) * (-robot.position) / robot.distance;
            }
        }
    }
    return result;
}

Eigen::Vector3d VATPilot::shill_term()
{
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for (uint32_t i = 0; i < this->known_raw_obstacles.segments.size(); i++)
    {
        Eigen::Vector3d segment_start = {this->known_raw_obstacles.segments[i].first_point.x, this->known_raw_obstacles.segments[i].first_point.y, this->known_raw_obstacles.segments[i].first_point.z};
        Eigen::Vector3d segment_end = {this->known_raw_obstacles.segments[i].last_point.x, this->known_raw_obstacles.segments[i].last_point.y, this->known_raw_obstacles.segments[i].last_point.z};
        Eigen::Vector3d shill_agent_pose = this->get_shill_agent_position(segment_start, segment_end);
        Eigen::Vector3d shill_agent_velocity = this->v_shill * (-shill_agent_pose).normalized();

        double velDiffNorm = (this->my_data.velocity - shill_agent_velocity).norm();

        double v_shillmax = SigmoidLin(shill_agent_pose.norm() - this->r_0_shill, this->a_shill, this->p_shill);
        if (velDiffNorm > v_shillmax)
        {
            // without the minus sign, the shill component is TOWARDS the obstacle (it is in the opposite direction of the shill agent velocity, which points toward the agent)
            result += -(velDiffNorm - v_shillmax) * (this->my_data.velocity - shill_agent_velocity) / velDiffNorm;
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
        result = this->secondary_objectives[this->robot_id] - this->my_data.position;
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
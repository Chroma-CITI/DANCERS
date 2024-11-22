#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <visualization_msgs/msg/marker.hpp>

#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

// #include <ros_net_sim_interfaces/msg/uwb.hpp>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using namespace std::chrono_literals;



// A transférer dans un maths_utils un jour ! -----------------------


double SigmoidLin(double r, double a, double p){
    if(r <= 0){
        return 0;
    } else if(r*p > 0 && r*p < a/p){
        return r*p;
    } else {
        return std::sqrt(2*a*r - std::pow(a,2)/std::pow(p,2));
    }
}

Eigen::Vector3d ENU_to_NED(Eigen::Vector3d cmd){

    Eigen::Vector3d z_axis(0.0, 0.0, 1.0); // Rotation axis (z-axis)
    Eigen::Quaterniond rot_half_pi_z(Eigen::AngleAxisd(M_PI/2, z_axis));
    rot_half_pi_z.normalize();

    Eigen::Vector3d x_axis(0.0, 1.0, 0.0); // Rotation axis (x-axis)
    Eigen::Quaterniond rot_pi_x(Eigen::AngleAxisd(M_PI, x_axis));
    rot_pi_x.normalize();

    cmd = rot_half_pi_z * cmd;
    cmd = rot_pi_x * cmd;

    return cmd;
}

// ------------------------------------------------------------------

// A neighboor's position expressed in the current UAV frame (i.e. relative positions)
struct UAV_data{
    uint64_t timestamp;
    std::string name;
    uint32_t id;
    Eigen::Vector3d initial_position;
    Eigen::Vector3d position; // The position off a robot in the LOCAL frame, i.e. relative position off the robot in the current robot frame
    Eigen::Vector3d velocity; // The velocity of a robot in the GLOBAL frame. We don't need the origin of the velocity,
    Eigen::Vector3d acceleration;
    double yaw;
    double distance;
};

double calc_distance(Eigen::Vector3d a, Eigen::Vector3d b){
    // return sqrt(pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2) + pow(a(2) - b(2), 2));
    return (a - b).norm();
}

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

visualization_msgs::msg::Marker rviz_arrow(Eigen::Vector3d start, Eigen::Vector3d end, int id, std::string frame_id, std::string color){
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
    if(color == "red"){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else 
    if(color == "green"){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;;
    } else 
    if(color == "blue"){
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    } else 
    if(color == "violet"){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;   
    } else 
    if(color == "cyan"){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;   
    } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;   
    }
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;


    return marker;
}

class VATPilot : public rclcpp::Node
{
    public:
        // Constructor
        VATPilot() : rclcpp::Node("VATPilot"){

            // Declare the config_file ROS2 parameter
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "Path to the YAML configuration file.";
            this->declare_parameter("config_file", "", param_desc);

            // Fetch the parameter path to the config file using ros2 parameter
            std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

            // Verify existence of the config file, abort if not found
            if(access(config_file_path.c_str(), F_OK) != 0){
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

            // Initialize the peers information
            for(int i=1 ; i <= num_robots ; i++){
                std::string robot_name = config["robots_model"].as<std::string>()+"_"+std::to_string(i); //e.g. x500_1, x500_2, etc.
                UAV_data data;
                data.name = robot_name;
                data.id = i;
                this->robots_data.push_back(data);
                RCLCPP_DEBUG(this->get_logger(), "Tracked robot: %s", robot_name.c_str());
            }
            this->my_data = UAV_data();

            // Declare the robot_id ROS2 parameter
            param_desc.description = "ID of the robot. This is the system id of each robot (instance number + 1)";
            this->declare_parameter("robot_id", 1, param_desc);
            this->robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint32_t>();
            if(robot_id <= 0){
                RCLCPP_FATAL(this->get_logger(), "Wrong robot ID ! Robot IDs must start at 1. Received: %i", robot_id);
            }
            this->my_data.name = config["robots_model"].as<std::string>()+"_"+std::to_string(this->robot_id);
            this->my_data.id = this->robot_id;

            this->tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            param_desc.description = "If true, use the ground truth position from the Gazebo simulation. If false, use the SITL sensor measurements.";
            this->declare_parameter("use_gz_positions", false, param_desc);
            this->use_gz_positions = this->get_parameter("use_gz_positions").get_parameter_value().get<bool>();
            RCLCPP_DEBUG(this->get_logger(), "Using ground truth positions from Gazebo");

            // Prepare the QoS that we will assign to the publishers / subscribers  
            auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
            
            // Create subscribers
            this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "/px4_"+std::to_string(robot_id)+"/fmu/out/vehicle_status",
                qos_sub, 
                std::bind(&VATPilot::vehicle_status_clbk, this, _1)
            );
            
            for(uint32_t i = 1; i <= num_robots; i++){
                if(i != robot_id){
                    this->neighbors_odom_subs.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
                        config["robots_model"].as<std::string>()+"_"+std::to_string(i)+"/odometry", 
                        qos_sub, 
                        [this, i](const nav_msgs::msg::Odometry &odom){VATPilot::neighbors_odom_clbk(odom, i);}
                    ));
                    RCLCPP_DEBUG(this->get_logger(), "Subscribed to robot %d /odometry topic.", i);
                }
            }
            this->my_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                this->my_data.name+"/odometry",
                qos_sub,
                [this](const nav_msgs::msg::Odometry &odom){VATPilot::my_odom_clbk(odom);}
            );

            this->raw_obstacle_sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
                this->my_data.name+"/raw_obstacles",
                qos_sub,
                std::bind(&VATPilot::raw_obstacle_clbk, this, _1)
            );

            // Create publishers
            this->offboard_control_mode_publisher_ =
                    this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_"+std::to_string(robot_id)+"/fmu/in/offboard_control_mode", qos_pub);
            this->trajectory_setpoint_publisher_ =
                    this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_"+std::to_string(robot_id)+"/fmu/in/trajectory_setpoint", qos_pub);
            this->vehicle_command_publisher_ = 
                    this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_"+std::to_string(robot_id)+"/fmu/in/vehicle_command", qos_pub);
            
            this->cmd_marker_publisher_ =
                    this->create_publisher<visualization_msgs::msg::Marker>("/px4_"+std::to_string(robot_id)+"/cmd_marker", 10);
            this->att_marker_publisher_ =
                    this->create_publisher<visualization_msgs::msg::Marker>("/px4_"+std::to_string(robot_id)+"/att_cmd_marker", 10);
            this->rep_marker_publisher_ =
                    this->create_publisher<visualization_msgs::msg::Marker>("/px4_"+std::to_string(robot_id)+"/rep_cmd_marker", 10);
            this->ali_marker_publisher_ =
                    this->create_publisher<visualization_msgs::msg::Marker>("/px4_"+std::to_string(robot_id)+"/ali_cmd_marker", 10);
            this->auto_marker_publisher_ =
                    this->create_publisher<visualization_msgs::msg::Marker>("/px4_"+std::to_string(robot_id)+"/auto_cmd_marker", 10);
            this->shi_marker_publisher_ =
                    this->create_publisher<visualization_msgs::msg::Marker>("/px4_"+std::to_string(robot_id)+"/shi_cmd_marker", 10);


            this->wait_time = 5000000; // 5s in us
            this->wait_counter = 0;
		    this->offboard_setpoint_counter_ = 0;
            this->timer_period = 100000us; // 100 ms

            this->peer_info_timeout = 1000000000; // 1s in ns

            RCLCPP_INFO(this->get_logger(), "VATPilot initialized");

            // start publisher timer
            timer_ = rclcpp::create_timer(this, this->get_clock(), this->timer_period, std::bind(&VATPilot::cmd_loop_clbk, this));
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
        uint64_t wait_time;
        uint64_t wait_counter;
        uint64_t peer_info_timeout; //!< time in ns for which we consider a pose information from a peer "valid". If the info is older than that, erase it
        uint32_t robot_id;
        uint16_t num_robots;
        std::vector<UAV_data> robots_data;
        UAV_data my_data;
        // std::vector<Msg_Uwb> msgs_uwb;
        sensor_msgs::msg::LaserScan lidar_msg;
        std::mutex lidar_msg_mutex;
        obstacle_detector::msg::Obstacles known_raw_obstacles;
        std::mutex known_raw_obstacles_mutex;
        // std::vector<std::unique_ptr<std::mutex>> robots_data_mutexes;
        bool use_gz_positions;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


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

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbors_odom_subs;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_odom_sub;
        rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr raw_obstacle_sub_;

        // publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr att_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rep_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ali_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr auto_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shi_marker_publisher_;

        // callbacks
        void vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg)
        {
            this->arming_state = msg.arming_state;
            this->nav_state = msg.nav_state;
            // RCLCPP_INFO(this->get_logger(), "ArmingState: %i\nNavState: %i", this->arming_state, this->nav_state);
        }
        void raw_obstacle_clbk(const obstacle_detector::msg::Obstacles & data);
        void lidar_clbk(const sensor_msgs::msg::LaserScan & data);
        void gz_positions_clbk(const geometry_msgs::msg::PoseArray & data);
        void neighbors_odom_clbk(const nav_msgs::msg::Odometry & odom, int peer_id);
        void my_odom_clbk(const nav_msgs::msg::Odometry & odom);
        void cmd_loop_clbk();

        // commands
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint(Eigen::Vector3d target, double yaw = M_PI/2);
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);
        Eigen::Vector3d get_shill_agent_position(Eigen::Vector3d start, Eigen::Vector3d end);
        // void update_neighborhood();
        Eigen::Vector3d compute_cmd();

        // flocking functions
        Eigen::Vector3d repulsion_term();
        Eigen::Vector3d attraction_term();
        Eigen::Vector3d alignment_term();
        Eigen::Vector3d shill_term();

};

void VATPilot::cmd_loop_clbk(){
    RCLCPP_INFO(this->get_logger(), "Entered cmd loop drone %d", this->robot_id);
    if(this->offboard_setpoint_counter_ <= 10 && this->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
        // Change to Offboard mode after 10 setpoints (1s)
        this->engage_offboard_mode();

        // Arm the vehicle
        this->arm();
    }


    // CMD must be expressed in NED (North East Down) frame.
    Eigen::Vector3d cmd = {0.0, 0.0, 0.0};
    if(this->offboard_setpoint_counter_ <= 70){
        cmd = {0.0, 0.0, -4.0};
    } else {
        cmd = compute_cmd();
    }
    this->publish_offboard_control_mode();
    if(this->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
        this->publish_trajectory_setpoint(cmd);
        // std::atan2(cmd[0], cmd[1])
    }

    this->offboard_setpoint_counter_++;
}

void VATPilot::raw_obstacle_clbk(const obstacle_detector::msg::Obstacles & _msg){
    this->known_raw_obstacles = _msg;
    RCLCPP_DEBUG(this->get_logger(), "Saved obstacles data");
}


/*
REMARK: The VehicleLocalPosition message also has ref_lat, ref_lon, ref_alt that are the reference point where the EKF2 was started (local NED frame origin)
We could use this information to infer the initial_position of the neighbour !
*/
// Callback called at each Odometry message received from neighbors
// Saves the position and velocity information in the UAV_data structure, the position is in the current UAV frame
void VATPilot::neighbors_odom_clbk(const nav_msgs::msg::Odometry & odom, int peer_id){
    geometry_msgs::msg::TransformStamped t;
    int peer_index = peer_id-1;
    std::string fromFrameRel = this->robots_data[peer_index].name;
    std::string toFrameRel = this->my_data.name;
    try {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & e){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", fromFrameRel.c_str(), toFrameRel.c_str(), e.what());
    }
    Eigen::Quaterniond quat = {t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z};
    quat.normalize();
    Eigen::Vector3d velocity_in_neighbor_frame = {odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z};
    Eigen::Vector3d velocity_in_local_frame = quat * velocity_in_neighbor_frame;
    this->robots_data[peer_index].position = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};
    this->robots_data[peer_index].velocity = velocity_in_local_frame; 
    this->robots_data[peer_index].distance = this->robots_data[peer_index].position.norm();
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i position: %f, %f, %f", peer_index, this->robots_data[peer_index].position[0], this->robots_data[peer_index].position[1], this->robots_data[peer_index].position[2]);
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i velocity: %f, %f, %f", peer_index, this->robots_data[peer_index].velocity[0], this->robots_data[peer_index].velocity[1], this->robots_data[peer_index].velocity[2]);
    // RCLCPP_DEBUG(this->get_logger(), "Saved UAV %i distance: %f", peer_index, this->robots_data[peer_index].distance);
}

void VATPilot::my_odom_clbk(const nav_msgs::msg::Odometry & odom){
    this->my_data.position = {odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
    this->my_data.velocity = {odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z};
    this->my_data.distance = NAN;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void VATPilot::arm()
{
    // send the arm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, this->robot_id+1);

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void VATPilot::disarm()
{
    // send the disarm command in a VehicleCommand message. the third parameter is the default parameter, it has no impact
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, this->robot_id+1);

	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Send a command to set the vehicle to Offboard mode
 */
void VATPilot::engage_offboard_mode()
{
    // send the arm command in a VehicleCommand message. 1 is offboard, 6 is ???
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, this->robot_id+1);

    RCLCPP_INFO(this->get_logger(), "Set Offboard mode command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void VATPilot::publish_offboard_control_mode(){
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
void VATPilot::publish_trajectory_setpoint(Eigen::Vector3d target, double yaw){
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);

    msg.position[0] = NAN;
    msg.position[1] = NAN;
    msg.position[2] = NAN; // The Z axis is toward the ground

    // We obtain undefined behavior if the unused fields are not explicitly set to NaN
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

    RCLCPP_DEBUG(this->get_logger(), "My position: %f, %f, %f", this->robots_data[this->robot_id-1].position[0], this->robots_data[this->robot_id-1].position[1], this->robots_data[this->robot_id-1].position[2]);

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
Eigen::Vector3d VATPilot::get_shill_agent_position(Eigen::Vector3d segment_start, Eigen::Vector3d segment_end){
    Eigen::Vector3d agent_pose;
    Eigen::Vector3d robot_pose = {0.0, 0.0, 0.0}; // The obstacle is in the lidar's frame, so we set the robot pose to 0
    double u = ((robot_pose.x() - segment_start.x()) * (segment_end.x() - segment_start.x()) + (robot_pose.y() - segment_start.y()) * (segment_end.y() - segment_start.y()))
        / std::pow(calc_distance(segment_start, segment_end), 2);
    if(u >= 1.0){
        u = 1.0;
    }
    if(u <= 0.0){
        u = 0.0;
    }
    agent_pose[0] = segment_start.x() + u * (segment_end.x() - segment_start.x());
    agent_pose[1] = segment_start.y() + u * (segment_end.y() - segment_start.y());
    agent_pose[2] = robot_pose[2];

    return agent_pose;
}   

// void VATPilot::update_neighborhood(){
//     for(UAV_data data : robots_data){
//         if(this->get_clock()->now().nanoseconds() - data.timestamp > this->peer_info_timeout){
//             data.position = {NAN, NAN, NAN};
//             data.velocity = {NAN, NAN, NAN};
//             data.acceleration = {NAN, NAN, NAN};
//             data.yaw = NAN;
//             RCLCPP_DEBUG(this->get_logger(), "Peer information is too old to be considered, wiping data for peer %s", data.name.c_str());
//         }
//     } 
// }


// The computation of the command law is done in the local frame of the robot (we only have relative position estimation), within the East-North-Up (ENU) 
// coordinate system. At the end of this method, we translate the command in the NED coordinate system (FRD), which is the one used by the autopilot.
Eigen::Vector3d VATPilot::compute_cmd()
{
    RCLCPP_DEBUG(this->get_logger(), "velocity: %f, %f, %f", this->robots_data[this->robot_id-1].velocity[0], this->robots_data[this->robot_id-1].velocity[1], this->robots_data[this->robot_id-1].velocity[2]);    
    Eigen::Vector3d autopropulsion = this->my_data.velocity.normalized()*this->v_flock;
    Eigen::Vector3d repulsion = this->repulsion_term();
    Eigen::Vector3d alignment = this->alignment_term();
    Eigen::Vector3d attraction = this->attraction_term();
    Eigen::Vector3d shill = this->shill_term();

    RCLCPP_DEBUG(this->get_logger(), "autopropulsion: %f, %f, %f", autopropulsion[0], autopropulsion[1], autopropulsion[2]);
    RCLCPP_DEBUG(this->get_logger(), "repulsion: %f, %f, %f", repulsion[0], repulsion[1], repulsion[2]);
    RCLCPP_DEBUG(this->get_logger(), "alignment: %f, %f, %f", alignment[0], alignment[1], alignment[2]);
    RCLCPP_DEBUG(this->get_logger(), "attraction: %f, %f, %f", attraction[0], attraction[1], attraction[2]);
    RCLCPP_DEBUG(this->get_logger(), "shill: %f, %f, %f", shill[0], shill[1], shill[2]);


    this->rep_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, repulsion, 0, this->my_data.name, "red"));
    this->att_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, attraction, 0, this->my_data.name, "green"));
    this->ali_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, alignment, 0, this->my_data.name, "blue"));
    this->auto_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, autopropulsion, 0, this->my_data.name, "violet"));
    this->shi_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, shill, 0, this->my_data.name, "cyan"));

    Eigen::Vector3d desired_velocity = alignment + attraction + repulsion + autopropulsion + shill;

    // Small propulsion toward front, to apply a forward motion
    desired_velocity[0] += 0.3;

    this->cmd_marker_publisher_->publish(rviz_arrow({0.0, 0.0, 0.0}, desired_velocity, 0, this->my_data.name, "red"));

    if(desired_velocity.norm() > this->v_max){
        desired_velocity = desired_velocity.normalized()*this->v_max;
    }

    // Small dumb Proportional controller for height (Z is towards UP because we are in ENU convention, )
    if(this->my_data.position[2] > 15.0){
        desired_velocity[2] = 0.1*(this->my_data.position[2] - 15.0);
    } else if(this->my_data.position[2] < 15.0){
        desired_velocity[2] = -0.1*(this->my_data.position[2] - 15.0);
    } else {
        desired_velocity[2] = 0.0;
    }

    return ENU_to_NED(desired_velocity);
}

// Notations and equations from Vasarhelyi 2018
Eigen::Vector3d VATPilot::alignment_term(){
    Eigen::Vector3d result = {0.0 , 0.0, 0.0};
    for(UAV_data robot : this->robots_data){
        if(robot.id != this->robot_id && robot.position.hasNaN() == false && robot.distance != NAN){
            double velDiffNorm = (robot.velocity - this->my_data.velocity).norm();
            double v_frictmax = std::max(this->v_frict, SigmoidLin(robot.distance - this->r_0_frict, this->a_frict, this->p_frict));
            // RCLCPP_INFO(this->get_logger(), "VelDiffNorm = %f", velDiffNorm);
            // RCLCPP_INFO(this->get_logger(), "v_frictmax = %f", v_frictmax);
            if(velDiffNorm > v_frictmax){
                result += this->C_frict*(velDiffNorm-v_frictmax)*(robot.velocity - this->my_data.velocity)/velDiffNorm;
            }
        }
    }
    return result;
}

// From model "Vasarhelyi + Attraction", Bonnefond 2021
Eigen::Vector3d VATPilot::attraction_term(){
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for(UAV_data robot : this->robots_data){
        if(robot.id != this->robot_id  && robot.position.hasNaN() == false && robot.distance != NAN){
            if(robot.distance > this->r_0_att){
                result += this->p_att*(this->r_0_att - robot.distance)*(-robot.position)/robot.distance;
            }
        }
    }
    return result;
}

// Notations and equations from Vasarhelyi 2018
Eigen::Vector3d VATPilot::repulsion_term(){
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for(UAV_data robot : this->robots_data){
        if(robot.id != this->robot_id && robot.position.hasNaN() == false){
            if(robot.distance < this->r_0_rep && robot.distance > 0.0){
                result += this->p_rep*(this->r_0_rep - robot.distance)*(-robot.position)/robot.distance;
            }
        }
    }
    return result;
}



Eigen::Vector3d VATPilot::shill_term(){
    Eigen::Vector3d result = {0.0, 0.0, 0.0};
    for(uint32_t i=0 ; i < this->known_raw_obstacles.segments.size() ; i++){
        Eigen::Vector3d segment_start = {this->known_raw_obstacles.segments[i].first_point.x, this->known_raw_obstacles.segments[i].first_point.y, this->known_raw_obstacles.segments[i].first_point.z};
        Eigen::Vector3d segment_end = {this->known_raw_obstacles.segments[i].last_point.x, this->known_raw_obstacles.segments[i].last_point.y, this->known_raw_obstacles.segments[i].last_point.z};
        Eigen::Vector3d shill_agent_pose = this->get_shill_agent_position(segment_start, segment_end);
        Eigen::Vector3d shill_agent_velocity = this->v_shill * (-shill_agent_pose).normalized();

        double velDiffNorm = (this->my_data.velocity - shill_agent_velocity).norm();

        double v_shillmax = SigmoidLin(shill_agent_pose.norm() - this->r_0_shill, this->a_shill, this->p_shill);
        if(velDiffNorm > v_shillmax){
            // without the minus sign, the shill component is TOWARDS the obstacle (it is in the opposite direction of the shill agent velocity, which points toward the agent)
            result += -(velDiffNorm-v_shillmax)*(this->my_data.velocity - shill_agent_velocity)/velDiffNorm;
        }
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
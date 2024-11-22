#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

struct UAV_data{
    Eigen::Vector3d initial_position;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double yaw;
};

double calc_distance(Eigen::Vector3d a, Eigen::Vector3d b){
    // return sqrt(pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2) + pow(a(2) - b(2), 2));
    return (a - b).norm();
}

class ControlTest : public rclcpp::Node
{
    public:
        // Constructor
        ControlTest() : rclcpp::Node("ControlTest"){

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

            num_robots = config["robots_number"].as<uint16_t>();

            // initialize empty peers information
            for(int i = 0; i < num_robots; i++){
                this->robots_data.push_back(UAV_data());
                // this->robots_data_mutexes.push_back(std::make_unique<std::mutex>());
            }


            // Declare the robot_id ROS2 parameter
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
                            "/px4_"+std::to_string(robot_id)+"/fmu/out/vehicle_status",
                            qos_sub, 
                            std::bind(&ControlTest::vehicle_status_clbk, this, _1)
            );

            // Subscribe to the position of all the robots
            std::string topic_name;
            for(int i = 0; i < num_robots; i++){
                topic_name = "/px4_"+std::to_string(i+1)+"/fmu/out/vehicle_local_position";
                this->peer_positions_subs_.push_back(this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                                            topic_name, 
                                            qos_sub,
                                            [this, i](const px4_msgs::msg::VehicleLocalPosition &pose){this->pose_clbk(pose, i);}
                ));
                RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic_name.c_str());
            }


            // Create publishers
            this->offboard_control_mode_publisher_ =
                    this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_"+std::to_string(robot_id)+"/fmu/in/offboard_control_mode", qos_pub);
            this->trajectory_setpoint_publisher_ =
                    this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_"+std::to_string(robot_id)+"/fmu/in/trajectory_setpoint", qos_pub);
            this->vehicle_command_publisher_ = 
                    this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_"+std::to_string(robot_id)+"/fmu/in/vehicle_command", qos_pub);

            this->wait_time = 5000000; // 5s in us
            this->wait_counter = 0;
		    this->offboard_setpoint_counter_ = 0;
            this->radius = 10.0; // m
            this->theta = 0.0; // rad
            this->omega = 0.5; // rad.s-1
            this->timer_period = 100000us; // 100 ms

            double distance_init = 10.0; // meters
            int num_columns = 3;
            double x_offset = 0.0;
            double y_offset = -((num_columns-1) * distance_init) / 2.0 ;

            // initial value of velocity
            this->desired_velocity = {0.0, 0.0, 0.0};
            // initial value of position
            this->desired_position = {0.0, 0.0, 0.0};

            // Starting in a grid
            this->starting_position = {
                x_offset + ((robot_id-1) / num_columns) * distance_init,
                y_offset + distance_init * (robot_id-1 % num_columns),
                0.0
            };
            
            // flocking parameters
            this->separation_radius = 10.0;
            this->cohesion_radius = 200.0;
            this->alignment_radius = 200.0;
            this->sep_factor = 1;
            this->cohesion_factor = 0.2;
            this->alignment_factor = 0.8;
            this->max_accel = 0.3;

            auto cmdloop_callback = [this]() -> void {
                if(this->offboard_setpoint_counter_ == 100){
                    // Change to Offboard mode after 50 setpoints (1s)
                    this->engage_offboard_mode();

                    // Arm the vehicle
                    // this->arm();

                }

                this->publish_offboard_control_mode();
                this->publish_trajectory_setpoint();

                // stop the counter after reaching 11
                if (this->offboard_setpoint_counter_ < 101) {
				    this->offboard_setpoint_counter_++;
                }
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
        uint64_t wait_time;
        uint64_t wait_counter;
        double radius;
        double theta;
        double omega;
        double separation_radius;
        double cohesion_radius;
        double alignment_radius;
        double alignment_factor;
        double sep_factor;
        double cohesion_factor;
        double max_accel;
        std::string node_namespace;
        uint8_t robot_id;
        uint16_t num_robots;
        std::vector<UAV_data> robots_data;
        // std::vector<std::unique_ptr<std::mutex>> robots_data_mutexes;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> peer_positions_subs_;

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
        void pose_clbk(const px4_msgs::msg::VehicleLocalPosition & pose, int robot_id);

        // commands
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t system_id = 0);

        // flocking functions
        Eigen::Vector3d separation_term();
        Eigen::Vector3d cohesion_term();
        Eigen::Vector3d alignment_term();

        Eigen::Vector3d starting_position;
        Eigen::Vector3d desired_velocity;
        Eigen::Vector3d desired_position;
};

// Callback called at each Pose message from other robots
void ControlTest::pose_clbk(const px4_msgs::msg::VehicleLocalPosition & pose, int peer_id){

    // First we take the position relative to initialization of the EFK2, then we translate it into position relative to the origin using the starting position
    this->robots_data[peer_id].position = {pose.x, pose.y, pose.z};
    this->robots_data[peer_id].position += this->starting_position;

    this->robots_data[peer_id].velocity = {pose.vx, pose.vy, pose.vz};

    this->robots_data[peer_id].acceleration = {pose.ax, pose.ay, pose.az};

    this->robots_data[peer_id].yaw = pose.heading;

    // RCLCPP_INFO(this->get_logger(), "Received local position from UAV %i", peer_id);
}



// =================================

/**
 * @brief Calculates the separation term for the flocking algorithm
 * 
 * The separation term is a force driving the robot away from its neighbours. Take the (normalized) vector from the 
 * neighbour to the robot, multiply it by the inverse of the distance from this neighbour and scale it with the separation contant
 * @returns Eigen::Vector3d The separation term (force)
*/
Eigen::Vector3d ControlTest::separation_term()
{
    Eigen::Vector3d separation_term = {0.0, 0.0, 0.0};
    int total = 0;
    for(int i = 0; i < this->num_robots; i++){
        if(i != this->robot_id - 1){
            double distance = calc_distance(this->robots_data[i].position, this->robots_data[this->robot_id-1].position);
            if(distance < this->separation_radius && distance != 0){
                Eigen::Vector3d diff = this->robots_data[i].position - this->robots_data[this->robot_id-1].position;
                diff.normalize();
                diff /= distance;
                diff *= this->sep_factor;
                separation_term += diff;
                total++;
            }
        }
    }
    if(total > 0){
        separation_term /= total;
    }
    return separation_term;
}

/**
 * @brief Calculates the alignment term for the flocking algorithm
 * 
 * The alignment term is a force driving the robot towards the average velocity of its neighbours
 * @returns Eigen::Vector3d The alignment term (force)
*/
Eigen::Vector3d ControlTest::alignment_term(){
    Eigen::Vector3d alignment_term = {0.0, 0.0, 0.0};
    int total = 0;
    for(int i = 0; i < this->num_robots; i++){
        if(i != this->robot_id - 1){
            double distance = calc_distance(this->robots_data[i].position, this->robots_data[this->robot_id-1].position);
            if(distance < this->alignment_radius){
                alignment_term += robots_data[i].velocity;
                total++;
            }
        }
    }
    if(total > 0){
        alignment_term /= total;
        alignment_term -= this->robots_data[this->robot_id-1].velocity;
        alignment_term *= this->alignment_factor;
    }
    return alignment_term;
}

/**
 * @brief Calculates the cohesion term for the flocking algorithm
 * 
 * The cohesion term is a force driving the robot towards the average position of its neighbours
 * @returns Eigen::Vector3d The cohesion term (force)
*/
Eigen::Vector3d ControlTest::cohesion_term(){
    Eigen::Vector3d cohesion_term = {0.0, 0.0, 0.0};
    int total = 0;
    for(int i = 0; i < this->num_robots; i++){
        if(i != this->robot_id - 1){
            double distance = calc_distance(this->robots_data[i].position, this->robots_data[this->robot_id-1].position);
            if(distance < this->cohesion_radius){
                cohesion_term += robots_data[i].position;
                total++;
            }
        }
    }
    if(total > 0){
        cohesion_term /= total;
        cohesion_term -= this->robots_data[this->robot_id-1].position;
        cohesion_term *= this->cohesion_factor;
    }
    return cohesion_term;
}


// =================================



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
 * @brief Send a command to set the vehicle to Offboard mode
 */
void ControlTest::engage_offboard_mode()
{
    // send the arm command in a VehicleCommand message. 1 is offboard, 6 is ???
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, this->robot_id);

    RCLCPP_INFO(this->get_logger(), "Set Offboard mode command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void ControlTest::publish_offboard_control_mode(){
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
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void ControlTest::publish_trajectory_setpoint(){
    if(this->wait_counter < this->wait_time){
        this->wait_counter += this->timer_period.count();
        RCLCPP_INFO(this->get_logger(), "Waiting...");
        px4_msgs::msg::TrajectorySetpoint msg{};
	    msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
        this->trajectory_setpoint_publisher_->publish(msg);
        return;
    }

    Eigen::Vector3d separation = this->separation_term();
    Eigen::Vector3d alignment = this->alignment_term();
    Eigen::Vector3d cohesion = this->cohesion_term();
    Eigen::Vector3d flock_accel = separation + alignment + cohesion;


    Eigen::Vector3d auto_propulsion_accel(0.0, 0.1, 0.0);

    flock_accel += auto_propulsion_accel;

    if(flock_accel.norm() > this->max_accel){
        RCLCPP_INFO(this->get_logger(), "Acceleration norm is: %f, reducing it to %f", flock_accel.norm(),  this->max_accel);
        flock_accel = (flock_accel * this->max_accel)/flock_accel.norm();
    }

    // Velocity = velocity + acceleration * dt
    this->desired_velocity = this->desired_velocity + flock_accel * (this->timer_period.count()/1000000.0);

    // Position = position + velocity * dt
    this->desired_position = this->desired_position + this->desired_velocity * (this->timer_period.count()/1000000.0);

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);

    float z_command = this->starting_position[2] + this->desired_position[2];
    // prevent crashes on the ground
    if(z_command < 10.0){
        z_command = 10.0;
    }

    msg.position[0] = this->starting_position[0] + this->desired_position[0];
    msg.position[1] = this->starting_position[1] + this->desired_position[1];
    msg.position[2] = -10; // The Z axis is toward the ground

    msg.velocity[0] = NAN;
    msg.velocity[1] = NAN;
    msg.velocity[2] = NAN;

    msg.acceleration[0] = NAN;
    msg.acceleration[1] = NAN;
    msg.acceleration[2] = NAN;

    msg.jerk[0] = NAN;
    msg.jerk[1] = NAN;
    msg.jerk[2] = NAN;

    msg.yaw = M_PI/2;

    // RCLCPP_INFO(this->get_logger(), "My position: %f, %f, %f", this->robots_data[this->robot_id-1].position[0], this->robots_data[this->robot_id-1].position[1], this->robots_data[this->robot_id-1].position[2]);

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
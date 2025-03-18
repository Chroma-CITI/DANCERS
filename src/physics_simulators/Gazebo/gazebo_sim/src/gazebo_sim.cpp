#include <iostream>
#include <fstream>
#include <optional>
#include <string>

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"

#include "udp_tcp_socket.hpp"

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include "gz/sim/ServerConfig.hh"
#include <gz/sim/components.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "protobuf_msgs/physics_update.pb.h"
#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

// ROS2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <dancers_msgs/msg/neighbor.hpp>
#include <dancers_msgs/msg/neighbor_array.hpp>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <yaml-cpp/yaml.h>

/**
 * \brief Compresses a string with the zip protocol.
 *
 * \param data The string to compress.
 * \return The compressed string.
 */
static std::string gzip_compress(const std::string &data)
{
    std::stringstream compressed;
    std::stringstream origin(data);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_compressor());
    in.push(origin);
    boost::iostreams::copy(in, compressed);

    return compressed.str();
}

/**
 * \brief Decompress a string with the zip protocol.
 *
 * \param data The compressed string to decompress.
 * \return The decompressed string.
 */
static std::string gzip_decompress(const std::string &data)
{
    std::stringstream compressed(data);
    std::stringstream decompressed;

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(compressed);
    boost::iostreams::copy(in, decompressed);

    return decompressed.str();
}

/**
 * \brief Generate a protobuf_msgs/ChannelData protobuf message holding information to be passed from the robotics to the network simulator.
 *
 * \param robot_poses A map of the robot's name and its absolute Pose in Gazebo.
 * \return A string-serialize [RobotsPositions] protobuf message holding positions of the robots.
 */
std::string generate_robots_positions(const std::map<std::string, gz::msgs::Pose> robot_poses)
{
    robots_positions_proto::RobotsPositions RobotsPositions_msg;

    for (const auto &robot : robot_poses)
    {
        gz::msgs::Vector3d pos = robot.second.position();
        gz::msgs::Quaternion quat = robot.second.orientation();
        robots_positions_proto::RobotPose *RobotPose_msg = RobotsPositions_msg.add_robot_pose();
        RobotPose_msg->set_x(pos.x());
        RobotPose_msg->set_y(pos.y());
        RobotPose_msg->set_z(pos.z());
        RobotPose_msg->set_qw(quat.w());
        RobotPose_msg->set_qx(quat.x());
        RobotPose_msg->set_qy(quat.y());
        RobotPose_msg->set_qz(quat.z());
    }

    std::string RobotsPositions_string;
    RobotsPositions_msg.SerializeToString(&RobotsPositions_string);

    return RobotsPositions_string;
}

/**
 * \brief Generates the final protobuf message of protobuf_msgs/PhysicsUpdate.
 *
 * Usually, this function will be passed an empty [PhysicsUpdate] protobuf message with message-type BEGIN.
 * It will fill the message with the (compressed) robots_positions for the current simulation window,
 * change the message-type to END, string-serialize the protobuf message and return it.
 *
 * \param robots_positions A std::string which is a string-serialized ChannelData protobuf message.
 * \param physics_update_msg A protobuf message of type PhysicsUpdate. Usually it is an empty message with type BEGIN, this function will fill the message and
 */
std::string GenerateResponseProtobuf(std::string robots_positions)
{
    // Change message's type to END
    physics_update_proto::PhysicsUpdate physics_update_msg;
    physics_update_msg.set_msg_type(physics_update_proto::PhysicsUpdate::END);
    // Fill the channel_data field with the compressed data from the physics simulation
    physics_update_msg.set_robots_positions(gzip_compress(robots_positions));
    // Transform the response [protobuf] --> [string]
    std::string str_response;
    physics_update_msg.SerializeToString(&str_response);
    str_response = gzip_compress(str_response);

    return str_response;
}

/**
 * \brief Prints the result of a call to a Gazebo Service (just for code compactness)
 *
 * \param service The name of the service
 * \param executed If the service timed out
 * \param result If the call to the service failed
 */
void check_service_results(std::string service, bool executed, bool result)
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

bool check_network_setup(std::string ip_address)
{

    struct ifaddrs *ifAddrStruct = NULL;
    struct ifaddrs *ifa = NULL;
    void *tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (!ifa->ifa_addr)
        {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET)
        { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            if (ip_address == addressBuffer)
            {
                return true;
            }
        }
        else if (ifa->ifa_addr->sa_family == AF_INET6)
        { // check it is IP6
            // is a valid IP6 Address
            tmpAddrPtr = &((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
            char addressBuffer[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            if (ip_address == addressBuffer)
            {
                return true;
            }
        }
    }
    if (ifAddrStruct != NULL)
        freeifaddrs(ifAddrStruct);
    return false;
}

class RoboticsCoordinator : public rclcpp::Node
{
public:
    RoboticsCoordinator() : Node("robotics_coordinator")
    {
        // Declare two parameters for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        param_desc.description = "Executes in co-simulation or ns-3 only.";
        this->declare_parameter("cosim_mode", true, param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        bool cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();
        bool verbose = this->get_parameter("verbose").get_parameter_value().get<bool>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s\nA config file must be given in the launch file.", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Get the path to the ROS_WS, it is mandatory to run
        if (getenv("ROS_WS") == NULL)
        {
            RCLCPP_FATAL(this->get_logger(), "ROS_WS environment variable not set, aborting.");
            exit(EXIT_FAILURE);
        }
        else
        {
            this->ros_ws_path = getenv("ROS_WS");
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        // ========================= COMPUTATION TIME SAVING =========================
        this->save_compute_time = config["save_compute_time"].as<bool>();

        if (this->save_compute_time)
        {

            // Create a folder based on the experience name, if not existant already
            std::string experience_name = config["experience_name"].as<std::string>();
            if (boost::filesystem::create_directories(this->ros_ws_path + "/data/" + experience_name))
            {
                RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", experience_name.c_str());
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Using existing data folder for this experiment");
            }

            // Define the output file name, based on the existing files in the experience folder (incremental)
            std::string temp_path;
            int i = 1;
            while (m_computation_time_file.empty())
            {
                temp_path = this->ros_ws_path + "/data/" + experience_name + "/physics" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    m_computation_time_file = temp_path;
                }
            }

            // initialize the output file with headers
            this->compute_time_output.open(m_computation_time_file.c_str(), std::ios::out);
            this->compute_time_output << "Time[us]"
                    << std::endl;
            this->compute_time_output.close();
        }

        // Store the names of the robots in gazebo in a vector
        std::vector<std::string> gazebo_models;
        for (int i = 1; i <= config["robots_number"].as<int>(); i++)
        {
            std::string robot_name = config["robots_model"].as<std::string>() + "_" + std::to_string(i); // e.g. x500_1, x500_2, etc.
            gazebo_models.push_back(robot_name);
            RCLCPP_INFO(this->get_logger(), "Tracked robot: %s", robot_name.c_str());
        }

        // ========================= ROS2 PUBLISHERS =========================
        // Prepare the QoS that we will assign to the publishers / subscribers
        auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();


        for (std::string robot_name : gazebo_models)
        {
            this->neighbors_pub_vector_.push_back(this->create_publisher<dancers_msgs::msg::NeighborArray>(robot_name + "/neighbors", qos_pub));
        }

        // ========================= GAZEBO SERVER CONFIGURATION =========================

        // Set the PX4_GZ_MODEL environment variable, used by the PX4 SITL to know which model we use
        // It is not a good practice to interact with the user's .bashrc, but well we do it anyway 
        const char* home = std::getenv("HOME");
        if (!home) {
            std::cerr << "HOME environment variable not set." << std::endl;
            exit(EXIT_FAILURE);
        }
        // Construct the path to the .bashrc file
        std::string bashrc_path = std::string(home) + "/.bashrc";

        // Open the .bashrc file and a temporary file
        std::ifstream file_in(bashrc_path);
        std::ofstream file_out("temp.txt");
        if (!file_in.is_open() || !file_out.is_open()) {
            std::cerr << "Failed to open .bashrc file." << std::endl;
            exit(EXIT_FAILURE);
        }
        // Look for a line starting with export PX4_GZ_MODEL and replace it with the right model
        std::string line;
        bool found_PX4_GZ_MODEL = false;
        bool found_DANCERS_NUM_ROBOTS = false;
        while (std::getline(file_in, line)) {
            if (line.rfind("export PX4_GZ_MODEL=", 0) == 0) { // Line starts with "export PX4_GZ_MODEL="
                line = "export PX4_GZ_MODEL=" + config["robots_model"].as<std::string>(); // Modify the line
                found_PX4_GZ_MODEL = true;
            }
            if (line.rfind("export DANCERS_NUM_ROBOTS=", 0) == 0) { // Line starts with "export PX4_GZ_MODEL="
                line = "export DANCERS_NUM_ROBOTS=" + config["robots_number"].as<std::string>(); // Modify the line
                found_DANCERS_NUM_ROBOTS = true;
            }

            file_out << line << std::endl;
        }
        if (found_PX4_GZ_MODEL == false)
        {
            file_out << "export PX4_GZ_MODEL=" + config["robots_model"].as<std::string>() << std::endl;
        }
        if (found_DANCERS_NUM_ROBOTS == false)
        {
            file_out << "export DANCERS_NUM_ROBOTS=" + config["robots_model"].as<std::string>() << std::endl;
        }
        // Close the files
        file_in.close();
        file_out.close();
        // Replace original file with modified file (I hope deleting the .bashrc file have no side-effect, even for a very short time !)
        std::remove(bashrc_path.c_str());
        std::rename("temp.txt", bashrc_path.c_str());


        // Store the system values
        this->sync_window = config["sync_window"].as<int>();
        this->step_length = config["phy_step_size"].as<int>(); // us
        int update_rate = config["update_rate"].as<int>();

        this->current_sim_time = rclcpp::Time(0);
        this->simulation_length = rclcpp::Time(config["simulation_length"].as<int64_t>() * 1e9);    // from s to ns

        if (sync_window % step_length != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
            exit(EXIT_FAILURE);
        }
        uint64_t steps_per_window = sync_window / step_length;

        /******** [Step 1] Set-up and start the Gazebo simulation server ********/

        // Verbosity level for Gazebo - defaults to 1 if unset
        gz::common::Console::SetVerbosity(1);

        // Object to pass custom configuration to the server
        gz::sim::ServerConfig serverConfig;

        // Populate with some configuration, for example, the SDF file to load
        // modify_sdf_world(config["world_sdf_path"].as<std::string>(), step_length, update_rate);
        serverConfig.SetSdfFile(this->ros_ws_path + "/" + config["world_sdf_path"].as<std::string>());
        serverConfig.SetUpdateRate(update_rate); // in Hz

        // Instantiate server
        gz::sim::Server server(serverConfig);

        RCLCPP_INFO(this->get_logger(), "Gazebo server started.");

        /******** [Step 2] Interact with Gazebo Server to modify the world and prepare subscribers ********/

        // Object enabling pub/sub and service calls
        gz::transport::Node node;

        bool executed{false};
        bool result{false};
        unsigned int timeout{5000};

        gz::msgs::StringMsg_V resp;

        // Request the /gazebo/worlds service to get the world's name and make sure it is correctly created.
        std::string service{"/gazebo/worlds"};
        executed = node.Request(service, timeout, resp, result);
        check_service_results(service, executed, result);

        std::string world_name = resp.data(0);
        RCLCPP_INFO(this->get_logger(), "World found with name : %s", world_name.c_str());

        // Call the /world/<world_name>/Create service to create the buildings
        resp.Clear();
        executed = false;
        result = false;
        service = "/world/" + world_name + "/create";
        gz::msgs::EntityFactory req;
        gz::msgs::Boolean respo;

        for (auto building : config["buildings"])
        {
            std::string name = building["id"].as<std::string>();
            double x = building["x"].as<double>();
            double y = building["y"].as<double>();
            double z = building["height"].as<double>() / 2;
            double size_x = building["size_x"].as<double>();
            double size_y = building["size_y"].as<double>();
            double height = building["height"].as<double>();
            std::string buildingSdf = R"(
                    <?xml version="1.0" ?>
                    <sdf version='1.7'>
                        <model name=')" +
                                      name + R"('>
                            <static>true</static>
                            <link name='link'>
                                <pose>)" +
                                      std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + R"( 0 0 0</pose>
                                <visual name='visual'>
                                    <geometry>
                                        <box>
                                            <size>)" +
                                      std::to_string(size_x) + " " + std::to_string(size_y) + " " + std::to_string(height) + R"(</size>
                                        </box>
                                    </geometry>
                                    <material>
                                        <ambient>0.2 0.2 0.2 0.3</ambient>
                                        <diffuse>0.4 0.4 0.8 1</diffuse>
                                        <specular>0.0 0.0 0.0 1</specular>
                                    </material>
                                </visual>
                                <collision name='collision'>
                                    <geometry>
                                        <box>
                                            <size>)" +
                                      std::to_string(size_x) + " " + std::to_string(size_y) + " " + std::to_string(height) + R"(</size>
                                        </box>
                                    </geometry>
                                </collision>
                            </link>
                        </model>
                    </sdf>
                )";

            req.set_sdf(buildingSdf);

            if (verbose)
            {
                RCLCPP_DEBUG(this->get_logger(), "Request creation of entity : \n%s", req.DebugString().c_str());
            }

            executed = node.Request(service, req, timeout, respo, result);
            check_service_results(service, executed, result);
            if (respo.data())
            {
                RCLCPP_DEBUG(this->get_logger(), "Created building %s", name.c_str());
            }
        }

        for (auto secondary_objective : config["secondary_objectives"])
        {
            std::string name = "objective_robot_" + std::to_string(secondary_objective.first.as<uint32_t>());
            int32_t x = secondary_objective.second[0].as<int32_t>();
            int32_t y = secondary_objective.second[1].as<int32_t>();
            int32_t z = secondary_objective.second[2].as<int32_t>();
            std::string objectiveSdf = R"(
                    <?xml version="1.0" ?>
                    <sdf version='1.7'>
                        <model name=')" +
                                        name + R"('>
                            <static>true</static>
                            <link name='link'>
                                <pose>)" +
                                        std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + R"( 0 0 0</pose>
                                <visual name='visual'>
                                    <geometry>
                                        <sphere>
                                            <radius>0.2</radius>
                                        </sphere>
                                    </geometry>
                                    <material>
                                        <ambient>0.2 0.2 0.2 0.3</ambient>
                                        <diffuse>1.0 0.05 0.05 1</diffuse>
                                        <specular>0.0 0.0 0.0 1</specular>
                                    </material>
                                </visual>
                            </link>
                        </model>
                    </sdf>
                )";

            req.set_sdf(objectiveSdf);

            if (verbose)
            {
                RCLCPP_DEBUG(this->get_logger(), "Request creation of entity : \n%s", req.DebugString().c_str());
            }

            executed = node.Request(service, req, timeout, respo, result);
            check_service_results(service, executed, result);
            if (respo.data())
            {
                RCLCPP_DEBUG(this->get_logger(), "Created objective %s", name.c_str());
            }
        }

        // Call the /world/<world_name>/set_physics service to set the Physics of Gazebo simulation
        executed = false;
        result = false;
        service = "/world/" + world_name + "/set_physics";
        gz::msgs::Physics request;
        gz::msgs::Boolean res;
        request.set_max_step_size((double)this->step_length * 0.000001); // step_length in us, converted to seconds
        executed = node.Request(service, request, timeout, res, result);
        check_service_results(service, executed, result);

        // Map holding the robots' names and Pose information and associated mutex
        std::map<std::string, gz::msgs::Pose> robot_poses;
        std::map<std::string, gz::msgs::Odometry> robot_odom;
        std::mutex rob_pos_mutex;
        std::mutex rob_odom_mutex;

        // Callback to the pose/info subscriber. Its role is to fill the robot_poses map
        std::function<void(const gz::msgs::Pose_V &)> cbPoseInfo =
            [&](const gz::msgs::Pose_V &_msg)
        {
            // We use a mutex because there is a concurrency issue with generate_channel_data() function
            std::lock_guard<std::mutex> guard(rob_pos_mutex);
            for (int i = 0; i < _msg.pose_size(); i++)
            {
                std::string entity_name = _msg.pose(i).name();
                // filter the entities given by the topic "/world/<world_name>/pose/info" with the gazebo_names given in the config file
                if (std::find(gazebo_models.begin(), gazebo_models.end(), entity_name) != gazebo_models.end())
                {
                    robot_poses[entity_name] = _msg.pose(i);
                }
            }
        };

        // Create a subscriber to /world/<world_name>/pose/info
        // (this topic sends the pose of all entities in the world, which is overkill for us, but more robust as it does not depend on a robot system plugin)
        if (node.Subscribe("/world/" + world_name + "/pose/info", cbPoseInfo))
        {
            RCLCPP_INFO(this->get_logger(), "Subscribed to /world/%s/pose/info", world_name.c_str());
        }

        if (cosim_mode)
        {
            // **************** UDS SOCKET FOR PHYSICS COORDINATOR ****************
            // Create and connect UDS Socket
            Socket *socket;
            boost::asio::io_context io_context;
            if (config["phy_use_uds"].as<bool>())
            {
                socket = new UDSSocket(io_context);
                socket->accept(config["phy_uds_server_address"].as<std::string>(), 0);
            }
            else
            {
                socket = new TCPSocket(io_context);
                socket->accept(config["phy_ip_server_address"].as<std::string>(), config["phy_ip_server_port"].as<unsigned short>());
            }

            /******** [Step 3] Main simulation loop ********/
            double sim_time = 0.0;

            while (this->current_sim_time < this->simulation_length || this->simulation_length == rclcpp::Time(0.0))
            {
                // std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
                // Wait until reception of a message on the UDS socket
                std::string received_data = gzip_decompress(socket->receive_one_message());
                // std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                // int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
                // if(wait != 0)
                //     RCLCPP_WARN(this->get_logger(), "Received %li bytes in %li microseconds", received_data.size(), wait);

                // Initialize empty protobuf message type [NetworkUpdate]
                network_update_proto::NetworkUpdate network_update_msg;
                // Transform the message received from the UDS socket (string -> protobuf)
                network_update_msg.ParseFromString(received_data);

                if(!network_update_msg.ordered_neighbors().empty())
                {

                    ordered_neighbors_proto::OrderedNeighborsList neighbors_list_msg;
                    neighbors_list_msg.ParseFromString(gzip_decompress(network_update_msg.ordered_neighbors()));

                    // std::cout << neighbors_list_msg.DebugString() << std::endl;

                    // convert the received protobuf message to ROS2 messages and publish them on separate topics
                    for (int i = 0; i < neighbors_list_msg.ordered_neighbors_size(); i++)
                    {
                        dancers_msgs::msg::NeighborArray msg{};
                        int agent_id = neighbors_list_msg.ordered_neighbors(i).agentid();
                        for (int j = 0; j < neighbors_list_msg.ordered_neighbors(i).neighborid_size(); j++)
                        {
                            dancers_msgs::msg::Neighbor neighbor{};
                            neighbor.agent_id = neighbors_list_msg.ordered_neighbors(i).neighborid(j) + 1;                // ATTENTION: here we add 1 to the agent IDs because we want the IDs to start at 1 when using Gazebo, because the PX4 works with instance numbers starting at 1 !
                            neighbor.link_quality = neighbors_list_msg.ordered_neighbors(i).linkquality(j);
                            msg.neighbors.push_back(neighbor);
                        }
                        this->neighbors_pub_vector_[agent_id]->publish(msg);
                    }
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Received a BEGIN message but empty neighbor information from the network simulator.");
                }

                std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
                // Advance the server in this thread (blocking) for W iterations, pause at the end
                server.Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);
                if (this->save_compute_time)
                {
                    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                    int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count(); // us
                    this->compute_time_output.open(m_computation_time_file.c_str(), std::fstream::app);
                    this->compute_time_output << wait << "\n";
                    this->compute_time_output.close();
                }

                if (verbose)
                {
                    // 34 = Blue :)
                    RCLCPP_INFO(this->get_logger(), "\x1b[34m[%f] Advanced %i milliseconds\x1b[0m", sim_time, this->sync_window / 1000);
                }

                // generate the channel_data message from the pose of the robots
                rob_pos_mutex.lock();
                std::string robots_positions = generate_robots_positions(robot_poses);
                rob_pos_mutex.unlock();

                // Generate the response message
                std::string response = GenerateResponseProtobuf(robots_positions);

                // Send the response in the UDS socket
                socket->send_one_message(response);

                this->current_sim_time += std::chrono::microseconds(this->step_length);

            }
        }
        else 
        {
            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

            server.Run();

            if (this->save_compute_time)
            {
                std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count(); // us
                this->compute_time_output.open(m_computation_time_file.c_str(), std::fstream::app);
                this->compute_time_output << wait << "\n";
                this->compute_time_output.close();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Simulation finished");
        exit(EXIT_SUCCESS);
    }

private:
    // saving of computation time
    bool save_compute_time;
    std::chrono::system_clock::time_point previous_end_time;
    std::string m_computation_time_file;
    std::ofstream compute_time_output;

    int step_length;
    int sync_window;

    rclcpp::Time current_sim_time;
    rclcpp::Time simulation_length;

    // ros2 publishers
    std::vector< rclcpp::Publisher<dancers_msgs::msg::NeighborArray>::SharedPtr > neighbors_pub_vector_;

    std::string ros_ws_path;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboticsCoordinator>());
    rclcpp::shutdown();
    return 0;
}
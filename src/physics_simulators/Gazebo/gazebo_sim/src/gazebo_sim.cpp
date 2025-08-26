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

#include <protobuf_msgs/dancers_update.pb.h>
#include <protobuf_msgs/ordered_neighbors.pb.h>
#include <protobuf_msgs/pose_vector.pb.h>

// ROS2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <dancers_msgs/msg/neighbor.hpp>
#include <dancers_msgs/msg/neighbor_array.hpp>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <time_probe.hpp>

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

class GazeboConnector : public rclcpp::Node
{
public:
    GazeboConnector() : Node("gazebo_connector")
    {
        /* ----------- Read Configuration file ----------- */

        // Declare the config_file ROS2 parameter
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        param_desc.description = "Executes in co-simulation or mini-dancers only.";
        this->declare_parameter("cosim_mode", true, param_desc);

        this->cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
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
        this->config = YAML::LoadFile(config_file_path);

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
            while (this->m_computation_time_file.empty())
            {
                temp_path = ros_ws_path + "/data/" + experience_name + "/physics" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    this->m_computation_time_file = temp_path;
                }
            }

            // initialize the output file with headers
            this->probe = WallTimeProbe(this->m_computation_time_file);
        }

        /* ----------- Configuration file parsing ----------- */

        this->it = 0;
        this->step_size = config["phy_step_size"].as<int>() / 1000000.0f; // us to s
        this->it_end_sim = uint64_t(simulation_length / this->step_size);


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
        int update_rate = config["update_rate"].as<int>();

        int sync_window_int = config["sync_window"].as<int>();
        int step_size_int = config["phy_step_size"].as<int>();
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the network step size, aborting.");
            exit(EXIT_FAILURE);
        }
        this->sync_window = sync_window_int / 1000000.0f;   // us to s
        this->step_size = step_size_int / 1000000.0f;       // us to s
        this->it = 0;
        this->simulation_length = config["simulation_length"].as<double>();
        this->it_end_sim = uint64_t(simulation_length / this->step_size);

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
        this->server_ptr = std::make_unique<gz::sim::Server>(serverConfig);

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

            RCLCPP_DEBUG(this->get_logger(), "Request creation of entity : \n%s", req.DebugString().c_str());

            executed = node.Request(service, req, timeout, respo, result);
            check_service_results(service, executed, result);
            if (respo.data())
            {
                RCLCPP_DEBUG(this->get_logger(), "Created building %s", name.c_str());
            }
        }

        for (auto secondary_objective : config["secondary_objectives"])
        {
            std::string name = "objective_robot_" + std::to_string(secondary_objective["id"].as<uint32_t>());
            float x = secondary_objective["position"]["x"].as<float>();
            float y = secondary_objective["position"]["y"].as<float>();
            float z = secondary_objective["position"]["z"].as<float>();
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

            RCLCPP_DEBUG(this->get_logger(), "Request creation of entity : \n%s", req.DebugString().c_str());

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
        request.set_max_step_size(this->step_size); // step_size is in seconds
        executed = node.Request(service, request, timeout, res, result);
        check_service_results(service, executed, result);

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
                    try{
                        uint32_t robot_id = std::stoi(entity_name.substr(entity_name.find_last_of('_') + 1));
                        this->robot_poses[robot_id] = _msg.pose(i);
                    }
                    catch (const std::invalid_argument& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to find the robot id in the entity name of the robot: %s", entity_name.c_str());
                    }
                }
            }
        };

        // Create a subscriber to /world/<world_name>/pose/info
        // (this topic sends the pose of all entities in the world, which is overkill for us, but more robust as it does not depend on a robot system plugin)
        if (node.Subscribe("/world/" + world_name + "/pose/info", cbPoseInfo))
        {
            RCLCPP_INFO(this->get_logger(), "Subscribed to /world/%s/pose/info", world_name.c_str());
        }

        /******** [Step 3] Main simulation loop ********/
        this->Loop();

    }

private:
    /* Time and iteration counters */
    double sync_window;                     // s
    double step_size;                       // s
    double simulation_length;               // s
    uint64_t it;
    uint64_t it_end_sim;

    // Map holding the robots' names and Pose information and associated mutex
    std::map<uint32_t, gz::msgs::Pose> robot_poses;
    std::mutex rob_pos_mutex;

    /* Publishers */
    std::vector< rclcpp::Publisher<dancers_msgs::msg::NeighborArray>::SharedPtr > neighbors_pub_vector_;

    std::string ros_ws_path;
    bool cosim_mode;
    YAML::Node config;

    /* Compute time saving */
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;

    /* Methods */
    void Loop();
    std::string GenerateResponseProtobuf();

    std::unique_ptr<gz::sim::Server> server_ptr;
};

/**
 * \brief Generates the final protobuf message of protobuf_msgs/PhysicsUpdate.
 *
 * It will fill the message with the (compressed) robots_positions for the current simulation window,
 * change the message-type to END, string-serialize the protobuf message, compress it, and return it.
 *
 */
std::string GazeboConnector::GenerateResponseProtobuf()
{
    // Change message's type to END
    dancers_update_proto::DancersUpdate physics_update_msg;
    physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

    dancers_update_proto::PoseVector robots_positions_msg;
    for (const auto& robot : this->robot_poses)
    {
        gz::msgs::Vector3d pos = robot.second.position();
        gz::msgs::Quaternion quat = robot.second.orientation();
        dancers_update_proto::Pose *robot_pose_msg = robots_positions_msg.add_pose();
        robot_pose_msg->set_agent_id(robot.first);
        robot_pose_msg->set_x(pos.x());
        robot_pose_msg->set_y(pos.y());
        robot_pose_msg->set_z(pos.z());
        robot_pose_msg->set_qw(quat.w());
        robot_pose_msg->set_qx(quat.x());
        robot_pose_msg->set_qy(quat.y());
        robot_pose_msg->set_qz(quat.z());
    }

    std::string robots_positions_string;
    robots_positions_msg.SerializeToString(&robots_positions_string);

    // Fill the channel_data field with the compressed data from the physics simulation
    physics_update_msg.set_payload(gzip_compress(robots_positions_string));

    // Transform the response [protobuf] --> [string]
    std::string str_response;
    physics_update_msg.SerializeToString(&str_response);
    str_response = gzip_compress(str_response);

    return str_response;
}

/**
 * @brief Main simulation loop
 *
 * This function should be called only once at the end of the ROS2 node's constructor. If the co-simulation mode is used, it connects to the Coordinator node via an UDS Socket and them loop until the end off the
 */
void GazeboConnector::Loop()
{
    if (this->cosim_mode)
    {
        // **************** UDS SOCKET FOR PHYSICS COORDINATOR ****************
        // Create and connect UDS Socket
        Socket *socket;
        boost::asio::io_context io_context;
        if (this->config["phy_use_uds"].as<bool>())
        {
            socket = new UDSSocket(io_context);
            socket->accept(this->config["phy_uds_server_address"].as<std::string>(), 0);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->accept(this->config["phy_ip_server_address"].as<std::string>(), this->config["phy_ip_server_port"].as<unsigned int>());
        }
        RCLCPP_INFO(this->get_logger(), "\x1b[32mSocket connected with Coordinator \x1b[0m");

        while (rclcpp::ok() && (this->it < this->it_end_sim || this->simulation_length == 0.0))
        {
            // Wait until reception of a message on the socket
            std::string received_data = gzip_decompress(socket->receive_one_message());

            // Initialize empty protobuf message type [DancersUpdate]
            dancers_update_proto::DancersUpdate network_update_msg;

            // Transform the message received from the UDS socket (string -> protobuf)
            network_update_msg.ParseFromString(received_data);

            if(!network_update_msg.payload().empty())
            {

                dancers_update_proto::OrderedNeighborsList neighbors_list_msg;
                neighbors_list_msg.ParseFromString(gzip_decompress(network_update_msg.payload()));

                // std::cout << neighbors_list_msg.DebugString() << std::endl;

                // convert the received protobuf message to ROS2 messages and publish them on separate topics
                for (int i = 0; i < neighbors_list_msg.ordered_neighbors_size(); i++)
                {
                    uint32_t agent_id = neighbors_list_msg.ordered_neighbors(i).agentid();

                    dancers_msgs::msg::NeighborArray msg{};
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

            if (this->save_compute_time)
            {
                this->probe.start();
            }                
            
            // Advance the server in this thread (blocking) for W iterations, pause at the end
            this->server_ptr->Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);

            if (this->save_compute_time)
            {
                this->probe.stop();
            }

            // Generate the response message
            std::string response = GenerateResponseProtobuf();

            // Send the response in the UDS socket
            socket->send_one_message(response);

            this->it++;

        }
    }
    else 
    {
        if (this->save_compute_time)
        {
            this->probe.start();
        }

        server_ptr->Run();

        if (this->save_compute_time)
        {
            this->probe.stop();
        }
    }

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboConnector>());
    rclcpp::shutdown();
    return 0;
}
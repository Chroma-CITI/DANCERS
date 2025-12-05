#include <thread>
#include <chrono>

// Main ROS2 lib
#include <rclcpp/rclcpp.hpp>

// ROS2 message for clock
#include <rosgraph_msgs/msg/clock.hpp>

#include <yaml_util.hpp>

// Protobuf messages for DANCERS
#include <protobuf_msgs/dancers_update.pb.h>

// Boost "barrier" lib
#include <boost/fiber/barrier.hpp>

// Custom UDS / TCP socket functions 
#include <uds_tcp_socket.hpp>

// Custom zip functions
#include <compression_util.hpp>

// Custom wall time and CPU time measure probe
#include <time_probe.hpp>


/**
 * @brief The Coordinator ROS2 node, part of the DANCERS co-simulator
 * 
 * It handles the synchronization and information-passing between a network simulator and a physics simulator. 
 */
class Coordinator : public rclcpp::Node
{
public:
    Coordinator() : Node("coordinator")
    {
        // Declare the config_file parameter for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        
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
        this->config_ = YAML::LoadFile(config_file_path);

        // Check that the step sizes are coherent with the synchronization window
        if (config_["sync_window"].as<uint32_t>() % config_["phy_step_size"].as<uint32_t>() != 0 || config_["sync_window"].as<uint32_t>() % config_["net_step_size"].as<uint32_t>() != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "The synchronization window size (sync_window) must be divisible by both the physics and network step sizes (phy_step_size ; net_step_size). Aborting.");
            exit(EXIT_FAILURE);
        }

        // Initialize the current simulation time to 0 and create /clock publisher
        this->current_sim_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
        auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        this->clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);

        // Run the physics and network threads handling communication with the connectors
        this->phy_protobuf_thread = std::thread(&Coordinator::run_phy_protobuf_client_, this);
        this->net_protobuf_thread = std::thread(&Coordinator::run_net_protobuf_client_, this);
        this->real_time_thread = std::thread(&Coordinator::run_real_time_thread_, this);

        phy_protobuf_thread.join();
        net_protobuf_thread.join();
        real_time_thread.join();

        if (getYamlValue<bool>(this->config_, "save_compute_time"))
        {
            this->probe = WallTimeProbe(getYamlValue<std::string>(this->config_, "results_folder")+"/compute_time_coordinator.csv");
            this->probe.start();
        }

        RCLCPP_INFO(this->get_logger(), "Coordinator node finished successfully, exiting.");
        exit(EXIT_SUCCESS);
    }


private:
    YAML::Node config_;

    rclcpp::Time current_sim_time; // us

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

    boost::fibers::barrier rendezvous_threads{3};

    std::string compressed_physics_to_network_data;
    std::string compressed_network_to_physics_data;

    std::mutex physics_to_network_data_mutex;
    std::mutex network_to_physics_data_mutex;

    void run_phy_protobuf_client_();
    void run_net_protobuf_client_();
    void run_real_time_thread_();

    std::thread phy_protobuf_thread;
    std::thread net_protobuf_thread;
    std::thread real_time_thread;

    std::string ros_ws_path;

    WallTimeProbe probe;
};

/**
 * @brief Thread function for interaction with the physics simulator
 * 
 * The function executed in a thread that handles all interactions with the physics simulator. It uses sockets (UDS or TCP) to exchange protobuf messages with the physics simulator
 */
void Coordinator::run_phy_protobuf_client_()
{
    RCLCPP_DEBUG(this->get_logger(), "Starting PHY protobuf client in thread.");

    try
    {
        // Opens either a UDS socket or a TCP socket depending on config
        CustomSocket *socket;
        boost::asio::io_context io_context;
        if (getYamlValue<bool>(this->config_, "phy_use_uds"))
        {
            socket = new UDSSocket(io_context);
            socket->accept(getYamlValue<std::string>(this->config_, "phy_uds_server_address"), 0);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->accept(config_["phy_ip_server_address"].as<std::string>(), config_["phy_ip_server_port"].as<unsigned short>());
        }
        
        RCLCPP_INFO(this->get_logger(), "\x1b[32m Connected PHY socket !\x1b[0m");
        
        // Simulation length is in seconds in the config file
        rclcpp::Time simulation_length = rclcpp::Time(getYamlValue<int64_t>(this->config_, "simulation_length") * 1e9, RCL_ROS_TIME);
        uint32_t sync_window = getYamlValue<uint32_t>(this->config_, "sync_window");
        uint32_t phy_step_size = getYamlValue<uint32_t>(this->config_, "phy_step_size");
        uint32_t net_step_size = getYamlValue<uint32_t>(this->config_, "net_step_size");
        bool save_compute_time = getYamlValue<bool>(this->config_, "save_compute_time");

        while (this->current_sim_time < simulation_length || simulation_length == rclcpp::Time(0.0, RCL_ROS_TIME))
        {
            // Execute N_{phy}=sync_window/phy_step_size simulation steps before synchronization
            for (uint32_t i = 0; i < sync_window / phy_step_size; i++)
            {

                // Send start request
                dancers_update_proto::DancersUpdate network_update_msg;
                network_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);

                // Add information from the network simulator if available
                {
                    std::lock_guard<std::mutex> lock(this->network_to_physics_data_mutex);
                    if (!this->compressed_network_to_physics_data.empty())
                    {
                        network_update_msg.set_payload(this->compressed_network_to_physics_data);
                        this->compressed_network_to_physics_data.clear();
                    }
                }

                std::string request = gzip_compress(network_update_msg.SerializeAsString());

                socket->send_one_message(request);

                // Get response (blocking call)
                std::string response = gzip_decompress(socket->receive_one_message());

                if (response.length() == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Empty message from PHY Connector");
                    continue;
                }

                dancers_update_proto::DancersUpdate physics_update_msg;
                physics_update_msg.ParseFromString(response);

                if (physics_update_msg.msg_type() != dancers_update_proto::DancersUpdate::END)
                {
                    throw "Coordinator received a non-END message from PHY Connector!";
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Finished 1 step of PHY simulator.");

                    // Publish current time of simulation to /clock
                    if (phy_step_size <= net_step_size)
                    {
                        this->current_sim_time += std::chrono::microseconds(phy_step_size);
                        rosgraph_msgs::msg::Clock clock_msg;
                        clock_msg.set__clock(this->current_sim_time);
                        this->clock_publisher_->publish(clock_msg);
                    }

                    std::lock_guard<std::mutex> lock(this->physics_to_network_data_mutex);
                    this->compressed_physics_to_network_data = physics_update_msg.payload();
                }
            }

            if (save_compute_time)
            {
                this->probe.stop();
                this->probe.start();
            }
            
            this->rendezvous_threads.wait();

        }

        // Cleanly close the socket
        dancers_update_proto::DancersUpdate physics_update_msg;
        physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::CLOSE);
        std::string request = gzip_compress(physics_update_msg.SerializeAsString());
        socket->send_one_message(request);

        socket->close();

        RCLCPP_INFO(this->get_logger(), "Simulation finished (PHY thread).");
    } // end try
    catch (std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        exit(EXIT_FAILURE);
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Error happened in the Physics protobuf thread !");
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Thread function for interaction with the network simulator
 * 
 * The function executed in a thread that handles all interactions with the network Connector. It uses sockets (UDS or TCP) to exchange protobuf messages with the network simulator. We keep the two thread functions separated even though they are similar for debugging purposes.
 */
void Coordinator::run_net_protobuf_client_()
{
    RCLCPP_DEBUG(this->get_logger(), "Starting NET protobuf client in thread.");

    try
    {
        // Opens either a UDS socket or a TCP socket depending on config
        CustomSocket *socket;
        boost::asio::io_context io_context;
        if (getYamlValue<bool>(this->config_, "net_use_uds"))
        {
            socket = new UDSSocket(io_context);
            socket->accept(getYamlValue<std::string>(this->config_, "net_uds_server_address"), 0);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->accept(config_["net_ip_server_address"].as<std::string>(), config_["net_ip_server_port"].as<unsigned short>());
        }
        
        RCLCPP_INFO(this->get_logger(), "\x1b[32m Connected NET socket !\x1b[0m");
        
        // Simulation length is in seconds in the config file
        rclcpp::Time simulation_length = rclcpp::Time(getYamlValue<int64_t>(this->config_, "simulation_length") * 1e9, RCL_ROS_TIME);
        uint32_t sync_window = getYamlValue<uint32_t>(this->config_, "sync_window");
        uint32_t phy_step_size = getYamlValue<uint32_t>(this->config_, "phy_step_size");
        uint32_t net_step_size = getYamlValue<uint32_t>(this->config_, "net_step_size");
        bool save_compute_time = getYamlValue<bool>(this->config_, "save_compute_time");

        while (this->current_sim_time < simulation_length || simulation_length == rclcpp::Time(0.0, RCL_ROS_TIME))
        {
            // Execute N_{phy}=sync_window/net_step_size simulation steps before synchronization
            for (uint32_t i = 0; i < sync_window / net_step_size; i++)
            {

                // Send start request
                dancers_update_proto::DancersUpdate physics_update_msg;
                physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);

                // Add the information from the physics simulator if available
                {
                    std::lock_guard<std::mutex> lock(this->physics_to_network_data_mutex);
                    if (!this->compressed_physics_to_network_data.empty())
                    {
                        physics_update_msg.set_payload(this->compressed_physics_to_network_data);
                        this->compressed_physics_to_network_data.clear();
                    }
                }

                std::string request = gzip_compress(physics_update_msg.SerializeAsString());

                socket->send_one_message(request);

                // Get response (blocking call)
                std::string response = gzip_decompress(socket->receive_one_message());

                if (response.length() == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Empty message from NET Connector");
                    continue;
                }

                dancers_update_proto::DancersUpdate network_update_msg;
                network_update_msg.ParseFromString(response);

                if (network_update_msg.msg_type() != dancers_update_proto::DancersUpdate::END)
                {
                    throw "Coordinator received a non-END message from NET Connector!";
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Finished 1 step of NET simulator.");

                    // Publish current time of simulation to /clock
                    if (net_step_size < phy_step_size)
                    {
                        this->current_sim_time += std::chrono::microseconds(net_step_size);
                        rosgraph_msgs::msg::Clock clock_msg;
                        clock_msg.set__clock(this->current_sim_time);
                        this->clock_publisher_->publish(clock_msg);
                    }

                    std::lock_guard<std::mutex> lock(this->network_to_physics_data_mutex);
                    this->compressed_network_to_physics_data = network_update_msg.payload();
                }
            }

            if (save_compute_time)
            {
                this->probe.stop();
                this->probe.start();
            }

            this->rendezvous_threads.wait();

        }

        // Cleanly close the socket
        dancers_update_proto::DancersUpdate physics_update_msg;
        physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::CLOSE);
        std::string request = gzip_compress(physics_update_msg.SerializeAsString());
        socket->send_one_message(request);

        socket->close();

        RCLCPP_INFO(this->get_logger(), "Simulation finished (NET thread).");
    } // end try
    catch (std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        exit(EXIT_FAILURE);
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Error happened in the Network protobuf thread !");
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Thread function that limits the speed of simulation (if specified)
 * 
 * If a real_time_factor is specified in the config file, the function will sleep for the appropriate amount of time between each synchronization window. If not, the function will run as fast as possible.
 */
void Coordinator::run_real_time_thread_()
{
    RCLCPP_DEBUG(this->get_logger(), "Starting real time factor thread.");
    double RTF;
    try
    {
        RTF = getYamlValue<double>(this->config_, "real_time_factor");
    }
    catch (std::runtime_error &e)
    {
        RCLCPP_WARN(this->get_logger(), "The real_time_factor parameter was not found in the config, running at max speed.");
        RTF = std::numeric_limits<double>::infinity();
    }

    try
    {
        rclcpp::Time simulation_length = rclcpp::Time(getYamlValue<int64_t>(this->config_, "simulation_length") * 1e9, RCL_ROS_TIME);
        uint32_t sync_window = getYamlValue<uint32_t>(this->config_, "sync_window");
        uint64_t time_to_sleep = (uint64_t)(sync_window / RTF);

        while(this->current_sim_time < simulation_length || simulation_length == rclcpp::Time(0.0, RCL_ROS_TIME))
        {
            std::this_thread::sleep_for(std::chrono::microseconds(time_to_sleep));

            this->rendezvous_threads.wait();
        }
    
    } // end try
    catch (std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
        exit(EXIT_FAILURE);
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Error happened in the Real time factor thread !");
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Coordinator>());
    rclcpp::shutdown();
    return 0;
}
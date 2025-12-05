// connector.hpp
#pragma once

// Main ROS2 lib
#include <rclcpp/rclcpp.hpp>

// Protobuf messages for DANCERS
#include <protobuf_msgs/dancers_update.pb.h>

// Custom UDS / TCP socket functions 
#include <uds_tcp_socket.hpp>

// Custom zip functions
#include <compression_util.hpp>

// Custom wrapper function for yaml access
#include <yaml_util.hpp>

// Custom wall time and CPU time measure probe
#include <time_probe.hpp>

// C++ standard lib
#include <unistd.h>
#include <thread>

class Connector : public rclcpp::Node
{
public:
    Connector(const std::string & node_name)
    : rclcpp::Node(node_name)
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

        // Set variables common to all connectors
        this->it = 0;
        this->simulation_length = config_["simulation_length"].as<double>();
        this->sync_window = config_["sync_window"].as<unsigned int>() / 1000000.0f; // us to s
    }

protected:
    YAML::Node config_;

    /* Time and iteration counters */
    double sync_window;             // s
    double step_size;               // s
    double simulation_length;       // s
    uint64_t it;                    // -
    uint64_t it_end_sim;            // -

    /* Thread running the co-simulation Loop */
    std::thread loop_thread_;

    /* Main loop function that handles exchanges with the coordinator */
    void Loop()
    {
        RCLCPP_DEBUG(this->get_logger(), "Connecting with Coordinator");

        // Create and connect UDS Socket
        CustomSocket *socket;
        boost::asio::io_context io_context;
        if (this->use_uds)
        {
            socket = new UDSSocket(io_context);
            socket->connect(this->uds_server_address);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->connect(this->ip_server_address);
        }
        RCLCPP_INFO(this->get_logger(), "\x1b[32mSocket connected with Coordinator \x1b[0m");

        // **************** MAIN SIMULATION LOOP ****************
        while (rclcpp::ok())
        {
            std::string received_data = gzip_decompress(socket->receive_one_message());

            // Initialize empty protobuf message
            dancers_update_proto::DancersUpdate update_msg;

            // Transform the message received from the UDS socket (string -> protobuf)
            update_msg.ParseFromString(received_data);

            if (update_msg.msg_type() == dancers_update_proto::DancersUpdate::CLOSE) 
            {
                break;
            }

            if (this->save_compute_time)
            {
                this->probe.start();
            }

            dancers_update_proto::DancersUpdate response_update_msg = this->StepSimulation(update_msg);

            if (this->save_compute_time)
            {
                this->probe.stop();
            }
            
            this->it++;

            // Convert the response protobuf message to compressed string
            std::string response;
            response_update_msg.SerializeToString(&response);
            response = gzip_compress(response);

            // Send the response in the UDS socket
            socket->send_one_message(response);
        }
        
        RCLCPP_INFO(this->get_logger(), "\x1b[32mSimulation finished cleanly.\x1b[0m");
        socket->close();
        exit(EXIT_SUCCESS);
    }

    /* Pure virtual methods to be implemented by subclasses */
    virtual void ReadConfigFile() = 0;
    virtual dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate update_msg) = 0;

    /* Compute time saving */
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;

    /* Co-simulation socket */
    bool use_uds;
    std::string uds_server_address;
    std::string ip_server_address;
    unsigned int ip_server_port;

    std::string ros_ws_path;

};


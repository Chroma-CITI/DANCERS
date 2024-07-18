#include <iostream>
#include <thread>
#include <barrier>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/fiber/barrier.hpp>

#include "udp_tcp_socket.hpp"

#include <yaml-cpp/yaml.h>

#include <protobuf_msgs/physics_update.pb.h>
#include <protobuf_msgs/network_update.pb.h>
#include <protobuf_msgs/robots_positions.pb.h>

static std::string m_computation_time_file;

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


class Coordinator : public rclcpp::Node
{
public:
    Coordinator() : Node("coordinator")
    {
        this->previous_end_time = std::chrono::system_clock::now();

        // Declare two parameters for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "/home/theotime/simulation_ws/src/config/config_flocking_2.yaml", param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s\nA config file must be given in the launch file.", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        this->save_compute_time = config["save_compute_time"].as<bool>();

        // vvvvvvvv DATA SAVING vvvvvvvv
        if (this->save_compute_time)
        {
            // Create a folder based on the experience name, if not existant already
            std::string experience_name = config["experience_name"].as<std::string>();
            if (boost::filesystem::create_directories("./data/" + experience_name))
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
                temp_path = "./data/" + experience_name + "/coordinator" + std::to_string(i) + ".csv";
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
            this->f.open(m_computation_time_file.c_str(), std::ios::out);
            this->f << "Time[us]"
                    << std::endl;
            this->f.close();
        }
        // ^^^^^^^^ DATA SAVING ^^^^^^^^

        this->current_sim_time = rclcpp::Time(0);
        auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        this->clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);

        this->phy_use_uds_socket = config["phy_use_uds"].as<bool>();
        this->net_use_uds_socket = config["net_use_uds"].as<bool>();
        this->phy_ip_server_address = config["phy_ip_server_address"].as<std::string>();
        this->net_ip_server_address = config["net_ip_server_address"].as<std::string>();
        this->phy_ip_server_port = config["phy_ip_server_port"].as<uint32_t>();
        this->net_ip_server_port = config["net_ip_server_port"].as<uint32_t>();
        this->phy_uds_server_address = config["phy_uds_server_address"].as<std::string>();
        this->net_uds_server_address = config["net_uds_server_address"].as<std::string>();
        this->phy_step_size = config["phy_step_size"].as<uint32_t>();
        this->net_step_size = config["net_step_size"].as<uint32_t>();
        this->sync_window = config["sync_window"].as<uint32_t>();
        if (this->sync_window % this->net_step_size != 0 || this->sync_window % this->phy_step_size != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "The synchronization window size (sync_window) must be divisible by both the physics and network step sizes (phy_step_size ; net_step_size). Aborting.");
            exit(EXIT_FAILURE);
        }

        this->phy_protobuf_thread = std::thread(&Coordinator::run_phy_protobuf_client_, this);
        this->net_protobuf_thread = std::thread(&Coordinator::run_net_protobuf_client_, this);


        phy_protobuf_thread.join();
        net_protobuf_thread.join();

        std::cout << "Finished constructor" << std::endl;
    }

private:
    rclcpp::Time current_sim_time; // us
    bool should_run = true;

    bool save_compute_time;
    boost::fibers::barrier rendezvous_threads{2};
    uint32_t phy_step_size; // us
    uint32_t net_step_size; // us
    uint32_t sync_window;
    std::string compressed_robots_positions;
    std::string compressed_ordered_neighbors;
    std::mutex robots_positions_mutex;
    std::mutex ordered_neighbors_mutex;

    bool phy_use_uds_socket;
    bool net_use_uds_socket;
    std::string phy_uds_server_address;
    std::string net_uds_server_address;
    std::string phy_ip_server_address;
    std::string net_ip_server_address;
    uint32_t phy_ip_server_port;
    uint32_t net_ip_server_port;

    std::ofstream f;
    std::chrono::system_clock::time_point previous_end_time;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

    void run_phy_protobuf_client_();
    void run_net_protobuf_client_();

    std::thread phy_protobuf_thread;
    std::thread net_protobuf_thread;
};


void Coordinator::run_phy_protobuf_client_()
{
    RCLCPP_INFO(this->get_logger(), "Starting PHY protobuf client in thread.");

    try
    {
        Socket *socket;
        boost::asio::io_context io_context;

        if (this->phy_use_uds_socket)
        {
            socket = new UDSSocket(io_context);
            socket->connect(this->phy_uds_server_address);
        }
        else 
        {
            socket = new TCPSocket(io_context);
            socket->connect(this->phy_ip_server_address);
        }
        RCLCPP_INFO(this->get_logger(), "\x1b[32m Connected PHY socket !\x1b[0m");

        while (this->should_run)
        {
            this->rendezvous_threads.wait();

            for (uint32_t i = 0; i < this->sync_window / this->phy_step_size; i++)
            {

                // Send start request
                physics_update_proto::PhysicsUpdate PhysicsUpdate_msg;
                PhysicsUpdate_msg.set_time_val(static_cast<uint32_t>(this->current_sim_time.nanoseconds()));
                PhysicsUpdate_msg.set_msg_type(physics_update_proto::PhysicsUpdate::BEGIN);

                // Add the neighbors list if we received one from the network simulator
                this->ordered_neighbors_mutex.lock();
                if (!this->compressed_ordered_neighbors.empty())
                {
                    PhysicsUpdate_msg.set_ordered_neighbors(this->compressed_ordered_neighbors);
                    this->compressed_ordered_neighbors.clear();
                }
                this->ordered_neighbors_mutex.unlock();

                std::string request = gzip_compress(PhysicsUpdate_msg.SerializeAsString());

                socket->send_one_message(request);

                // Get response
                std::string response = gzip_decompress(socket->receive_one_message());

                if (response.length() == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Empty message");
                    continue;
                }
                PhysicsUpdate_msg.ParseFromString(response);

                robots_positions_proto::RobotsPositions robots_positions_msg;
                robots_positions_msg.ParseFromString(gzip_decompress(PhysicsUpdate_msg.robots_positions()));
                
                RCLCPP_INFO(this->get_logger(), "Received robots positions from viragh simulator: %s", robots_positions_msg.DebugString().c_str());


                if (PhysicsUpdate_msg.msg_type() != physics_update_proto::PhysicsUpdate::END)
                {
                    throw "Coordinator received a non-END message from physics simulator !";
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Finished 1 iteration of PHY simulator.");

                    std::lock_guard<std::mutex> lock(this->robots_positions_mutex);
                    this->compressed_robots_positions = PhysicsUpdate_msg.robots_positions();
                }
            }
        }

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

void Coordinator::run_net_protobuf_client_()
{
    RCLCPP_INFO(this->get_logger(), "Starting NET protobuf client in thread.");

    try
    {
        Socket *socket;
        boost::asio::io_context io_context;

        if (this->net_use_uds_socket)
        {
            socket = new UDSSocket(io_context);
            socket->connect(this->net_uds_server_address);
        }
        else 
        {
            socket = new TCPSocket(io_context);
            socket->connect(this->net_ip_server_address);
        }
        RCLCPP_INFO(this->get_logger(), "\x1b[32m Connected NET socket !\x1b[0m");

        while (this->should_run)
        {
            this->rendezvous_threads.wait();

            if (this->save_compute_time)
            {
                std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
                int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(now - this->previous_end_time).count();
                this->f.open(m_computation_time_file.c_str(), std::fstream::app);
                this->f << wait << "\n";
                this->f.close();
                this->previous_end_time = now;
            }

            for (uint32_t i = 0; i < this->sync_window / this->net_step_size; i++)
            {

                // Send start request
                network_update_proto::NetworkUpdate NetworkUpdate_msg;
                NetworkUpdate_msg.set_time_val(static_cast<uint32_t>(this->current_sim_time.nanoseconds()));
                NetworkUpdate_msg.set_msg_type(network_update_proto::NetworkUpdate::BEGIN);

                // Add the positions of the robots only if we received them from the physics simulator !
                this->robots_positions_mutex.lock();
                if (!this->compressed_robots_positions.empty())
                {
                    NetworkUpdate_msg.set_robots_positions(this->compressed_robots_positions);
                    this->compressed_robots_positions.clear();
                }
                this->robots_positions_mutex.unlock();

                std::string request = gzip_compress(NetworkUpdate_msg.SerializeAsString());

                socket->send_one_message(request);

                // Get response
                std::string response = gzip_decompress(socket->receive_one_message());
                if (response.length() == 0)
                {
                    continue;
                }
                NetworkUpdate_msg.ParseFromString(response);
                if (NetworkUpdate_msg.msg_type() != network_update_proto::NetworkUpdate::END)
                {
                    throw "Coordinator received a non-END message from network simulator !";
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "Finished 1 iteration of NET simulator.");

                    // Publish current time of simulation to /clock for ROS Nodes
                    this->current_sim_time += std::chrono::microseconds(this->net_step_size);
                    rosgraph_msgs::msg::Clock clock_msg;
                    clock_msg.set__clock(this->current_sim_time);
                    this->clock_publisher_->publish(clock_msg);

                    std::lock_guard<std::mutex> lock(this->ordered_neighbors_mutex);
                    this->compressed_ordered_neighbors = NetworkUpdate_msg.ordered_neighbors();
                }
            }
        }

    } // end try
    catch (std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Error happened in the Physics protobuf thread !");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Coordinator>());
    rclcpp::shutdown();
    return 0;
}
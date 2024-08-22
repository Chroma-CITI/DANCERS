#include "rclcpp/rclcpp.hpp"

#include <protobuf_msgs/physics_update.pb.h>
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include <udp_tcp_socket.hpp>

#include <yaml-cpp/yaml.h>

#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <fstream>

struct RobotPose {
    double x;
    double y;
    double z;
};

void print_robot_pose(struct RobotPose *pose)
{
    printf("Robot pose: x=%f, y=%f, z=%f\n", pose->x, pose->y, pose->z);
}

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
 * \brief Receives a message from a socket.
 *
 * This function will block until the next message is received, read its header (first 4 bytes)
 * and then read the content of the message and return it as a string.
 *
 * \param sock The socket on which to listen for the next message.
 * \return The received message as a std::string.
 */
std::string receive_one_message(boost::asio::local::stream_protocol::socket &sock)
{
    // Read Preamble
    uint32_t data_preamble[4];
    size_t length = sock.receive(boost::asio::buffer(data_preamble, 4));
    uint32_t receive_length = ntohl(*data_preamble);
    // Read Message
    char *data = new char[receive_length];
    length = sock.receive(boost::asio::buffer(data, receive_length));
    std::string data_string(data, length);

    return data_string;
}

/**
 * \brief Sends a message from a socket.
 *
 * \param sock The socket used to send the message.
 * \param str The string message to send.
 */
void send_one_message(boost::asio::local::stream_protocol::socket &sock, std::string str)
{
    // Send Preamble
    std::size_t response_size = str.size();
    uint32_t send_length = htonl(static_cast<uint32_t>(response_size));
    sock.send(boost::asio::buffer(&send_length, 4));
    // Send Message
    sock.send(boost::asio::buffer(str.data(), str.size()));
}

std::string generate_robots_positions(RobotPose *robots, int nb_robots)
{
    robots_positions_proto::RobotsPositions RobotsPositions_msg;
    for (int i = 0; i < nb_robots; i++)
    {
        robots_positions_proto::RobotPose *RobotPose_msg = RobotsPositions_msg.add_robot_pose();
        RobotPose_msg->set_x(robots[i].x / 100.0); // Distances in Viragh are in cm, DANCERS uses m in its protobuf messages.
        RobotPose_msg->set_y(robots[i].y / 100.0);
        RobotPose_msg->set_z(robots[i].z / 100.0);
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
 * \param PhysicsUpdate_msg A protobuf message of type PhysicsUpdate. Usually it is an empty message with type BEGIN, this function will fill the message and
 */
std::string generate_response(std::string robots_positions, physics_update_proto::PhysicsUpdate PhysicsUpdate_msg)
{
    // Change message's type to END
    PhysicsUpdate_msg.set_msg_type(physics_update_proto::PhysicsUpdate::END);
    // Fill the channel_data field with the compressed data from the physics simulation
    PhysicsUpdate_msg.set_robots_positions(gzip_compress(robots_positions));
    // Transform the response [protobuf] --> [string]
    std::string str_response;
    PhysicsUpdate_msg.SerializeToString(&str_response);

    return str_response;
}

/**
 * This ROS2 node has nothing to do with ROS, it is only to leverage colcon.
 * 
 * This connector's purpose is to have a limited footprint on the Viragh's simulator C code. Protobuf being a C++ library, it is difficult to incorporate it in Viragh's C simulator.
 * With this connector, we continue to use the protobuf paradigm of DANCERS. It receives the BEGIN protobuf message from the coordinator and send a small message to Viragh's simulator.
 * Then Viragh's simulator sends the robots positions and the connector format the protobuf message before sending it to the coordinator.
 * 
 * author: Théotime Balaguer
 * 
 */
class ViraghConnector : public rclcpp::Node
{
public:
    ViraghConnector() : Node("viragh_connector")
    {
        /* ---- ROS2 parameter ---- */
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

        /* ---- Load Configuration file ---- */
        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s\nA config file must be given in the launch file.", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }
        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        this->robots_number = config["robots_number"].as<int>();
        this->sync_window = config["sync_window"].as<int>();        // us
        this->step_length = config["phy_step_size"].as<int>();      // us

        if (sync_window % step_length != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
            exit(EXIT_FAILURE);
        }

        /* Temporary solution so that the obstacles are similar in viragh and ns-3. launch viragh simulator with "-obst obstacles/cosim_obstacles.default"*/
        std::ofstream f;
        f.open("/home/theotime/gits/flocking-simulator/obstacles/cosim_obstacles.default", std::ios::out);
        f << "[init]\n\nangle=0\n\n[obstacles]\n\n";
        for (auto building : config["buildings"])
        {
            std::string name = building["name"].as<std::string>();
            double x = building["x"].as<double>() * 100; // Multiply everything by 100 because lengths are in cm in Viragh's simulator
            double y = building["y"].as<double>() * 100;
            double z = (building["height"].as<double>() / 2) * 100;
            double size_x = building["size_x"].as<double>() * 100;
            double size_y = building["size_y"].as<double>() * 100;
            double height = building["height"].as<double>() * 100;
            f << name << ".point=" << x + size_x / 2 << " " << y + size_y / 2 << "\n";
            f << name << ".point=" << x + size_x / 2 << " " << y - size_y / 2 << "\n";
            f << name << ".point=" << x - size_x / 2 << " " << y - size_y / 2 << "\n";
            f << name << ".point=" << x - size_x / 2 << " " << y + size_y / 2 << "\n";
            f << "\n";
        }
        f.close();

        /* ---- Create socket (server) with Coordinator ---- */
        Socket *socket_coord;
        boost::asio::io_context io_context;
        socket_coord = new UDSSocket(io_context);
        socket_coord->accept(config["phy_uds_server_address"].as<std::string>(), 0);

        /* ---- Create socket (client) with Viragh's simulator ---- */
        Socket *socket_viragh;
        boost::asio::io_context io_context2;
        socket_viragh = new UDSSocket(io_context2);
        socket_viragh->connect(config["viragh_uds_server_address"].as<std::string>());

        std::string data;
        int array_length = 1 + 2 * this->robots_number*this->robots_number;
        double *array_to_transmit = new double[array_length];

        for (int i = 0; i < this->robots_number; i++)
        {
            for (int j = 0; j < this->robots_number; j++){
                array_to_transmit[1+ i * 2 * this->robots_number + 2*j] = -1.0;
                array_to_transmit[1+ i * 2 * this->robots_number + 2*j+1] = -1.0;
            }
        }

        while (true)
        {
            std::string received_data = gzip_decompress(socket_coord->receive_one_message());

            // Initialize empty protobuf message type [PhysicsUpdate]
            physics_update_proto::PhysicsUpdate PhysicsUpdate_msg;
            // Transform the message received from the UDS socket (string -> protobuf)
            PhysicsUpdate_msg.ParseFromString(received_data);

            std::cout << "Received message from Coordinator" << std::endl;

            array_to_transmit[0] = (double)this->step_length;

            if(!PhysicsUpdate_msg.ordered_neighbors().empty())
            {
                for (int i = 0; i < this->robots_number; i++)
                {
                    for (int j = 0; j < this->robots_number; j++){
                        array_to_transmit[1+ i * 2 * this->robots_number + 2*j] = -1.0;
                        array_to_transmit[1+ i * 2 * this->robots_number + 2*j+1] = -1.0;
                    }
                }
                ordered_neighbors_proto::OrderedNeighborsList neighbors_list_msg;
                neighbors_list_msg.ParseFromString(gzip_decompress(PhysicsUpdate_msg.ordered_neighbors()));

                std::cout << neighbors_list_msg.DebugString() << std::endl;

                for (int i = 0; i < neighbors_list_msg.ordered_neighbors_size(); i++)
                {
                    for (int j = 0; j < neighbors_list_msg.ordered_neighbors(i).neighborid_size(); j++)
                    {
                        array_to_transmit[1+ 2 * this->robots_number * i + 2*j] = neighbors_list_msg.ordered_neighbors(i).neighborid(j);
                        array_to_transmit[1+ 2 * this->robots_number * i + 2*j + 1] = neighbors_list_msg.ordered_neighbors(i).linkquality(j);
                    }
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Received a BEGIN message but empty neighbor information from the coordinator.");
            }

            //print array_to_transmit
            std::cout << "array_to_transmit: " << std::endl;
            for (int i = 0; i < 1 + 2 * this->robots_number*this->robots_number; i++)
            {
                std::cout << array_to_transmit[i] << " ";
            }
            std::cout << std::endl;

            std::cout << "array_length " << array_length << std::endl;
            socket_viragh->send_double_array(array_to_transmit, array_length); // phy_step_size in us
            // socket_viragh->send_one_message("wesh");
            std::cout << "Sent BEGIN message to Viragh" << std::endl;

            data = socket_viragh->receive_one_message();
            std::cout << "Received VIRAGH message:" << std::endl;

            print_robot_pose((RobotPose*)&data[0]);

            std::string robots_positions = generate_robots_positions((RobotPose*)&data[0], this->robots_number);

            // Generate the response message
            std::string response = gzip_compress(generate_response(robots_positions, PhysicsUpdate_msg));

            // Send the response in the UDS socket
            socket_coord->send_one_message(response);
            std::cout << "Sent message to Coordinator\n" << std::endl;

        }

    }
private:
    int robots_number;
    int sync_window;
    int step_length;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViraghConnector>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"

#include <protobuf_msgs/physics_update.pb.h>
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include <udp_tcp_socket.hpp>

#include <yaml-cpp/yaml.h>

#include "boost/filesystem.hpp"
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

    // std::cout << RobotsPositions_msg.DebugString() << std::endl;

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
std::string generate_response(std::string robots_positions, physics_update_proto::PhysicsUpdate PhysicsUpdate_msg, bool targets_reached)
{
    // Change message's type to END
    PhysicsUpdate_msg.set_msg_type(physics_update_proto::PhysicsUpdate::END);
    // Fill the channel_data field with the compressed data from the physics simulation
    PhysicsUpdate_msg.set_robots_positions(gzip_compress(robots_positions));

    PhysicsUpdate_msg.set_targets_reached(targets_reached);

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
        param_desc.description = "Path to the folder containing Viragh's simulator.";
        this->declare_parameter("viragh_path", "~", param_desc);
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

            this->previous_end_time = std::chrono::system_clock::now();

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
            this->compute_time_output.open(this->m_computation_time_file.c_str(), std::ios::out);
            this->compute_time_output << "Time[us]"
                    << std::endl;
            this->compute_time_output.close();
        }


        this->robots_number = config["robots_number"].as<int>();
        this->sync_window = config["sync_window"].as<int>();        // us
        this->step_length = config["phy_step_size"].as<int>();      // us
        this->max_neighbors = config["max_neighbors"].as<int>();

        this->current_sim_time = rclcpp::Time(0);
        this->simulation_length = rclcpp::Time(config["simulation_length"].as<int64_t>() * 1e9);    // from s to ns

        if (sync_window % step_length != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
            exit(EXIT_FAILURE);
        }

        /* Temporary solution so that the obstacles are similar in viragh and ns-3. Must launch viragh simulator with "-obst obstacles/cosim_obstacles.default"*/
        std::ofstream f;
        f.open(this->get_parameter("viragh_path").get_parameter_value().get<std::string>()+"/obstacles/cosim_obstacles.default", std::ios::out);
        f << "[init]\n\nangle=0\n\n[obstacles]\n\n";
        for (auto building : config["buildings"])
        {
            std::string name = building["name"].as<std::string>();
            double x = building["x"].as<double>() * 100; // Multiply everything by 100 because lengths are in cm in Viragh's simulator (wtf)
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

        
        // Copy the number of agents of the global configuration file to the one of viragh's simulator
        std::string initfile = this->get_parameter("viragh_path").get_parameter_value().get<std::string>()+"/parameters/initparams.dat";
        std::ifstream file_in(initfile);
        std::ofstream file_out("temp.txt");
        
        if (!file_in.is_open() || !file_out.is_open()) {
            std::cout << "Error opening file!" << std::endl;
            exit(EXIT_FAILURE);
        }

        std::string line;
        while (std::getline(file_in, line)) {
            if (line.rfind("NumberOfAgents=", 0) == 0) { // Line starts with "NumberOfAgents"
                line = "NumberOfAgents="+std::to_string(this->robots_number); // Modify the line
            }
            if (line.rfind("Length=", 0) == 0) { // Line starts with "NumberOfAgents"
                line = "Length="+std::to_string(config["simulation_length"].as<double>()); // Modify the line
            }
            file_out << line << std::endl;
        }
        file_in.close();
        file_out.close();

        // Replace original file with modified file
        std::remove(initfile.c_str());
        std::rename("temp.txt", initfile.c_str());

        // Copy the Arena size and location to viragh's simulator config files
        std::string flockingfile = this->get_parameter("viragh_path").get_parameter_value().get<std::string>()+"/parameters/flockingparams.dat";
        file_in = std::ifstream(flockingfile);
        file_out = std::ofstream("temp.txt");
        
        if (!file_in.is_open() || !file_out.is_open()) {
            std::cout << "Error opening file!" << std::endl;
            exit(EXIT_FAILURE);
        }

        while (std::getline(file_in, line)) {
            if (line.rfind("ArenaRadius=", 0) == 0) { // Line starts with "ArenaRadius"
                line = "ArenaRadius="+std::to_string(config["arena_radius"].as<int>()*100); // Modify the line
            }
            if (line.rfind("ArenaCenterX=", 0) == 0) { // Line starts with "ArenaCenterX"
                line = "ArenaCenterX="+std::to_string(config["arena_center_x"].as<int>()*100); // length are in cm in viragh's simulator
            }
            if (line.rfind("ArenaCenterY=", 0) == 0) { // Line starts with "ArenaCenterY"
                line = "ArenaCenterY="+std::to_string(config["arena_center_y"].as<int>()*100); // length are in cm in viragh's simulator
            }
            file_out << line << std::endl;
        }

        file_in.close();
        file_out.close();

        // Replace original file with modified file
        std::remove(flockingfile.c_str());
        std::rename("temp.txt", flockingfile.c_str());


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
                array_to_transmit[1+ i * 2 * this->robots_number + 2*j] = -1.0;         // neighbor ID
                array_to_transmit[1+ i * 2 * this->robots_number + 2*j+1] = -1.0;       // neighbor Received
            }
        }

        while (this->current_sim_time < this->simulation_length || this->simulation_length == rclcpp::Time(0.0))
        {
            std::string received_data = gzip_decompress(socket_coord->receive_one_message());

            // Initialize empty protobuf message type [PhysicsUpdate]
            physics_update_proto::PhysicsUpdate PhysicsUpdate_msg;
            // Transform the message received from the UDS socket (string -> protobuf)
            PhysicsUpdate_msg.ParseFromString(received_data);

            // std::cout << "Received message from Coordinator" << std::endl;

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
                    int agent_id = neighbors_list_msg.ordered_neighbors(i).agentid();
                    for (int j = 0; j < neighbors_list_msg.ordered_neighbors(i).neighborid_size(); j++)
                    {
                        array_to_transmit[1+ 2 * this->robots_number * agent_id + 2*j] = neighbors_list_msg.ordered_neighbors(i).neighborid(j);
                        array_to_transmit[1+ 2 * this->robots_number * agent_id + 2*j + 1] = neighbors_list_msg.ordered_neighbors(i).linkquality(j);
                    }
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Received a BEGIN message but empty neighbor information from the network simulator.");
            }

            // print array_to_transmit
            // std::cout << "array_to_transmit: " << std::endl;
            // for (int i = 0; i < 1 + 2 * this->robots_number*this->robots_number; i++)
            // {
            //     std::cout << array_to_transmit[i] << " ";
            // }
            // std::cout << std::endl;


            socket_viragh->send_double_array(array_to_transmit, array_length); // phy_step_size in us
            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

            // We receive an array from viragh-s simulator in the form :
            // [targets_reached (int), coords[0][0] (double), coords[0][1], coords[0][2], coords[1][0], ... , coords[N][2] (double)]
            data = socket_viragh->receive_one_message();

            if (this->save_compute_time)
                {
                    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                    int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    this->compute_time_output.open(m_computation_time_file.c_str(), std::fstream::app);
                    this->compute_time_output << wait << "\n";
                    this->compute_time_output.close();
                }

            // print_robot_pose((RobotPose*)&data[0]);

            // We create a pointer of RobotPose pointing to the first byte after the "target_reached" byte
            std::string robots_positions = generate_robots_positions((RobotPose*)&data[1], this->robots_number);

            bool targets_reached = false;
            if((int)data.c_str()[0] == 1)
            {
                targets_reached = true;
                std::cout << "Targets reached" << std::endl;
            }        

            // Generate the response message
            std::string response = gzip_compress(generate_response(robots_positions, PhysicsUpdate_msg, targets_reached));

            // Send the response in the UDS socket
            socket_coord->send_one_message(response);
            // std::cout << "Sent message to Coordinator\n" << std::endl;

            this->current_sim_time += std::chrono::microseconds(this->step_length);
        }
        RCLCPP_INFO(this->get_logger(), "Simulation finished.");
        exit(EXIT_SUCCESS);
    }
private:
    int robots_number;
    int sync_window;
    int step_length;
    int max_neighbors;

    rclcpp::Time current_sim_time;
    rclcpp::Time simulation_length;

    // saving of computation time
    bool save_compute_time;
    std::chrono::system_clock::time_point previous_end_time;
    std::string m_computation_time_file;
    std::ofstream compute_time_output;

    std::string ros_ws_path;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViraghConnector>());
    rclcpp::shutdown();
    return 0;
}
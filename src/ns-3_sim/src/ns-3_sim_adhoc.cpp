/**
 * This file is the network simulator based on ns-3 for the DANCERS co-simulator.
 *
 * WIFI : Ad-hoc mode
 */

#include <iostream>

#include "boost/filesystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "boost/asio.hpp"
#include "boost/iostreams/filtering_streambuf.hpp"
#include "boost/iostreams/copy.hpp"
#include "boost/iostreams/filter/gzip.hpp"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/node-list.h"
#include "ns3/buildings-module.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/tap-bridge-module.h"

#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/spectrum-wifi-helper.h"

#include "ns3/udp-client-server-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"

#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/robots_positions.pb.h"

#include <yaml-cpp/yaml.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("UAV_fleet_adhoc_network");

static std::string m_computation_time_file;

// Global variables for use in callbacks.
double g_signalDbmAvg;                        //!< Average signal power [dBm]
double g_noiseDbmAvg;                         //!< Average noise power [dBm]
uint32_t g_samples;                           //!< Number of samples
std::map<Ipv4Address, Ptr<Node>> ip_node_map; //!< A map of a NS-3 Node object to its corresponding IP address

/**
 * \brief Updates the position of a node.
 *
 * \param node A pointer to the target node.
 * \param position A ns3::Vector containing the [x, y, z] position of the drone, with respect to the origin.
 */
void SetNodePosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    mobility->SetPosition(position);

    // std::cout << node->GetId() << ": " << position << std::endl;
}

/**
 * \brief Compresses a string with the zip protocol.
 *
 * \param data The string to compress.
 * \return The compressed string.
 */
static std::string
gzip_compress(const std::string &data)
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
static std::string
gzip_decompress(const std::string &data)
{
    std::stringstream compressed(data);
    std::stringstream decompressed;

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(compressed);
    boost::iostreams::copy(in, decompressed);

    return decompressed.str();
}

class MySocket
{
public:
    virtual void accept(const std::string &host, unsigned short port) = 0;
    virtual void send_one_message(const std::string &message) = 0;
    virtual std::string receive_one_message() = 0;
    virtual void close() = 0;
};

class MyUDSSocket : public MySocket
{
public:
    MyUDSSocket(boost::asio::io_context &io_context) : socket_(io_context) {}
    void accept(const std::string &host, unsigned short port) override
    {
        ::unlink(host.c_str());
        boost::asio::local::stream_protocol::acceptor acceptor(socket_.get_executor(), boost::asio::local::stream_protocol::endpoint(host));
        acceptor.accept(socket_);
    }
    void send_one_message(const std::string &message) override
    {
        // Send Preamble
        std::size_t response_size = message.size();
        uint32_t send_length = htonl(static_cast<uint32_t>(response_size));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(message.data(), message.size()));
    }
    std::string receive_one_message() override
    {
        // Read Preamble
        uint32_t data_preamble[4];
        size_t length = socket_.receive(boost::asio::buffer(data_preamble, 4));
        uint32_t receive_length = ntohl(*data_preamble);
        // Read Message
        char *data = new char[receive_length];
        length = socket_.receive(boost::asio::buffer(data, receive_length));
        std::string data_string(data, length);
        return data_string;
    }
    void close() override
    {
        socket_.close();
    }

private:
    boost::asio::local::stream_protocol::socket socket_;
};

class MyTCPSocket : public MySocket
{
public:
    MyTCPSocket(boost::asio::io_context &io_context) : socket_(io_context) {}
    void accept(const std::string &host, unsigned short port) override
    {
        boost::asio::ip::tcp::acceptor acceptor(socket_.get_executor(), boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address(host), port));
        acceptor.accept(socket_);
    }
    void send_one_message(const std::string &message) override
    {
        // Send Preamble
        std::size_t response_size = message.size();
        uint32_t send_length = htonl(static_cast<uint32_t>(response_size));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(message.data(), message.size()));
    }
    std::string receive_one_message() override
    {
        // Read Preamble
        uint32_t data_preamble[4];
        size_t length = socket_.receive(boost::asio::buffer(data_preamble, 4));
        uint32_t receive_length = ntohl(*data_preamble);
        // Read Message
        char *data = new char[receive_length];
        length = socket_.receive(boost::asio::buffer(data, receive_length));
        std::string data_string(data, length);
        return data_string;
    }
    void close() override
    {
        socket_.close();
    }

private:
    boost::asio::ip::tcp::socket socket_;
};

/**
 * \brief Receives a message from a socket.
 *
 * This function will block until the next message is received, read its header (first 4 bytes)
 * and then read the content of the message and return it as a string.
 *
 * \param sock The socket on which to listen for the next message.
 * \return The received message as a std::string.
 */
std::string
receive_one_message(boost::asio::local::stream_protocol::socket &sock)
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

/**
 * \brief Generates the final protobuf message of protobuf_msgs/NetworkUpdate.
 *
 * Usually, this function will be passed the [NetworkUpdate] protobuf message with message-type BEGIN from the Network coordinator.
 * It will fill the message with the Bit Error Rate (BER) of each packet ,
 * change the message-type to END, string-serialize the protobuf message and return it to the network coordinator.
 *
 * \param PhysicsUpdate_msg A protobuf message of type NetworkUpdate.
 * \return A string-serialized version of the updated protobuf message.
 */
std::string
generate_response(network_update_proto::NetworkUpdate NetworkUpdate_msg)
{
    // Change message type to "END"
    NetworkUpdate_msg.set_msg_type(network_update_proto::NetworkUpdate::END);
    // Transform the response to std::string
    std::string str_response;
    NetworkUpdate_msg.SerializeToString(&str_response);

    return str_response;
}

/**
 * We declare a ROS2 Node here. It takes the form of a class which constructor will be called by rclcpp::spin().
 * The use of ROS for the NS3 simulator is essentially to harmonize the code, and allow the use of a standard logging system (RCLCPP_INFO, etc.).
 * We could very well have standard NS-3 code here.
 */
class Ns3Simulation : public rclcpp::Node
{
public:
    Ns3Simulation() : Node("ns3_simulation")
    {
        // ========================= PARAMETER READING AND LOG FILE INITIALIZATION =========================

        // Declare parameters for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using a ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        this->save_compute_time = config["save_compute_time"].as<bool>();

        // Prepare the headers of the csv file for computation time saving
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
                temp_path = "./data/" + experience_name + "/network_sim_data" + std::to_string(i) + ".csv";
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
            this->output_file_compute_time.open(m_computation_time_file.c_str(), std::ios::out);
            this->output_file_compute_time << "Time[us]" << std::endl;
            this->output_file_compute_time.close();
        }

        // ========================= NS3 CONFIGURATION =========================

        // Set the seed for the random number generator
        SeedManager::SetRun(config["ns3_seed"].as<int>());

        double frequency = 2.4e9;         // operating frequency in Hz
        Time timeRes = MilliSeconds(100); // time resolution
        std::string errorModelType = config["error_model_type"].as<std::string>();
        std::string wifiType = config["wifi_type"].as<std::string>();
        std::string phyMode = config["phy_mode"].as<std::string>(); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
        uint32_t numNodes = config["robots_number"].as<int>();
        uint32_t step_size = config["net_step_size"].as<uint32_t>(); // in milliseconds

        for (int i = 0; i < numNodes; i++)
        {
            this->numReceivedPackets.push_back(0);
        }

        // Create the nodes
        NodeContainer nodes;
        nodes.Create(numNodes);

        // **************** BUILDINGS MODULE ****************

        // [Buildings] -- Define buildings with their center (x, y), and their size in the three dimensions : (size_x, size_y, height)
        std::vector<Ptr<Building>> buildings;
        for (auto building : config["buildings"])
        {
            double x_min = building["x"].as<int>() - (building["size_x"].as<int>() / 2);
            double x_max = building["x"].as<int>() + (building["size_x"].as<int>() / 2);
            double y_min = building["y"].as<int>() - (building["size_y"].as<int>() / 2);
            double y_max = building["y"].as<int>() + (building["size_y"].as<int>() / 2);
            double z_min = 0.0;
            double z_max = building["height"].as<int>();
            Ptr<Building> build = CreateObject<Building>();
            build->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
            build->SetBuildingType(Building::Office);
            build->SetExtWallsType(Building::ConcreteWithoutWindows);
            build->SetNFloors(1);
            build->SetNRoomsX(1);
            build->SetNRoomsY(1);
            buildings.push_back(build);
            RCLCPP_DEBUG(this->get_logger(), "Created a building with corners (%f, %f, %f, %f)", x_min, x_max, y_min, y_max);
        }

        // **************** MOBILITY MODULE ****************

        // For all the nodes create a ConstantPositionMobilityModel to initialize their position.
        // Assign different positions for each robot as these values will be overwritten at first step of simulation anyway
        for (uint32_t i = 0; i < numNodes; i++)
        {
            Ptr<MobilityModel> nodeMob;
            nodeMob = CreateObject<ConstantPositionMobilityModel>();
            nodeMob->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(i, i, 0.5));
            nodes.Get(i)->AggregateObject(nodeMob);
        }

        // **************** PROPAGATION MODULE ****************

        SpectrumWifiPhyHelper spectrumPhy;
        YansWifiPhyHelper yansPhy;

        if (wifiType == "yans_default")
        {
            YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
            yansPhy.SetChannel(wifiChannel.Create());
        }
        else if (wifiType == "yans_log_distance")
        {
            YansWifiChannelHelper wifiChannel;
            wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                           "Exponent", DoubleValue(3.0));
            wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
            yansPhy.SetChannel(wifiChannel.Create());
        }
        else if (wifiType == "yans_hybrid_buildings")
        {
            YansWifiChannelHelper channel;
            channel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                       "Frequency",            // Additional loss for each internal wall [dB]
                                       DoubleValue(frequency), // Default 2.4e9
                                       "ShadowSigmaExtWalls",  // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                                       DoubleValue(5.0),       // Default 5
                                       "ShadowSigmaIndoor",    // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                                       DoubleValue(8.0),       // Default 8
                                       "ShadowSigmaOutdoor",   // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                                       DoubleValue(7.0));
            channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
            yansPhy.SetChannel(channel.Create());
            yansPhy.Set("TxPowerStart", DoubleValue(22));
            yansPhy.Set("TxPowerEnd", DoubleValue(22));
        }
        else if (wifiType == "spectrum_3GPP_V2V_urban_channel")
        {
            Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

            // create the channel condition model
            Ptr<ChannelConditionModel> condModel;
            condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
            condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100))); // Channel conditions are evaluated every 100 ms

            // create the propagation loss model and add it to the channel condition
            Ptr<ThreeGppPropagationLossModel> propagationLossModel;
            propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
            propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
            propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(true));
            propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(condModel));
            spectrumChannel->AddPropagationLossModel(propagationLossModel);

            // Create the delay model and add it to the channel condition
            Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
            spectrumChannel->SetPropagationDelayModel(delayModel);

            spectrumPhy.SetChannel(spectrumChannel);
            spectrumPhy.SetErrorRateModel(errorModelType);
            spectrumPhy.Set("TxPowerStart", DoubleValue(0.5)); // dBm  (1.26 mW)
            spectrumPhy.Set("TxPowerEnd", DoubleValue(0.5));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported wifiType : %s\n Options are %s, %s, %s, %s", wifiType.c_str(), "yans_default", "yans_log_distance", "yans_hybrid_buildings", "spectrum_3GPP_V2V_urban_channel");
            exit(EXIT_FAILURE);
        }

        // **************** WIFI MODULE ****************
        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211g);
        WifiMacHelper mac;

        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode));
        // wifi.SetRemoteStationManager("ns3::IdealWifiManager");

        NetDeviceContainer devices;

        mac.SetType("ns3::AdhocWifiMac");

        if (wifiType == "yans_default" || wifiType == "yans_log_distance" || wifiType == "yans_hybrid_buildings")
        {
            devices = wifi.Install(yansPhy, mac, nodes);
        }
        else if (wifiType == "spectrum_3GPP_V2V_urban_channel")
        {
            devices = wifi.Install(spectrumPhy, mac, nodes);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported wifiType : %s\n Options are %s, %s, %s, %s", wifiType.c_str(), "yans_default", "yans_log_distance", "yans_hybrid_buildings", "spectrum_3GPP_V2V_urban_channel");
            exit(EXIT_FAILURE);
        }

        Config::Set(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
            BooleanValue(true));

        // [Buildings] -- Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
        BuildingsHelper::Install(nodes);

        // **************** TAP-BRIDGE MODULE ****************
        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue("UseLocal"));
        char buffer[10];
        for (uint32_t i = 0; i < numNodes; i++)
        {
            sprintf(buffer, "wifi_tap%d", i + 1);
            tapBridge.SetAttribute("DeviceName", StringValue(buffer));
            tapBridge.Install(nodes.Get(i), devices.Get(i));
        }

        for (auto netDevice = devices.Begin(); netDevice != devices.End(); ++netDevice)
        {
            (*netDevice)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this, (*netDevice)->GetNode()->GetId()));
        }

        // **************** UDS SOCKET FOR NETWORK COORDINATOR ****************
        // // Create and connect UDS Socket
        // boost::asio::io_service io_service;
        // ::unlink("/tmp/net_server_socket");
        // boost::asio::local::stream_protocol::endpoint ep("/tmp/net_server_socket");
        // boost::asio::local::stream_protocol::acceptor acceptor(io_service, ep);
        // boost::asio::local::stream_protocol::socket socket(io_service);
        // acceptor.accept(socket);
        // RCLCPP_INFO(this->get_logger(), "finished setting up UDS socket");

        MySocket *socket;
        boost::asio::io_context io_context;
        if (config["net_use_uds"].as<bool>())
        {
            socket = new MyUDSSocket(io_context);
            socket->accept(config["net_uds_server_address"].as<std::string>(), 0);
        }
        else
        {
            socket = new MyTCPSocket(io_context);
            socket->accept(config["net_ip_server_address"].as<std::string>(), config["net_ip_server_port"].as<unsigned short>());
        }

        std::cout << "Network simulator ready" << std::endl;

        // **************** MAIN SIMULATION LOOP ****************
        while (true)
        {

            // Wait until reception of a message on the UDS socket
            std::string received_data = gzip_decompress(socket->receive_one_message());

            // Initialize empty protobuf message type [NetworkUpdate]
            network_update_proto::NetworkUpdate NetworkUpdate_msg;
            // Transform the message received from the UDS socket [string] -> [protobuf]
            NetworkUpdate_msg.ParseFromString(received_data);

            // Read the "physical" information transmitted by the NetworkCoordinator, and update the node's positions
            // Also verifies that the number of nodes sent by the NetworkCoordinator corresponds to the number of existing nodes in NS-3
            rclcpp::Clock clock;
            robots_positions_proto::RobotsPositions robots_positions_msg;
            if (!NetworkUpdate_msg.robots_positions().empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                robots_positions_msg.ParseFromString(gzip_decompress(NetworkUpdate_msg.robots_positions()));

                // RCLCPP_INFO(this->get_logger(), "Received robots positions from Coordinator: %s", robots_positions_msg.DebugString().c_str());

                // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                // Then, update the node's positions (orientation is ignored for now)
                if (nodes.GetN() != (uint32_t)robots_positions_msg.robot_pose_size())
                {
                    if (this->get_parameter("verbose").as_bool())
                    {
                        // RCLCPP_WARN_THROTTLE(this->get_logger(),
                        //                      clock,
                        //                      1000, // ms
                        //                      "Network simulator received position information of %i robots but NS-3 has %zu nodes.",
                        //                      robots_positions_msg.robot_pose_size(),
                        //                      ip_node_map.size());
                        RCLCPP_WARN(this->get_logger(), "Network simulator received position information of %i robots but NS-3 has %zu nodes.", robots_positions_msg.robot_pose_size(), ip_node_map.size());
                    }
                }
                else
                {
                    for (uint32_t i = 0; i < nodes.GetN(); i++)
                    {
                        Vector pos;
                        pos.x = robots_positions_msg.robot_pose(i).x();
                        pos.y = robots_positions_msg.robot_pose(i).y();
                        pos.z = robots_positions_msg.robot_pose(i).z();
                        SetNodePosition(nodes.Get(i), pos);
                    }
                }
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
            }

            // Once all the events are scheduled, advance W time in the simulation and stop
            Simulator::Stop(MicroSeconds(step_size));

            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            Simulator::Run();
            if (this->save_compute_time)
            {
                std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                this->output_file_compute_time.open(m_computation_time_file.c_str(), std::fstream::app);
                this->output_file_compute_time << wait << "\n";
                this->output_file_compute_time.close();
            }

            if (this->get_parameter("verbose").get_parameter_value().get<bool>())
            {
                // 32 = Green :)
                RCLCPP_INFO(this->get_logger(), "\x1b[32m[%f] Advanced %i microseconds\x1b[0m", Simulator::Now().GetSeconds(), step_size);
            }
            std::string response = gzip_compress(generate_response(NetworkUpdate_msg));

            // Send the response to the network coordinator
            socket->send_one_message(response);
        }
    }

private:
    bool save_compute_time;
    std::ofstream output_file_compute_time;
    void node_receive_clbk(uint32_t nodeId);
    std::vector<int> numReceivedPackets;
};

void Ns3Simulation::node_receive_clbk(uint32_t nodeId)
{
    this->numReceivedPackets[nodeId] += 1;
}

/**
 * \brief The main function, spins the ROS2 Node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ns3Simulation>());
    rclcpp::shutdown();
    return 0;
}

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
#include "ns3/aodv-helper.h"
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
#include "ns3/csma-helper.h"

#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/spectrum-wifi-helper.h"

#include "ns3/udp-client-server-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/flow-monitor-helper.h"

#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include "udp_tcp_socket.hpp"

#include <yaml-cpp/yaml.h>

// Define a port on which the nodes listen and talk
#define PORT 80

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("UAV_fleet_adhoc_network");

static std::string m_output_file;
static std::string m_output_file_packets_received;

static Ptr<ThreeGppPropagationLossModel> m_propagationLossModel;      //!< the PropagationLossModel object
static Ptr<ThreeGppSpectrumPropagationLossModel> m_spectrumLossModel; //!< the SpectrumPropagationLossModel object
static Ptr<ChannelConditionModel> m_condModel;                        //!< the ChannelConditionModel object

// Global variables for use in callbacks.
double g_signalDbmAvg;                        //!< Average signal power [dBm]
double g_noiseDbmAvg;                         //!< Average noise power [dBm]
uint32_t g_samples;                           //!< Number of samples
std::map<Ipv4Address, Ptr<Node>> ip_node_map; //!< A map of a NS-3 Node object to its corresponding IP address

// /**
//  * Monitor sniffer Rx trace
//  *
//  * \param packet The sensed packet.
//  * \param channelFreqMhz The channel frequency [MHz].
//  * \param txVector The Tx vector.
//  * \param aMpdu The aMPDU.
//  * \param signalNoise The signal and noise dBm.
//  * \param staId The STA ID.
//  */
// void
// MonitorSniffRx(Ptr<const Packet> packet,
//                uint16_t channelFreqMhz,
//                WifiTxVector txVector,
//                MpduInfo aMpdu,
//                SignalNoiseDbm signalNoise,
//                uint16_t staId)

// {
//     g_samples++;
//     g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
//     g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);
//     std::cout << "Sniffed Rx Signal Noise (dBm) " << signalNoise.signal << " / " << signalNoise.noise << std::endl;
// }

Vector
GetNodePosition(Ptr<ns3::Node> node)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    return mobility->GetPosition();
}

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
 * \brief Get the address of the first ipv4 interface of a node.
 *
 * \param node A pointer to the target node.
 */
Ipv4Address
GetAddressOfNode(Ptr<Node> node)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
    Ipv4Address addri = iaddr.GetAddress();
    return addri;
}

/**
 * \brief Print the positions of two nodes every given period of simulated time
 *
 * \param period Simulated time between each message
 * \param model A Ptr to the BuildingsPropagationLossModel
 * \param sender A Ptr to the sending Node
 * \param receiver A Ptr to the receiving Node
 */
void printPosition(Time period, Ptr<ns3::Node> sender, Ptr<ns3::Node> receiver)
{
    Vector pos_sender = GetNodePosition(sender);
    Vector pos_receiver = GetNodePosition(receiver);
    std::cout << "Sender: (" << pos_sender.x << " ; " << pos_sender.y << ")\nReceiver: (" << pos_receiver.x << " ; " << pos_receiver.y << ")\n";
    Simulator::Schedule(period,
                        &printPosition,
                        period,
                        sender,
                        receiver);
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
    char data[receive_length];
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
    // static_cast<uint32_t>(response_size);
    uint32_t send_length = htonl(response_size);
    sock.send(boost::asio::buffer(&send_length, 4));
    // Send Message
    sock.send(boost::asio::buffer(str.data(), str.size()));
}

/**
 * Send an IP packet.
 *
 * \param sender The Node id of the source node.
 * \param ip_dest The ipv4 address of the destination as ns3::Ipv4Address object
 * \param pkt_size The size of the packet (in bytes?).
 */
static void
SendPacket(uint32_t sender, Ipv4Address ip_dest, uint32_t pkt_size)
{
    // Get the pointers to the nodes
    Ptr<Node> nodeSender = NodeList::GetNode(sender);
    // At initialisation, each node was aggregated to a Socket object, get this Socket
    Ptr<ns3::Socket> socket = nodeSender->GetObject<ns3::Socket>();
    InetSocketAddress remote = InetSocketAddress(ip_dest, PORT);
    if (socket->Connect(remote) != -1)
    {
        NS_LOG_INFO("socket connected");
    }
    else
    {
        NS_LOG_INFO("Error when connecting socket");
    }

    // Send a "fake" packet in the socket (second argument is the socket control flags)
    if (socket->Send(Create<Packet>(pkt_size)))
    {
        NS_LOG_INFO("One packet sent");
    }
    else
    {
        NS_LOG_INFO("Error when sending a packet");
    }
}

/**
 * Function called when a packet is received.
 *
 * \param socket The receiving socket.
 */
void ReceivePacket(Ptr<ns3::Socket> socket)
{
    while (socket->Recv())
    {
        // NS_LOG_UNCOND("Received one packet!");
        g_samples += 1;
        std::fstream file_received_packets;
        file_received_packets.open(m_output_file_packets_received.c_str(), std::fstream::app);
        file_received_packets << Simulator::Now().GetMicroSeconds() << "," << g_samples << std::endl;
    }
}

Ptr<ns3::Socket>
SetupPacketReceive(Ptr<Node> node)
{
    TypeId tid = TypeId::LookupByName("ns3::PacketSocketFactory");
    Ptr<ns3::Socket> sink = ns3::Socket::CreateSocket(node, tid);
    sink->Bind();
    sink->SetRecvCallback(MakeCallback(ReceivePacket));
    return sink;
}

std::string
generate_neighbors_msg(std::map<uint32_t, std::map<uint32_t, double>> neighbors){
    ordered_neighbors_proto::OrderedNeighborsList ordered_neighbors_msg;
    for (auto const& x : neighbors)
    {
        ordered_neighbors_proto::OrderedNeighbors* neighbor = ordered_neighbors_msg.add_ordered_neighbors();
        neighbor->set_agentid(x.first);
        for (auto const& y : x.second)
        {
            neighbor->add_neighborid(y.first);
            neighbor->add_linkquality(y.second);
        }
    }
    std::string str_response;
    ordered_neighbors_msg.SerializeToString(&str_response);
    return str_response;
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
generate_response(network_update_proto::NetworkUpdate NetworkUpdate_msg, std::map<uint32_t, std::map<uint32_t, double>> neighbors)
{
    // Change message type to "END"
    NetworkUpdate_msg.set_msg_type(network_update_proto::NetworkUpdate::END);
    NetworkUpdate_msg.set_ordered_neighbors(gzip_compress(generate_neighbors_msg(neighbors)));
    std::string str_response;
    NetworkUpdate_msg.SerializeToString(&str_response);

    return str_response;
}

void RxPowerCallback(Ptr<const Packet> packet, double rxPowerDbm)
{
    std::cout << "Received packet with power: " << rxPowerDbm << " dBm" << std::endl;
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

        // Declare two parameters for this ros2 node
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

        // vvvvvvvv DATA SAVING vvvvvvvv
        std::string experience_name = config["experience_name"].as<std::string>();
        if (this->save_compute_time)
        {
            // Create a folder based on the experience name, if not existant already
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
            while (m_output_file.empty())
            {
                temp_path = "./data/" + experience_name + "/network_sim_data" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    m_output_file = temp_path;
                }
            }

            // initialize the output file with headers
            this->f.open(m_output_file.c_str(), std::ios::out);
            this->f << "Time[us]"
                    << std::endl;
            this->f.close();
        }

        // Define the output file name, based on the existing files in the experience folder (incremental)
        std::string temp_path;
        int i = 1;
        while (m_output_file_packets_received.empty())
        {
            temp_path = "./data/co-sim_validation/" + experience_name + "/received_pkts_" + std::to_string(i) + ".csv";
            if (boost::filesystem::exists(temp_path))
            {
                i++;
            }
            else
            {
                m_output_file_packets_received = temp_path;
            }
        }

        // initialize the output file with headers
        this->file_received_packets.open(m_output_file_packets_received.c_str(), std::ios::out);
        this->file_received_packets << "nsec" << "," << "counter"
                                    << std::endl;
        this->file_received_packets.close();

        // ^^^^^^^^ DATA SAVING ^^^^^^^^

        // ========================= NS3 CONFIGURATION =========================

        // Set the seed for the random number generator
        SeedManager::SetRun(config["ns3_seed"].as<int>());

        double frequency = 5.2e9;         // operating frequency in Hz
        Time timeRes = MilliSeconds(100); // time resolution
        std::string errorModelType = "ns3::NistErrorRateModel";
        std::string wifiType = config["wifi_type"].as<std::string>();
        std::string phyMode("ErpOfdmRate54Mbps"); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
        uint32_t numNodes = config["robots_number"].as<int>();
        this->nNodes = numNodes;
        Time step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        Time simEndTime = Seconds(0.0);                                         // simulation time (s)
        uint32_t payloadSize = 20;                                             // for UDP (1000 bytes IPv4)
        Time currTime = MicroSeconds(0);                                       // us
                                                                               // Old parameters
        // double txPow_dbm = 30.0;            // tx power in dBm
        // double noiseFigure = 9.0;           // noise figure in dB
        // std::string scenario = "V2V-Urban"; // 3GPP propagation scenario, V2V-Urban or V2V-Highway
        // double vScatt = 0;                  // maximum speed of the vehicles in the scenario [m/s]
        // double subCarrierSpacing = 60e3;    // subcarrier spacing in kHz
        // uint32_t numRb = 275;               // number of resource blocks
        // double datarate = 14.4;            //The data rate associated with the configuration index 9 of examples/wireless/wifi-spectrum-per-example.cc
        // double distance = 3;                    // Distance in m between the nodes at initialization
        // bool wifiVerbose = false;               // Turns on/off logging for ALL the wifi-related ns3 modules
        // bool tracing = false;
        // Fetch the length of one step in the robotics simulator (in ms) and the number of iterations between two synchronization
        // and compute the window size, i.e. the time between two synchronizations with the robotics simulator

        // Initializations for each agent
        for (uint32_t i = 0; i < numNodes; i++)
        {
            this->numReceivedPackets.push_back(0);
            // for (uint32_t j = 0; j < numNodes; j++)
            // {
            //     this->neigh_last_received[i].push_back(Time(0.0));
            //     if (i == j)
            //     {
            //         this->neigh_pathloss[i].push_back(-1.0);
            //     }
            //     else
            //     {
            //         this->neigh_pathloss[i].push_back(0.0);
            //     }
            // }
            //     this->ordered_neighbors_list_msg.add_ordered_neighbors()->set_agentid(i);
        }

        // Create the nodes
        this->nodes.Create(numNodes);

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
            build->SetExtWallsType(Building::ConcreteWithWindows);
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
            this->nodes.Get(i)->AggregateObject(nodeMob);
        }

        // **************** PROPAGATION MODULE ****************

        SpectrumWifiPhyHelper spectrumPhy;
        YansWifiPhyHelper wifiPhy;

        if (wifiType == "yans_default")
        {
            YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
            wifiPhy.SetChannel(wifiChannel.Create());
        }
        else if (wifiType == "yans_log_distance")
        {
            YansWifiChannelHelper wifiChannel;
            wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                           "Exponent", DoubleValue(3.0));
            wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
            wifiPhy.SetChannel(wifiChannel.Create());
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
            wifiPhy.SetChannel(channel.Create());
            wifiPhy.Set("TxPowerStart", DoubleValue(22));
            wifiPhy.Set("TxPowerEnd", DoubleValue(22));
        }
        else if (wifiType == "spectrum_3GPP_V2V_urban_channel")
        {
            Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

            // create the channel condition model
            m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
            m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

            // create the propagation loss model and add it to the channel condition
            m_propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
            m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
            m_propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(true));
            m_propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));
            spectrumChannel->AddPropagationLossModel(m_propagationLossModel);

            // Create the delay model and add it to the channel condition
            Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
            spectrumChannel->SetPropagationDelayModel(delayModel);

            spectrumPhy.SetChannel(spectrumChannel);
            spectrumPhy.SetErrorRateModel(errorModelType);
            spectrumPhy.Set("TxPowerStart", DoubleValue(10)); // dBm  (1.26 mW)
            spectrumPhy.Set("TxPowerEnd", DoubleValue(10));

            spectrumChannel->TraceConnectWithoutContext("PathLoss", MakeCallback(&Ns3Simulation::SpectrumPathLossTrace, this));
        }
        else
        {
            NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
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
            devices = wifi.Install(wifiPhy, mac, this->nodes);
        }
        else if (wifiType == "spectrum_3GPP_V2V_urban_channel")
        {
            devices = wifi.Install(spectrumPhy, mac, this->nodes);
        }
        else
        {
            NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
        }

        Config::Set(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
            BooleanValue(true));

        // [Buildings] -- Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
        BuildingsHelper::Install(this->nodes);

        // // **************** TAP-BRIDGE MODULE ****************
        // TapBridgeHelper tapBridge;
        // tapBridge.SetAttribute("Mode", StringValue("UseLocal"));
        // char buffer[10];
        // for (uint32_t i=0; i<numNodes; i++) {
        //     sprintf(buffer, "wifi_tap%d", i+1);
        //     tapBridge.SetAttribute ("DeviceName", StringValue(buffer));
        //     tapBridge.Install(nodes.Get(i), devices.Get(i));
        // }

        // for(auto netDevice = devices.Begin(); netDevice != devices.End(); ++netDevice){
        //     (*netDevice)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this, (*netDevice)->GetNode()->GetId()));
        // }
        // devices.Get(0)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this));
        // devices.Get(2)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this));

        /* **************** IP / ROUTING MODULE **************** */
        // Ipv4ListRoutingHelper ipv4List;
        // AodvHelper aodv;
        InternetStackHelper internet;
        // ipv4List.Add(aodv, 100);

        // internet.SetRoutingHelper(ipv4List);
        internet.Install(this->nodes);

        Ipv4AddressHelper addressAdhoc;
        addressAdhoc.SetBase("10.0.0.0", "255.255.255.0");
        Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign(devices);
        Address serverAddress = Address(adhocInterfaces.GetAddress(0));
        Address clientAddress = Address(adhocInterfaces.GetAddress(1));

        // Create the broadcast address
        Ipv4Address broadcastAddress = Ipv4Address::GetBroadcast();

        /* **************** APPLICATION MODULE **************** */
        // PacketSocketHelper packetSocket;
        // packetSocket.Install(nodes);

        // PacketSocketAddress socket_;
        // socket_.SetSingleDevice(devices.Get(0)->GetIfIndex());
        // socket_.SetPhysicalAddress(devices.Get(1)->GetAddress());
        // socket_.SetProtocol(1);

        // OnOffHelper onoff("ns3::PacketSocketFactory", Address(socket_));
        // onoff.SetConstantRate(DataRate(60000000));
        // onoff.SetAttribute("PacketSize", UintegerValue(2000));

        // ApplicationContainer apps = onoff.Install(nodes.Get(0));
        // apps.Start(Seconds(5.0));
        // apps.Stop(Seconds(40.0));

        // Ptr<Socket> recvSink = SetupPacketReceive(nodes.Get(1));

        // UDP server on all nodes to receive the UDP pose broadcast traffic
        uint16_t port = 4000;
        UdpServerHelper server(port);
        ApplicationContainer servers;
        for (int i = 0; i < numNodes; i++)
        {
            ApplicationContainer app = server.Install(nodes.Get(i));
            app.Start(Seconds(1.0));
            app.Stop(simEndTime);
            if (app.Get(0)->TraceConnect("RxWithAddresses", std::to_string(i), MakeCallback(&Ns3Simulation::server_receive_clbk, this)))
            {
                std::cout << "Connected trace Rx" << std::endl;
            }
            else
            {
                std::cout << "Could not connect trace Rx" << std::endl;
                exit(EXIT_FAILURE);
            }
            servers.Add(app);
        }

        // UDP client on all nodes that regularly sends a small UDP broadcast packet, simulating pose sharing between agents
        uint32_t MaxPacketSize = 1024;
        Time interPacketInterval = Seconds(0.05);
        uint32_t maxPacketCount = 4294967295;
        UdpClientHelper client(broadcastAddress, port);
        client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
        client.SetAttribute("Interval", TimeValue(interPacketInterval));
        client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
        double min = 0.0;
        double max = 1.0;
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
        x->SetAttribute("Min", DoubleValue(min));
        x->SetAttribute("Max", DoubleValue(max));
        ApplicationContainer clients;
        for (int i = 0; i < numNodes; i++)
        {
            ApplicationContainer app = client.Install(nodes.Get(i));
            app.Start(Seconds(1.0 + x->GetValue()));
            app.Stop(simEndTime);
            clients.Add(app);
        }
        std::cout << "Number of client applications: " << clients.GetN() << std::endl;

        // **************** SOCKET MODULE ****************
        // Create a socket on each node,
        // Bind it to the IP address of the node
        // Aggregate the two objects (node and socket together) <-- allows to call GetObject() on each other
        // Fill the ip-to-node map
        // Print info
        // TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        // for(uint32_t i=0 ; i < numNodes ; i++)
        // {
        //     Ptr<Socket> recvSink = Socket::CreateSocket(nodes.Get(i), tid);
        //     InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
        //     recvSink->Bind(local);
        //     recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));
        //     nodes.Get(i)->AggregateObject(recvSink);
        //     ip_node_map.emplace(ip.GetAddress(i), nodes.Get(i));
        //     std::cout << "IP of node " << i << " : " << ip.GetAddress(i) << std::endl;
        // }

        // **************** UDS SOCKET FOR NETWORK COORDINATOR ****************
        // Create and connect UDS Socket
        ::Socket *socket;
        boost::asio::io_context io_context;
        if (config["net_use_uds"].as<bool>())
        {
            socket = new UDSSocket(io_context);
            socket->accept(config["net_uds_server_address"].as<std::string>(), 0);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->accept(config["net_ip_server_address"].as<std::string>(), config["net_ip_server_port"].as<unsigned short>());
        }
        RCLCPP_INFO(this->get_logger(), "finished setting up UDS socket");

        // **************** MAIN SIMULATION LOOP ****************
        while (currTime < simEndTime || simEndTime == Seconds(0.0))
        {

            // std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            // Wait until reception of a message on the UDS socket
            std::string received_data = gzip_decompress(socket->receive_one_message());
            // std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            // int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
            // if(wait != 0)
            // RCLCPP_WARN(this->get_logger(), "Received %li bytes in %li microseconds", received_data.size(), wait);

            // Initialize empty protobuf message type [NetworkUpdate]
            network_update_proto::NetworkUpdate NetworkUpdate_msg;
            // Transform the message received from the UDS socket [string] -> [protobuf]
            NetworkUpdate_msg.ParseFromString(received_data);

            // std::cout << "Proto received from Network coordinator: \n" << NetworkUpdate_msg.DebugString() << std::endl;

            // Read the "physical" information transmitted by the NetworkCoordinator, and update the node's positions
            // Also verifies that the number of nodes sent by the NetworkCoordinator corresponds to the number of existing nodes in NS-3
            rclcpp::Clock clock;
            robots_positions_proto::RobotsPositions robots_positions_msg;
            if(!NetworkUpdate_msg.robots_positions().empty()){
                RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                robots_positions_msg.ParseFromString(gzip_decompress(NetworkUpdate_msg.robots_positions()));

                // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                // Then, update the node's positions (orientation is ignored for now)
                if(this->nodes.GetN() != (uint32_t)robots_positions_msg.robot_pose_size()){
                    if(this->get_parameter("verbose").as_bool()){
                        RCLCPP_WARN_THROTTLE(this->get_logger(),
                                clock,
                                1000, // ms
                                "Network simulator received position information of %i robots but NS-3 has %zu nodes.",
                                robots_positions_msg.robot_pose_size(),
                                ip_node_map.size());
                    }
                } else {
                    for(uint32_t i=0 ; i < this->nodes.GetN() ; i++){
                            Vector pos;
                            pos.x = robots_positions_msg.robot_pose(i).x();
                            pos.y = robots_positions_msg.robot_pose(i).y();
                            pos.z = robots_positions_msg.robot_pose(i).z();
                            SetNodePosition(this->nodes.Get(i), pos);
                        }
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
            }

            // Once all the events are scheduled, advance W time in the simulation and stop
            Simulator::Stop(step_size);

            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            Simulator::Run();
            if (this->save_compute_time)
            {
                std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                int64_t wait = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                this->f.open(m_output_file.c_str(), std::fstream::app);
                this->f << wait << "\n";
                this->f.close();
            }

            if (this->get_parameter("verbose").get_parameter_value().get<bool>())
            {
                // 32 = Green :)
                RCLCPP_INFO(this->get_logger(), "\x1b[32m[%f] Advanced %i microseconds\x1b[0m", Simulator::Now().GetSeconds(), step_size);
                // std::cout << "\x1b[32m[" << Simulator::Now().GetSeconds() << "] Advanced " << step_size << " milliseconds\x1b[0m" << std::endl;
            }
            std::map<uint32_t, std::map<uint32_t, double>> neighbors = create_neighbors(1.0);
            // for (int i = 0; i < numNodes; i++)
            // {
            //     for (int j = 0; j < numNodes; j++)
            //     {
            //         // print neighbors
            //         // if(neighbors.find(i) != neighbors.end() && neighbors[i].find(j) != neighbors[i].end())
            //         // {
            //             RCLCPP_INFO(this->get_logger(), "Node %i has neighbor %i with pathloss %f", i, j, neighbors[i][j]);
            //         // }
            //     }
            // }
            // std::cout << std::endl;

            std::string response=gzip_compress(generate_response(NetworkUpdate_msg, neighbors));

            // Send the response to the network coordinator
            socket->send_one_message(response);

            currTime += step_size;

            // print the number of packets sent to each group of node:
            // std::cout << "\rNumber of packets sent to nodes / other addresses: " << num_packets_to_nodes << " / " << num_packets_to_other_addresses << std::endl;
        }

        RCLCPP_INFO(this->get_logger(), "Simulation finished");
        int k = 0;
        for (auto s = servers.Begin(); s != servers.End(); ++s)
        {
            Ptr<UdpServer> server = (*s)->GetObject<UdpServer>();
            std::cout << "Server " << k << " received " << server->GetReceived() << " packets." << std::endl;
            k++;
        }
        exit(EXIT_SUCCESS);
    }

private:
    bool save_compute_time;
    std::ofstream f;
    std::ofstream file_received_packets;
    void node_receive_clbk(uint32_t nodeId);
    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    void server_receive_clbk(std::string context, const Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress);
    void FlowMonSaveResults(Ptr<FlowMonitor> flowMonitor);
    std::map<uint32_t, std::map<uint32_t, double>> create_neighbors(double timeout);
    std::vector<int> numReceivedPackets;
    ordered_neighbors_proto::OrderedNeighborsList ordered_neighbors_list_msg;
    NodeContainer nodes;
    std::map<uint32_t, std::map<uint32_t, Time>> neigh_last_received;
    std::map<uint32_t, std::map<uint32_t, double>> neigh_pathloss;
    uint32_t nNodes;
};

void Ns3Simulation::SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t txId = txPhy->GetDevice()->GetNode()->GetId();
    uint32_t rxId = rxPhy->GetDevice()->GetNode()->GetId();
    // std::cout << "Tx: " << txId << " Rx: " << rxId << " Loss: " << lossDb << std::endl;
    this->neigh_pathloss[txId][rxId] = lossDb;
}

void Ns3Simulation::server_receive_clbk(std::string context, const Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress)
{
    uint32_t nodeId = std::stoi(context);
    uint32_t txId = InetSocketAddress::ConvertFrom(srcAddress).GetIpv4().CombineMask("0.0.0.255").Get() - 1;
    Time now = Simulator::Now();
    std::cout << "UDP Server " << nodeId << " received a packet from " << txId << " at " << now << std::endl;
    this->neigh_last_received[nodeId][txId] = now;
    for(auto x : this->neigh_last_received)
    {
        for(auto y : x.second)
        {
            RCLCPP_INFO(this->get_logger(), "Node %d -> %d : %f", x.first, y.first, y.second.ToDouble(Time::Unit::US));
        }
    }
    std::cout << std::endl;
}

std::map<uint32_t, std::map<uint32_t, double>>
Ns3Simulation::create_neighbors(double timeout)
{
    std::map<uint32_t, std::map<uint32_t, double>> neighbors;
    for (uint32_t i = 0; i < this->nNodes; i++)
    {
        for (uint32_t j = 0; j < this->nNodes; j++)
        {
            if (i != j && this->neigh_last_received.find(i) != this->neigh_last_received.end())
            {
                if (this->neigh_last_received[i].find(j) != this->neigh_last_received[i].end())
                {
                    Time now = Simulator::Now();
                    Time last_received = this->neigh_last_received[i][j];
                    double pathloss = this->neigh_pathloss[i][j]; // Yes, we assume here that neigh_last_received and neigh_pathloss have same keys at all time
                    if (now - last_received < Seconds(timeout))
                    {
                        neighbors[i][j] = pathloss;
                    }
                }
            }
        }
    }
    return neighbors;
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

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
#include "ns3/internet-module.h"
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
#include "ns3/wifi-module.h"

#include "ns3/udp-client-server-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/flow-monitor-helper.h"

#include "ns3/stats-module.h"

#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/robots_positions.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include "udp_tcp_socket.hpp"

#include <yaml-cpp/yaml.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("UAV_fleet_adhoc_network");

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
        param_desc.description = "Executes in co-simulation or ns-3 only.";
        this->declare_parameter("cosim_mode", true, param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using a ros2 parameter
        std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        bool cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();
        bool verbose = this->get_parameter("verbose").get_parameter_value().get<bool>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // Parse the config file
        YAML::Node config = YAML::LoadFile(config_file_path);

        // ========================= NS3 CONFIGURATION =========================

        // Set the seed for the random number generator
        SeedManager::SetRun(config["ns3_seed"].as<int>());

        Time step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        Time currTime = MicroSeconds(0);                                       // us
        Time simEndTime = Seconds(20.0);                                        // simulation time (s)

        uint32_t numNodes = config["robots_number"].as<int>();

        std::string wifiType = config["wifi_type"].as<std::string>();
        std::string errorModelType = config["error_model_type"].as<std::string>();
        std::string phyMode(config["phy_mode"].as<std::string>()); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
        double frequency = 5.2e9;         // operating frequency in Hz

        // Create the nodes
        this->nodes.Create(numNodes);

        // **************** BUILDINGS MODULE ****************

        // [Buildings] -- Define buildings with their center (x, y), and their size in the three dimensions : (size_x, size_y, height)
        std::vector<Ptr<Building>> buildings;
        for (auto building : config["buildings"])
        {
            double x_min = building["x"].as<float>() - (building["size_x"].as<float>() / 2);
            double x_max = building["x"].as<float>() + (building["size_x"].as<float>() / 2);
            double y_min = building["y"].as<float>() - (building["size_y"].as<float>() / 2);
            double y_max = building["y"].as<float>() + (building["size_y"].as<float>() / 2);
            double z_min = 0.0;
            double z_max = building["height"].as<float>();
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
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of MOBILITY module");


        // **************** PROPAGATION MODULE ****************

        SpectrumWifiPhyHelper spectrumPhy;
        YansWifiPhyHelper wifiPhy;

        if (wifiType == "yans_default")
        {
            YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
            wifiPhy.SetChannel(wifiChannel.Create());

            for (uint32_t i = 0; i < this->nodes.GetN(); i++)
            {
                for (uint32_t j = 0; j < this->nodes.GetN(); j++)
                {
                    this->neigh_pathloss[i][j] = -69.0;
                }
            }
        }
        else if (wifiType == "spectrum_3GPP_V2V_urban_channel")
        {
            Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

            // create the channel condition model
            Ptr<ChannelConditionModel> m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
            m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

            // create the propagation loss model and add it to the channel condition
            Ptr<ThreeGppPropagationLossModel> m_propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
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
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of PHY layer");


        // **************** WIFI MODULE ****************

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211g);
        WifiMacHelper mac;

        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode));

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

        // "Mission" flow : unicast, unidirectional
        uint32_t source_node_id = config["mission_flow"]["source_robot_id"].as<uint32_t>();
        uint32_t sink_node_id = config["mission_flow"]["sink_robot_id"].as<uint32_t>();
        double start_traffic_time = config["mission_flow"]["start_traffic_time"].as<double>();  // s
        double stop_traffic_time = config["mission_flow"]["stop_traffic_time"].as<double>();    // s
        uint32_t packet_size = config["mission_flow"]["packet_size"].as<uint32_t>();                 // bytes        
        uint64_t interval = config["mission_flow"]["interval"].as<uint64_t>();                       // us
        uint16_t mission_flow_port = config["mission_flow"]["port"].as<uint16_t>();
        
        UdpClientHelper mission_flow_sender(this->nodes.Get(sink_node_id)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(), mission_flow_port);
        mission_flow_sender.SetAttribute("MaxPackets", UintegerValue(4294967295));
        mission_flow_sender.SetAttribute("Interval", TimeValue(MicroSeconds(interval)));
        mission_flow_sender.SetAttribute("PacketSize", UintegerValue(packet_size));
        ApplicationContainer mission_flow_sender_app = mission_flow_sender.Install(this->nodes.Get(source_node_id));
        mission_flow_sender_app.Start(Seconds(start_traffic_time));
        mission_flow_sender_app.Stop(Seconds(stop_traffic_time));

        UdpServerHelper mission_flow_receiver(mission_flow_port);
        ApplicationContainer mission_flow_receiver_app = mission_flow_receiver.Install(this->nodes.Get(sink_node_id));
        mission_flow_receiver_app.Start(Seconds(1.0));
        mission_flow_receiver_app.Stop(simEndTime);

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
                RCLCPP_DEBUG(this->get_logger(), "Connected trace Rx");
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Could not connect trace Rx for node %d.", i);
                exit(EXIT_FAILURE);
            }
            servers.Add(app);
        }

        // UDP client on all nodes that regularly sends a small UDP broadcast packet, simulating pose sharing between agents
        uint32_t MaxPacketSize = 200;
        Time interPacketInterval = Seconds(0.1);
        uint32_t maxPacketCount = 4294967295;
        UdpClientHelper client(broadcastAddress, port);
        client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
        client.SetAttribute("Interval", TimeValue(interPacketInterval));
        client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
        
        // Random start, otherwise they never access the medium (I think)
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

        if(cosim_mode)
        {

        
            // **************** UDS SOCKET FOR NETWORK COORDINATOR ****************
            // Create and connect UDS Socket
            CustomSocket *socket;
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
                if(!NetworkUpdate_msg.robots_positions().empty()){
                    RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                    robots_positions_msg.ParseFromString(gzip_decompress(NetworkUpdate_msg.robots_positions()));

                    // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                    // Then, update the node's positions (orientation is ignored for now)
                    if(this->nodes.GetN() != (uint32_t)robots_positions_msg.robot_pose_size()){
                        if(verbose){
                            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                    clock,
                                    1000, // ms
                                    "Network simulator received position information of %i robots but NS-3 has %u nodes.",
                                    robots_positions_msg.robot_pose_size(),
                                    this->nodes.GetN());
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

                Simulator::Run();

                // Create an object giving the neighborhood of each node, based on the packets received by their UDP server, neighbors lifetime is 1 second.
                std::map<uint32_t, std::map<uint32_t, double>> neighbors = create_neighbors(1.0);

                std::string response=gzip_compress(generate_response(NetworkUpdate_msg, neighbors));

                // Send the response to the network coordinator
                socket->send_one_message(response);

                currTime += step_size;

            }
        }
        else
        {
            Simulator::Stop(simEndTime);
            std::cout << "Starting simulation." << std::endl;
            
            Simulator::Run();
        }

        std::cout << "Simulation finished." << std::endl;

        Ptr<UdpServer> mission_server = DynamicCast<UdpServer>(mission_flow_receiver_app.Get(0));
        Ptr<UdpClient> mission_client = DynamicCast<UdpClient>(mission_flow_sender_app.Get(0));

        int k = 0;
        for (auto s = servers.Begin(); s != servers.End(); ++s)
        {
            Ptr<UdpServer> server = (*s)->GetObject<UdpServer>();
            std::cout << "Position server " << k << " received " << server->GetReceived()*MaxPacketSize << " bytes." << std::endl;
            k++;
        }
        std::cout << std::endl;

        std::cout << "Mission source sent " << mission_client->GetTotalTx() << " bytes" << std::endl;
        std::cout << "Mission sink received " << mission_server->GetReceived()*1024 << " bytes" << std::endl;

        Simulator::Destroy();

        exit(EXIT_SUCCESS);
    }

private:
    NodeContainer nodes;
    std::map<uint32_t, std::map<uint32_t, Time>> neigh_last_received;
    std::map<uint32_t, std::map<uint32_t, double>> neigh_pathloss;

    ordered_neighbors_proto::OrderedNeighborsList ordered_neighbors_list_msg;

    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    std::map<uint32_t, std::map<uint32_t, double>> create_neighbors(double timeout);
    void server_receive_clbk(std::string context, const Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress);
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
    RCLCPP_DEBUG(this->get_logger(), "UDP Server %d received a packet from %d at time %f", nodeId, txId, now.ToDouble(Time::Unit::S));
    this->neigh_last_received[nodeId][txId] = now;
    // for(auto x : this->neigh_last_received)
    // {
    //     for(auto y : x.second)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Node %d -> %d : %f", x.first, y.first, y.second.ToDouble(Time::Unit::US));
    //     }
    // }
    // std::cout << std::endl;
}

std::map<uint32_t, std::map<uint32_t, double>>
Ns3Simulation::create_neighbors(double timeout)
{
    std::map<uint32_t, std::map<uint32_t, double>> neighbors;
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < this->nodes.GetN(); j++)
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

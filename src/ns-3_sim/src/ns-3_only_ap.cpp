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
#include "ns3/ssid.h"

#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/spectrum-wifi-helper.h"

#include "ns3/udp-client-server-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"

#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"

#include "protobuf_msgs/network_update.pb.h"
#include "protobuf_msgs/channel_data.pb.h"

#include <yaml-cpp/yaml.h>

// Define a port on which the nodes listen and talk
#define PORT 80

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("UAV_fleet_adhoc_network");

static std::string m_output_file;

static Ptr<ThreeGppPropagationLossModel> m_propagationLossModel;        //!< the PropagationLossModel object
static Ptr<ThreeGppSpectrumPropagationLossModel> m_spectrumLossModel;   //!< the SpectrumPropagationLossModel object
static Ptr<ChannelConditionModel> m_condModel;                          //!< the ChannelConditionModel object

// Global variables for use in callbacks.
double g_signalDbmAvg; //!< Average signal power [dBm]
double g_noiseDbmAvg;  //!< Average noise power [dBm]
uint32_t g_samples;    //!< Number of samples
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
void
SetNodePosition(Ptr<Node> node, Vector position)
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
void 
printPosition(Time period, Ptr<ns3::Node> sender, Ptr<ns3::Node> receiver)
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
gzip_compress(const std::string& data)
{
    std::stringstream compressed;
    std::stringstream origin(data);

    boost::iostreams::filtering_streambuf< boost::iostreams::input> in;
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
gzip_decompress(const std::string& data)
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
    uint32_t receive_length=ntohl(*data_preamble);
    // Read Message
    char data[receive_length];
    length = sock.receive(boost::asio::buffer(data, receive_length));
    std::string data_string(data,length);

    return data_string;
}

/**
 * \brief Sends a message from a socket.
 *
 * \param sock The socket used to send the message.
 * \param str The string message to send.
 */
void 
send_one_message(boost::asio::local::stream_protocol::socket &sock, std::string str)
{
    // Send Preamble
    std::size_t response_size=str.size();
    // static_cast<uint32_t>(response_size);
    uint32_t send_length=htonl(response_size);
    sock.send(boost::asio::buffer(&send_length,4));
    // Send Message
    sock.send(boost::asio::buffer(str.data(), str.size()));
}

// /**
//  * Send an IP packet.
//  *
//  * \param sender The Node id of the source node.
//  * \param ip_dest The ipv4 address of the destination as ns3::Ipv4Address object
//  * \param pkt_size The size of the packet (in bytes?).
//  */
// static void
// SendPacket(uint32_t sender, Ipv4Address ip_dest, uint32_t pkt_size)
// {
//     // Get the pointers to the nodes
//     Ptr<Node> nodeSender = NodeList::GetNode(sender);
//     // At initialisation, each node was aggregated to a Socket object, get this Socket
//     Ptr<Socket> socket = nodeSender->GetObject<Socket>();
//     InetSocketAddress remote = InetSocketAddress(ip_dest, PORT);
//     if(socket->Connect(remote) != -1)
//     {
//         NS_LOG_INFO("socket connected");
//     } else {
//         NS_LOG_INFO("Error when connecting socket");
//     }

//     // Send a "fake" packet in the socket (second argument is the socket control flags)
//     if(socket->Send(Create<Packet>(pkt_size)))
//     {
//         NS_LOG_INFO("One packet sent");
//     } else {
//         NS_LOG_INFO("Error when sending a packet");
//     }
// }

/**
 * Function called when a packet is received.
 *
 * \param socket The receiving socket.
 */
void
ReceivePacket(Ptr<Socket> socket)
{
    while (socket->Recv())
    {
        NS_LOG_UNCOND("Received one packet!");
        // std::cout << socket->GetNode() << " Received a packet!" << std::endl;
    }
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
    // for(int i=0 ; i < NetworkUpdate_msg.pkt_id_size() ; i++){
    //     NetworkUpdate_msg.add_rx_ip(NetworkUpdate_msg.dst_ip(i)); // Not dealing with broadcast
    // }
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

            // Declare two parameters for this ros2 node
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "Path to the YAML configuration file.";
            this->declare_parameter("config_file", "", param_desc);
            this->declare_parameter("verbose", false);

            // Fetch the parameter path to the config file using a ros2 parameter
            std::string config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();

            // Verify existence of the config file, abort if not found
            if(access(config_file_path.c_str(), F_OK) != 0){
               RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
               exit(EXIT_FAILURE);
            }

             // Parse the config file
            YAML::Node config = YAML::LoadFile(config_file_path);

            this->save_compute_time = config["save_compute_time"].as<bool>();

// vvvvvvvv DATA SAVING vvvvvvvv
            if(this->save_compute_time){
                // Create a folder based on the experience name, if not existant already
                std::string experience_name = config["experience_name"].as<std::string>();
                if(boost::filesystem::create_directories("./data/"+experience_name)){
                    RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", experience_name.c_str());
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "Using existing data folder for this experiment");
                }

                // Define the output file name, based on the existing files in the experience folder (incremental)
                std::string temp_path;
                int i = 1;
                while(m_output_file.empty()){
                    temp_path = "./data/"+experience_name+"/network_sim_data"+std::to_string(i)+".csv";
                    if(boost::filesystem::exists(temp_path)){
                        i++;
                    } else {
                        m_output_file = temp_path;
                    }
                }

                // initialize the output file with headers
                this->f.open(m_output_file.c_str(), std::ios::out);
                this->f << "Time[us]"
                << std::endl;
                this->f.close();
            }
// ^^^^^^^^ DATA SAVING ^^^^^^^^


// ========================= NS3 CONFIGURATION =========================

            // Set the seed for the random number generator
            SeedManager::SetRun(config["ns3_seed"].as<int>());

            // we interact with the "real world" through tap bridges, so we need to use the real-time implementation of the simulator
            GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
            GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));


            double frequency = 5.2e9;          // operating frequency in Hz
            Time timeRes = MilliSeconds(100);    // time resolution
            std::string errorModelType = "ns3::NistErrorRateModel";
            std::string wifiType = config["wifi_type"].as<std::string>();
            std::string phyMode("DsssRate1Mbps");   // Define a "Phy mode" that will be given to the WifiRemoteStationManager
            uint32_t numNodes = config["robots_number"].as<int>();
            uint32_t step_size = config["net_step_size"].as<uint32_t>()/1000; // in milliseconds
            double rss = -80;           // -dBm

// Old parameters
            // double txPow_dbm = 30.0;            // tx power in dBm
            // double noiseFigure = 9.0;           // noise figure in dB
            // double simTime = 60;         // simulation time
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
            // uint32_t payloadSize = 20; // for UDP (1000 bytes IPv4)

            for(int i=0; i<numNodes; i++){
                this->numReceivedPackets.push_back(0);
            }

            // Create the nodes
            NodeContainer stas;
            stas.Create(numNodes-1);
            NodeContainer ap;
            ap.Create(1);
            NodeContainer nodes;
            nodes.Add(ap);
            nodes.Add(stas);

    // **************** BUILDINGS MODULE ****************

            // [Buildings] -- Define buildings with their center (x, y), and their size in the three dimensions : (size_x, size_y, height)
            std::vector< Ptr<Building> > buildings;
            for(auto building : config["buildings"]){
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
            for(uint32_t i = 0; i < numNodes; i++){
                Ptr<MobilityModel> nodeMob;
                nodeMob = CreateObject<ConstantPositionMobilityModel>();
                nodeMob->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(i, i, 0.5));
                nodes.Get(i)->AggregateObject(nodeMob);
            }

    // **************** PROPAGATION MODULE ****************

            YansWifiPhyHelper phy;
            SpectrumWifiPhyHelper spectrumPhy;
            YansWifiPhyHelper wifiPhy;

            wifiPhy.Set("RxGain", DoubleValue(0));

            if (wifiType == "yans_default"){
                YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
                wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
                // The below FixedRssLossModel will cause the rss to be fixed regardless
                // of the distance between the two stations, and the transmit power
                wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(rss));
                wifiPhy.SetChannel (wifiChannel.Create ());
            }

        // --------- YANS ---------
            else if (wifiType == "yans_hybrid_buildings"){
                YansWifiChannelHelper channel;
                channel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                            "Frequency",     // Additional loss for each internal wall [dB] 
                                            DoubleValue(frequency),       // Default 2.4e9
                                            "ShadowSigmaExtWalls",  // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls 
                                            DoubleValue(5.0),       // Default 5
                                            "ShadowSigmaIndoor",    // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                                            DoubleValue(8.0),       // Default 8
                                            "ShadowSigmaOutdoor",   // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                                            DoubleValue(7.0));       // Default 7
                channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
                phy.SetChannel(channel.Create());
                phy.Set("TxPowerStart", DoubleValue(22)); // dBm
                phy.Set("TxPowerEnd", DoubleValue(22));

            }
        // --------- SPECTRUM ---------
            else if (wifiType == "spectrum_3GPP_V2V_urban_channel"){

                Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
                
                // create the channel condition model
                m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
                m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

                // create the propagation loss model
                m_propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
                m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
                m_propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(false));
                m_propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));

                spectrumChannel->AddPropagationLossModel(m_propagationLossModel);

                Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();

                spectrumChannel->SetPropagationDelayModel(delayModel);

                spectrumPhy.SetChannel(spectrumChannel);
                spectrumPhy.SetErrorRateModel(errorModelType);
                spectrumPhy.Set("TxPowerStart", DoubleValue(5)); // dBm  (1.26 mW)
                spectrumPhy.Set("TxPowerEnd", DoubleValue(5));
            }
            else {
                NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
            }

    // **************** WIFI MODULE ****************
            WifiHelper wifi;
            Ssid ssid = Ssid("wifi-default");
            wifi.SetStandard(WIFI_STANDARD_80211b);
            wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                        "DataMode",
                                        StringValue(phyMode),
                                        "ControlMode",
                                        StringValue(phyMode));
            // wifi.SetRemoteStationManager("ns3::IdealWifiManager");

            WifiMacHelper wifiMac_sta;
            WifiMacHelper wifiMac_ap;

            NetDeviceContainer devices;
            NetDeviceContainer ap_device;
            NetDeviceContainer sta_devices;


            wifiMac_ap.SetType( "ns3::ApWifiMac",
                            "Ssid", SsidValue (ssid));

            wifiMac_sta.SetType("ns3::StaWifiMac",
                            "ActiveProbing", BooleanValue (true),
                            "Ssid", SsidValue (ssid));

            if (wifiType == "yans_default" || wifiType == "yans_log_distance" || wifiType == "yans_hybrid_buildings")
            {
                sta_devices = wifi.Install(wifiPhy, wifiMac_sta, stas);
                ap_device = wifi.Install(wifiPhy, wifiMac_ap, ap);
            } else if (wifiType == "spectrum_3GPP_V2V_urban_channel"){
                sta_devices = wifi.Install(spectrumPhy, wifiMac_sta, stas);
                ap_device = wifi.Install(spectrumPhy, wifiMac_ap, ap);
            } else {
                NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
            }

            devices.Add(ap_device);
            devices.Add(sta_devices);

            Config::Set(
                "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
                BooleanValue(true)
            );
            
            // [Buildings] -- Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
            BuildingsHelper::Install(nodes);

            // **************** TAP-BRIDGE MODULE ****************
            TapBridgeHelper tapBridge;
            tapBridge.SetAttribute("Mode", StringValue("UseLocal"));
            char buffer[10];
            for (uint32_t i=0; i<numNodes; i++) {
                sprintf(buffer, "wifi_tap%d", i+1);
                tapBridge.SetAttribute ("DeviceName", StringValue(buffer));
                tapBridge.Install(nodes.Get(i), devices.Get(i));
            }

            // for(auto netDevice = devices.Begin(); netDevice != devices.End(); ++netDevice){
            //     (*netDevice)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this, (*netDevice)->GetNode()->GetId()));
            // }
            // devices.Get(0)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this));
            // devices.Get(2)->SetReceiveCallback(MakeCallback(&Ns3Simulation::node_receive_clbk, this));

/*    // **************** APPLICATION MODULE ****************
            ApplicationContainer serverApp;

            // UDP flow
            uint16_t port = 9;
            UdpServerHelper server(port);
            serverApp = server.Install(nodes.Get(0));
            serverApp.Start(Seconds(0.0));
            serverApp.Stop(Seconds(simTime + 1));

            UdpClientHelper client(ip.GetAddress(1), port);
            client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
            client.SetAttribute("Interval", TimeValue(Time("0.01s"))); // packets/s
            client.SetAttribute("PacketSize", UintegerValue(payloadSize));
            ApplicationContainer clientApp = client.Install(nodes.Get(1));
            clientApp.Start(Seconds(1.0));
            clientApp.Stop(Seconds(simTime + 1));
*/

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

            Simulator::Stop(Seconds(60));
            std::cout << "Network simulator ready" << std::endl;

            Simulator::Run();

            std::cout << "Network simulator ended" << std::endl;


        }
    private:
        bool save_compute_time;
        std::ofstream f;
        void node_receive_clbk(uint32_t nodeId);
        std::vector<int> numReceivedPackets;
};

void
Ns3Simulation::node_receive_clbk(uint32_t nodeId){
    this->numReceivedPackets[nodeId] += 1;
}

/**
 * \brief The main function, spins the ROS2 Node
*/
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ns3Simulation>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>

#include <stack>

#include <map>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "ns3/core-module.h"
#include "ns3/buildings-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
// #include "ns3/dsr-module.h"
#include "ns3/dsdv-module.h"

#include "protobuf_msgs/dancers_update.pb.h"
#include "protobuf_msgs/pose_vector.pb.h"
#include "protobuf_msgs/ordered_neighbors.pb.h"

#include <dancers_msgs/msg/target.hpp>

#include <yaml-cpp/yaml.h>

#include <util.hpp>
#include <time_probe.hpp>

#include <wifi-application.h>
#include <flocking-application.h>

#include <udp_tcp_socket.hpp>

using namespace ns3;
using std::placeholders::_1;

/**
 * @brief The ns-3 ROS2 Node, part of the DANCERS co-simulator
 *
 * This class is a ROS2 node that holds the network simulation for mobile communicating nodes, using the ns-3 simulator.
 */
class Ns3Sim : public rclcpp::Node
{
public:
    Ns3Sim() : Node("ns3_sim_pseudo_routing")
    {
        RCLCPP_INFO(this->get_logger(), "Adhoc Chain Flocking Node Created");

        // ========================= PARAMETER READING AND LOG FILE INITIALIZATION =========================
        // Declare two parameters for this ros2 node
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Path to the YAML configuration file.";
        this->declare_parameter("config_file", "", param_desc);
        param_desc.description = "Executes in co-simulation or ns-3 only.";
        this->declare_parameter("cosim_mode", true, param_desc);
        this->declare_parameter("verbose", false);

        // Fetch the parameter path to the config file using a ros2 parameter
        this->config_file_path = this->get_parameter("config_file").get_parameter_value().get<std::string>();
        this->cosim_mode = this->get_parameter("cosim_mode").get_parameter_value().get<bool>();
        this->verbose = this->get_parameter("verbose").get_parameter_value().get<bool>();

        // Verify existence of the config file, abort if not found
        if (access(config_file_path.c_str(), F_OK) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The config file was not found at : %s", config_file_path.c_str());
            exit(EXIT_FAILURE);
        }

        // get the name of the folder containing the config file
        // boost::filesystem::path config_file(config_file_path);
        // boost::filesystem::path config_folder = config_file.parent_path();

        // Get the path to the ROS_WS, it is mandatory to run
        if (getenv("ROS_WS") == NULL)
        {
            RCLCPP_FATAL(this->get_logger(), "ROS_WS environment variable not set, aborting.");
            exit(EXIT_FAILURE);
        }
        else
        {
            this->m_ros_ws_path = getenv("ROS_WS");
        }

        // Parse the config file
        this->config = YAML::LoadFile(config_file_path);

        // ========================= OUTPUT FILES =========================
        this->save_compute_time = config["save_compute_time"].as<bool>();

        this->experience_name = config["experience_name"].as<std::string>();
        this->run_id = config["run_id"].as<int>();

        if (this->save_compute_time)
        {

            // Create a folder based on the experience name, if not existant already
            if (boost::filesystem::create_directories(this->m_ros_ws_path + "/data/" + this->experience_name))
            {
                RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", this->experience_name.c_str());
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
                temp_path = this->m_ros_ws_path + "/data/" + this->experience_name + "/network" + std::to_string(i) + ".csv";
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

        this->save_mission_packets = config["mission_flow"]["save_packets"].as<bool>();
        if (this->save_mission_packets)
        {
            // Create a folder based on the experience name, if not existant already
            if (boost::filesystem::create_directories(this->m_ros_ws_path + "/data/" + this->experience_name))
            {
                RCLCPP_DEBUG(this->get_logger(), "Created a new data folder for this experience : %s", this->experience_name.c_str());
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Using existing data folder for this experiment");
            }

            // Define the output file name, based on the existing files in the experience folder (incremental)
            std::string temp_path;
            int i = 1;
            while (this->m_mission_packets_file.empty())
            {
                temp_path = this->m_ros_ws_path + "/data/" + this->experience_name + "/mission_received_packets_" + std::to_string(i) + ".csv";
                if (boost::filesystem::exists(temp_path))
                {
                    i++;
                }
                else
                {
                    this->m_mission_packets_file = temp_path;
                }
            }

            // Write headers
            std::ofstream f(this->m_mission_packets_file, std::ios::app);
            f << "rcv_time(us),delay(us)" << std::endl;
            f.close();
        }

        // temporary path, should be incremented to not lose exp data later
        // std::string capacity_cost_out_file = this->m_ros_ws_path + "/data/" + experience_name + "/capacity_cost.csv";
        // std::ofstream capacity_cost_out(capacity_cost_out_file, std::ios::app);
        // capacity_cost_out << "time(ms),cost" << std::endl;
        // capacity_cost_out.close();

        /* ----------- ROS2 Subscribers ----------- */
        this->update_target_sub_ = this->create_subscription<dancers_msgs::msg::Target>("targets", 10, std::bind(&Ns3Sim::update_target_clbk, this, _1));
        if (this->update_target_sub_ == nullptr)
        {
            RCLCPP_FATAL(this->get_logger(), "Could not create the targets subscriber");
            exit(EXIT_FAILURE);
        }

        // ========================= NS3 =========================

        this->ns3_config = this->ReadNs3Config(config);

        this->ConfigureNs3(this->ns3_config);

        this->step_size = MicroSeconds(config["net_step_size"].as<uint32_t>()); // in microseconds
        this->currTime = MicroSeconds(0);                                       // us
        this->simEndTime = Seconds(config["simulation_length"].as<uint32_t>()); // simulation time (s)
        int numNodes = config["robots_number"].as<int>();
        this->pseudoRoutingAlgo = config["pseudo_routing_algorithm"].as<std::string>();

        this->broadcast_window_size = Seconds(config["minimize_error_rate"]["broadcast_window_size"].as<double>());

        // ========================= Pseudo-routing parameters =========================
        if (pseudoRoutingAlgo == "shortest_dist")
        {
            this->shortest_dist_r1 = config["shortest_dist"]["distance_threshold"].as<double>();
            this->shortest_dist_c1 = config["shortest_dist"]["constant_before_threshold"].as<double>();
        }
        else if (pseudoRoutingAlgo == "hybrid_dist_error_rate")
        {
            this->hybrid_r1 = config["hybrid_dist_error_rate"]["distance_threshold"].as<double>();
            this->hybrid_c1 = config["hybrid_dist_error_rate"]["constant_before_threshold"].as<double>();
            this->hybrid_e1 = config["hybrid_dist_error_rate"]["error_rate_exponent"].as<double>();
            this->hybrid_k1 = config["hybrid_dist_error_rate"]["distance_gain"].as<double>();
            this->hybrid_k2 = config["hybrid_dist_error_rate"]["error_rate_gain"].as<double>();
        }

        // Colours are fun: "\x1b[32m" enables red color ; "\x1b[0m" set default back
        RCLCPP_INFO(this->get_logger(), "\x1b[32mns-3 configuration finished. Starting simulation !\x1b[0m");

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&Ns3Sim::Loop, this);
    }

    ~Ns3Sim()
    {
        if (loop_thread_.joinable())
        {
            RCLCPP_INFO(this->get_logger(), "Ns3Sim Node Destroyed");
            loop_thread_.join();
        }
    }


private:
    NodeContainer nodes; //!< ns-3 nodes container. This is our access to reconfigure the network (add application, etc.) on the go, during the simulation

    std::string config_file_path;
    std::string m_ros_ws_path;
    ns3_configuration_t ns3_config;

    std::map<uint32_t, std::map<uint32_t, double>> pathlosses;
    std::map<uint32_t, std::map<uint32_t, double>> snrs;
    std::map<uint32_t, std::map<uint32_t, std::vector<Time>>> broadcast_received_packets;
    std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> mission_neighbors;
    std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> broadcast_neighbors;
    std::map<Mac48Address, uint32_t> mac_to_id;

    // Co-simulation related variables
    Time step_size; //!< The length of a ns-3 "step". An "iteration" can contain multiple "steps".
    Time currTime; //!< The current time of the simulation
    Time simEndTime; //!< The time at which the simulation should end

    Time broadcast_window_size;
    std::string pseudoRoutingAlgo;

    double shortest_dist_r1;
    double shortest_dist_c1;

    double hybrid_r1;
    double hybrid_c1;
    double hybrid_e1;
    double hybrid_k1;
    double hybrid_k2;

    YAML::Node config;

    bool cosim_mode;
    bool verbose;

    // Thread running the co-simulation Loop
    std::thread loop_thread_;

    bool stats_enabled;
    bool mission_flow;

    std::string experience_name;
    int run_id;
    DataCollector data;

    /* Subscribers */
    rclcpp::Subscription<dancers_msgs::msg::Target>::SharedPtr update_target_sub_;
    void update_target_clbk(dancers_msgs::msg::Target msg);

    Ptr<PropagationLossModel> m_propagationLossModel;

    bool targets_reached;

    // ns3 Trace callbacks
    void SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb);
    void PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow);
    void PhyTxEndTrace(Ptr<const Packet> packet);
    void PhyRxEndTrace(std::string context, Ptr<const Packet> packet);
    void mission_flow_receiver_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk(Ptr<const Packet> packet);
    void mission_flow_sender_clbk_2(Ptr<const Packet> packet);
    void flock_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id);
    void flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet);
    void MonitorSnifferRxTrace(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId);

    // save compute time (optional)
    bool save_compute_time;
    std::string m_computation_time_file;
    WallTimeProbe probe;

    // save received mission packets (optional)
    bool save_mission_packets;
    std::string m_mission_packets_file;

    // Stats attributes (optional)
    Ptr<PacketCounterCalculator> missionTotalRx;
    Ptr<PacketCounterCalculator> missionTotalTx;
    Ptr<TimeMinMaxAvgTotalCalculator> missionDelay;
    std::vector<Ptr<PacketCounterCalculator>> navTotalRxVector;
    std::vector<Ptr<PacketCounterCalculator>> navTotalTxVector;
    Ptr<PacketCounterCalculator> navEffectiveTx;

    // co-simulation specific methods
    std::string generateResponseProtobuf();
    std::string generateNeighborsMsg();

    void timeoutNeighbors();
    void updateNeighborsPathloss();
    double costFunction(double dist, double r1, double c1);
    double hybrid_dist_error_rate_cost_function(double dist, double r1, double c1, double error_rate, double e1, double k1, double k2);
    void updateStaticIpv4Routes(const std::vector<int> path);
    void Loop();

    ns3_configuration_t ReadNs3Config(const YAML::Node& config);
    void ConfigureNs3(const ns3_configuration_t& ns3_config);
    void RunPseudoRoutingAlgorithm(const std::string pseudo_routing_algo_name);
};

/**
 * @brief Read the ns3 configuration from the config file
 * 
 * Read the YAML configuration file and translate it in a struct object, I find it easier to handle
 */
ns3_configuration_t Ns3Sim::ReadNs3Config(const YAML::Node& config)
{
    ns3_configuration_t ns3_config;

    ns3_config.seed = config["seed"].as<int>();
    ns3_config.num_nodes = config["robots_number"].as<uint32_t>();
    ns3_config.wifi_phy_type = config["wifi_type"].as<std::string>();
    ns3_config.wifi_phy_mode = config["phy_mode"].as<std::string>(); // Define a "Phy mode" that will be given to the WifiRemoteStationManager
    ns3_config.error_model = config["error_model_type"].as<std::string>();
    ns3_config.propagation_loss_model = config["propagation_loss_model"].as<std::string>();
    ns3_config.frequency = config["frequency"].as<double>();
    ns3_config.routing_algorithm = config["routing_algorithm"].as<std::string>();
    ns3_config.short_guard_interval_supported = config["short_guard_interval_supported"].as<bool>();
    ns3_config.use_real_routing_algorithm = config["use_real_routing_algorithm"].as<bool>();
    for (auto building : config["buildings"])
    {
        AABB_t b;
        b.x_min = building["x"].as<float>() - (building["size_x"].as<float>() / 2);
        b.x_max = building["x"].as<float>() + (building["size_x"].as<float>() / 2);
        b.y_min = building["y"].as<float>() - (building["size_y"].as<float>() / 2);
        b.y_max = building["y"].as<float>() + (building["size_y"].as<float>() / 2);
        b.z_min = 0.0;
        b.z_max = building["height"].as<float>();
        ns3_config.buildings.push_back(b);
    }

    ns3_config.enable_broadcast_flow = config["broadcast_flow"]["enable"].as<bool>(); 
    ns3_config.start_broadcast_time = config["broadcast_flow"]["start_time"].as<double>();
    ns3_config.stop_broadcast_time = config["broadcast_flow"]["stop_time"].as<double>();
    ns3_config.broadcast_packet_size = config["broadcast_flow"]["packet_size"].as<uint32_t>();
    ns3_config.broadcast_interval = config["broadcast_flow"]["interval"].as<uint32_t>(); // us
    ns3_config.broadcast_port = config["broadcast_flow"]["port"].as<uint16_t>();
    ns3_config.broadcast_timeout = config["broadcast_flow"]["timeout"].as<uint32_t>();


    ns3_config.enable_mission_flow = config["mission_flow"]["enable"].as<bool>();
    ns3_config.source_robots_ids = config["mission_flow"]["source_robot_ids"].as<std::vector<uint32_t>>();
    ns3_config.sink_robot_id = config["mission_flow"]["sink_robot_id"].as<uint32_t>();
    ns3_config.start_mission_time = config["mission_flow"]["start_time"].as<double>(); // s
    ns3_config.stop_mission_time = config["mission_flow"]["stop_time"].as<double>();   // s
    ns3_config.mission_packet_size = config["mission_flow"]["packet_size"].as<uint32_t>();     // bytes
    ns3_config.mission_interval = config["mission_flow"]["interval"].as<uint32_t>();           // us
    ns3_config.mission_port = config["mission_flow"]["port"].as<uint16_t>();
    ns3_config.mission_flow_id = config["mission_flow"]["flow_id"].as<uint8_t>();
    ns3_config.mission_timeout = config["mission_flow"]["timeout"].as<uint32_t>();

    ns3_config.enable_stats_module = config["enable_stats"].as<bool>();

    return ns3_config;
}

/**
 * @brief Configures the network in the ns-3 fashion
 * 
 * This is where the ns-3 nodes are created and configured. It holds most of the network configuration and options and is called once at the launch of the simulation.
 */
void Ns3Sim::ConfigureNs3(const ns3_configuration_t& ns3_config)
{
    // Set the seed for the random number generator
    SeedManager::SetRun(ns3_config.seed);

    // Create the nodes
    this->nodes.Create(ns3_config.num_nodes);

    // **************** MOBILITY MODULE ****************

    // For all the nodes, create a ConstantPositionMobilityModel to initialize their position.
    // Assign different positions for each robot as these values will be overwritten at first step of simulation anyway
    for (uint32_t i = 0; i < ns3_config.num_nodes; i++)
    {
        Ptr<MobilityModel> nodeMob;
        nodeMob = CreateObject<ConstantPositionMobilityModel>();
        nodeMob->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(i, i, 0.5));
        this->nodes.Get(i)->AggregateObject(nodeMob);
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of MOBILITY module");

    // **************** BUILDINGS MODULE ****************

    for (AABB_t build : ns3_config.buildings)
    {
        Ptr<Building> ns3_building = CreateObject<Building>();
        ns3_building->SetBoundaries(Box(build.x_min, build.x_max, build.y_min, build.y_max, build.z_min, build.z_max));
        ns3_building->SetBuildingType(Building::Office);
        ns3_building->SetExtWallsType(Building::ConcreteWithWindows);
        ns3_building->SetNFloors(1);
        ns3_building->SetNRoomsX(1);
        ns3_building->SetNRoomsY(1);

        RCLCPP_DEBUG(this->get_logger(), "Created a building with corners (%f, %f, %f, %f)", build.x_min, build.y_min, build.x_max, build.y_max);
    }

    // Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
    BuildingsHelper::Install(this->nodes);

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of BUILDINGS module");

    // **************** WIFI MODULE ****************

    // The NetDevices on which we will install WiFi
    NetDeviceContainer devices;

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(ns3_config.wifi_phy_mode), "ControlMode", StringValue(ns3_config.wifi_phy_mode));

    // Fix non-unicast WiFi data rate to be the same as that of unicast
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                        StringValue(ns3_config.wifi_phy_mode));

    // MAC layer
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // PHY layer (we support two WifiPhy types: YansWifiPhy and SpectrumWifiPhy)
    SpectrumWifiPhyHelper spectrumWifiPhy;
    YansWifiPhyHelper yansWifiPhy;

    Ptr<PropagationLossModel> propagationLossModel;

    if (ns3_config.wifi_phy_type == "YansWifiPhy")
    {
        YansWifiChannelHelper yansChannel;

        if (ns3_config.propagation_loss_model == "LogDistancePropagationLossModel")
        {
            yansChannel = YansWifiChannelHelper::Default();
            propagationLossModel = CreateObject<LogDistancePropagationLossModel>();
        }
        else if (ns3_config.propagation_loss_model == "HybridBuildingsPropagationLossModel")
        {
            propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
            propagationLossModel->SetAttribute("Frequency", DoubleValue(ns3_config.frequency));     // Default 2.4e9
            propagationLossModel->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0)); // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
            propagationLossModel->SetAttribute("ShadowSigmaIndoor", DoubleValue(8.0));   // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
            propagationLossModel->SetAttribute("ShadowSigmaOutdoor", DoubleValue(7.0));  // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
            yansChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                            "Frequency",            // Additional loss for each internal wall [dB]
                                            DoubleValue(ns3_config.frequency), // Default 2.4e9
                                            "ShadowSigmaExtWalls",  // Standard deviation of the normal distribution used to calculate the shadowing due to ext walls
                                            DoubleValue(5.0),       // Default 5
                                            "ShadowSigmaIndoor",    // Standard deviation of the normal distribution used to calculate the shadowing for indoor nodes
                                            DoubleValue(8.0),       // Default 8
                                            "ShadowSigmaOutdoor",   // Standard deviation of the normal distribution used to calculate the shadowing for outdoor nodes
                                            DoubleValue(7.0));
            yansChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        }

        yansWifiPhy.SetChannel(yansChannel.Create());
        yansWifiPhy.Set("TxPowerStart", DoubleValue(18));
        yansWifiPhy.Set("TxPowerEnd", DoubleValue(18));
        yansWifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

        devices = wifi.Install(yansWifiPhy, mac, this->nodes);

        yansWifiPhy.EnablePcap("ns-3_sim_pseudo_routing", devices);
    }
    else if (ns3_config.wifi_phy_type == "SpectrumWifiPhy")
    {
        Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

        // create the channel condition model
        Ptr<ChannelConditionModel> m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
        m_condModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));

        // create the propagation loss model and add it to the channel condition
        propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
        propagationLossModel->SetAttribute("Frequency", DoubleValue(ns3_config.frequency));
        propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(false));
        propagationLossModel->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));
        spectrumChannel->AddPropagationLossModel(propagationLossModel);

        // Create the delay model and add it to the channel condition
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);

        spectrumWifiPhy.SetChannel(spectrumChannel);
        spectrumWifiPhy.SetErrorRateModel(ns3_config.error_model);
        spectrumWifiPhy.Set("TxPowerStart", DoubleValue(14)); // dBm
        spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(14));
        spectrumWifiPhy.Set("ChannelSettings", StringValue("{0, 20, BAND_5GHZ, 0}"));
        spectrumWifiPhy.SetPcapDataLinkType(SpectrumWifiPhyHelper::DLT_IEEE802_11_RADIO);

        // Connect a callback that will save the pathloss value in an attribute every time it is calculated
        spectrumChannel->TraceConnectWithoutContext("PathLoss", MakeCallback(&Ns3Sim::SpectrumPathLossTrace, this));

        devices = wifi.Install(spectrumWifiPhy, mac, this->nodes);

        spectrumWifiPhy.EnablePcap("ns-3_sim_pseudo_routing", devices);
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(), "Unsupported WiFi type %s", ns3_config.wifi_phy_type.c_str());
        exit(EXIT_FAILURE);
    }

    this->m_propagationLossModel = propagationLossModel;

    // Short guard interval = 400ns (vs 800ns otherwise)
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        BooleanValue(ns3_config.short_guard_interval_supported));

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of WIFI module");

    /* **************** IP / ROUTING MODULE **************** */
    InternetStackHelper internet;

    // There is a great dichotomy between using a routing algorithm properly implemented in ns-3 such as OLSR or AODV, and using "pseudo-routing algorithms"
    // that use global information to form routes (non-realistic simulation)
    if (ns3_config.use_real_routing_algorithm)
    {
        if (ns3_config.routing_algorithm == "OLSR")
        {
            // Add OLSR routing
            Ipv4ListRoutingHelper ipv4List;
            OlsrHelper olsr;
            olsr.Set("HelloInterval", TimeValue(Seconds(0.5)));
            olsr.Set("TcInterval", TimeValue(Seconds(1)));
            ipv4List.Add(olsr, 100);
            internet.SetRoutingHelper(ipv4List);

            // Print routing table every 5 seconds
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
            olsr.PrintRoutingTableAllEvery(Seconds(20), routingStream);
        }
        else if (ns3_config.routing_algorithm == "AODV")
        {
            // Add AODV routing
            Ipv4ListRoutingHelper ipv4List;
            AodvHelper aodv;
            ipv4List.Add(aodv, 100);
            internet.SetRoutingHelper(ipv4List);

            // Print routing table every 5 seconds
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
            aodv.PrintRoutingTableAllEvery(Seconds(20), routingStream);
        }
        else if (ns3_config.routing_algorithm == "DSDV")
        {
            // Add DSDV routing
            DsdvHelper dsdv;
            dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(15)));
            dsdv.Set("SettlingTime", TimeValue(Seconds(6)));
            internet.SetRoutingHelper(dsdv);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Unsupported routing algorithm %s", ns3_config.routing_algorithm.c_str());
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        Ipv4StaticRoutingHelper staticRouting;
        internet.SetRoutingHelper(staticRouting);

        // Print routing table every 20 seconds
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
        staticRouting.PrintRoutingTableAllEvery(Seconds(20), routingStream);
    }


    // Actually install the internet stack on all nodes
    internet.Install(this->nodes);

    // Assign IP addresses to the net devices
    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign(devices);

    // Connect Trace callbacks on all nodes
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        // Access the internal list of aggregated objects
        Ptr<WifiMac> wifi_mac = devices.Get(i)->GetObject<WifiNetDevice>()->GetMac();
        Ptr<WifiPhy> wifi_phy = devices.Get(i)->GetObject<WifiNetDevice>()->GetPhy();

        this->mac_to_id[wifi_mac->GetAddress()] = i;
        wifi_phy->TraceConnect("PhyTxBegin", std::to_string(i), MakeCallback(&Ns3Sim::PhyTxBeginTrace, this));
        wifi_phy->TraceConnect("PhyRxEnd", std::to_string(i), MakeCallback(&Ns3Sim::PhyRxEndTrace, this));

        wifi_phy->TraceConnect("MonitorSnifferRx", std::to_string(i), MakeCallback(&Ns3Sim::MonitorSnifferRxTrace, this));
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of IP/ROUTING module");

    /* **************** APPLICATION MODULE **************** */

    // Broadcast flow
    if (ns3_config.enable_broadcast_flow)
    {
        // Random start, otherwise they never access the medium (I think)
        double min = 0.0;
        double max = 1.0;
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
        x->SetAttribute("Min", DoubleValue(min));
        x->SetAttribute("Max", DoubleValue(max));

        for (uint32_t i = 0; i < this->nodes.GetN(); i++)
        {
            Ptr<ns3::Node> node = this->nodes.Get(i);

            Ptr<FlockingBroadcaster> broadcaster = CreateObject<FlockingBroadcaster>();
            Ptr<FlockingReceiver> flock_receiver = CreateObject<FlockingReceiver>();

            broadcaster->SetStartTime(Seconds(ns3_config.start_broadcast_time + x->GetValue()));
            broadcaster->SetStopTime(Seconds(ns3_config.stop_broadcast_time));
            broadcaster->SetAttribute("PacketSize", UintegerValue(ns3_config.broadcast_packet_size));
            broadcaster->SetAttribute("Port", UintegerValue(ns3_config.broadcast_port));
            Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
            rand->SetAttribute("Constant", DoubleValue(ns3_config.broadcast_interval / 1000000.0f)); // from us to s (because custom class Sender uses Interval as Seconds)
            broadcaster->SetAttribute("Interval", PointerValue(rand));

            flock_receiver->SetStartTime(Seconds(0.0)); // Always have the receiver activated.

            if (broadcaster->TraceConnect("Tx", std::to_string(i), MakeCallback(&Ns3Sim::flocking_broadcaster_clbk, this)))
            {
                RCLCPP_DEBUG(this->get_logger(), "Connected broadcaster to node %d", i);
            }
            if (flock_receiver->TraceConnect("Rx", std::to_string(i), MakeCallback(&Ns3Sim::flock_receiver_clbk, this)))
            {
                RCLCPP_DEBUG(this->get_logger(), "Connected receiver to node %d", i);
            }

            // App 1 is the sender (broadcaster)
            // App 2 is the receiver
            node->AddApplication(broadcaster);
            node->AddApplication(flock_receiver);
        }
    }

    // "Mission" flow : unicast, unidirectional
    // "Sender" and "Receiver" are classes defined in the custom ns3 Application wifi-application.h
    if (ns3_config.enable_mission_flow)
    {
        // Install Mission sender application on all the source robots 
        for (uint32_t source_node_id : ns3_config.source_robots_ids)
        {
            Ptr<Sender> sender = CreateObject<Sender>();
            Ptr<ns3::Node> source_node = this->nodes.Get(source_node_id);
            sender->SetStartTime(Seconds(ns3_config.start_mission_time));
            sender->SetStopTime(Seconds(ns3_config.stop_mission_time));
            sender->SetAttribute("Destination", Ipv4AddressValue(this->nodes.Get(ns3_config.sink_robot_id)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()));
            sender->SetAttribute("Port", UintegerValue(ns3_config.mission_port));
            sender->SetAttribute("PacketSize", UintegerValue(ns3_config.mission_packet_size));
            sender->SetAttribute("NumPackets", UintegerValue(4294967295));  // do not limit number of sent packets (infinite flow)
            sender->SetAttribute("FlowId", UintegerValue(ns3_config.mission_flow_id));
            Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
            rand->SetAttribute("Constant", DoubleValue(ns3_config.mission_interval / 1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
            sender->SetAttribute("Interval", PointerValue(rand));
            sender->TraceConnectWithoutContext("Tx", MakeCallback(&Ns3Sim::mission_flow_sender_clbk, this));
            
            // App 3 is the Mission flow sender (unicast, only for leaders)
            source_node->AddApplication(sender);

            RCLCPP_INFO(this->get_logger(), "Configured mission app (source) for node %d", source_node_id);
        }

        // Install the Mission receiver application on the sink robot
        Ptr<Receiver> receiver = CreateObject<Receiver>();
        Ptr<ns3::Node> sink_node = this->nodes.Get(ns3_config.sink_robot_id);
        receiver->SetStartTime(Seconds(0.0));
        receiver->SetAttribute("Port", UintegerValue(ns3_config.mission_port));
        receiver->TraceConnectWithoutContext("Rx", MakeCallback(&Ns3Sim::mission_flow_receiver_clbk, this));
        
        sink_node->AddApplication(receiver);

        RCLCPP_INFO(this->get_logger(), "Configured mission app (sink) for node %d", ns3_config.sink_robot_id);
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of APPLICATION module");

    /* **************** STATS MODULE **************** */
    if (ns3_config.enable_stats_module)
    {
        std::string strategy = "VAT flocking";

        this->data.DescribeRun(this->experience_name, strategy, this->config_file_path, std::to_string(this->run_id));

        // Add any information we wish to record about this run.
        this->data.AddMetadata("author", "tbalaguer");

        // Create a counter for the number of Packets sent WITHIN THE TARGETS
        this->missionTotalTx = CreateObject<PacketCounterCalculator>();
        this->missionTotalTx->SetKey("mission_sent_packets_within_target");
        // this->missionTotalTx->SetContext("mission flow node[" + std::to_string(source_node_id) + "]");
        this->data.AddDataCalculator(this->missionTotalTx);

        // Create a counter for the number of Packets received WITHIN THE TARGETS
        this->missionTotalRx = CreateObject<PacketCounterCalculator>();
        this->missionTotalRx->SetKey("mission_received_packets_within_target");
        // this->missionTotalRx->SetContext("mission flow node[" + std::to_string(sink_node_id) + "]");
        this->data.AddDataCalculator(this->missionTotalRx);

        // Create a statistics object for the delay of the packets
        this->missionDelay = CreateObject<TimeMinMaxAvgTotalCalculator>();
        this->missionDelay->SetKey("mission_packet_delay");
        this->missionDelay->SetContext("mission flow");
        this->data.AddDataCalculator(this->missionDelay);

        // Create a counter for the number of Packets received by each 'navigation flow' server
        for (int i = 0; i < this->nodes.GetN(); i++)
        {
            Ptr<PacketCounterCalculator> navTotalRx = CreateObject<PacketCounterCalculator>();
            navTotalRx->SetKey("nav_received_packets");
            navTotalRx->SetContext("nav flow node[" + std::to_string(i) + "]");
            this->navTotalRxVector.push_back(navTotalRx);
            this->data.AddDataCalculator(this->navTotalRxVector[i]);
        }

        // Connect the callback to all the trace sources
        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&Ns3Sim::PhyTxEndTrace, this));

        this->navEffectiveTx = CreateObject<PacketCounterCalculator>();
        navEffectiveTx->SetKey("nav_effective_tx");
        navEffectiveTx->SetContext("nav flow effectively sent packets");
        this->data.AddDataCalculator(this->navEffectiveTx);

        // Create a counter for the number of Packets sent by each 'navigation flow' client
        for (int i = 0; i < this->nodes.GetN(); i++)
        {
            Ptr<PacketCounterCalculator> navTotalTx = CreateObject<PacketCounterCalculator>();
            navTotalTx->SetKey("nav_sent_packets");
            navTotalTx->SetContext("nav flow node[" + std::to_string(i) + "]");
            this->navTotalTxVector.push_back(navTotalTx);
            this->data.AddDataCalculator(this->navTotalTxVector[i]);
        }
        RCLCPP_DEBUG(this->get_logger(), "Finished the configuration of STATS module");
    }
    
}

void Ns3Sim::RunPseudoRoutingAlgorithm(const std::string pseudo_routing_algo_name)
{

    double bandwidth = DynamicCast<WifiNetDevice>(this->nodes.Get(0)->GetDevice(0))->GetPhy()->GetChannelWidth() * 1e6;

    // Pseudo Routing algorithm
    std::vector<std::vector<std::pair<int, double>>> graph;
    // Create a graph object where the nodes are the agents, and the edges exist only if they received a broadcast message from the neighbor, and weighted according to a cost function
    for (int16_t i = 0; i < this->nodes.GetN(); i++)
    {
        std::vector<std::pair<int, double>> neighbors;
        for (int j = 0; j < this->nodes.GetN(); j++)
        {
            if (i == j)
                continue;
            // j is not a "broadcast neighbor" of i
            if (this->broadcast_neighbors[i].count(j) == 0)
                continue;
            if (pseudo_routing_algo_name == "shortest_dist")
            {
                double distance = this->nodes.Get(i)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(j)->GetObject<MobilityModel>());
                double cost = costFunction(distance, shortest_dist_r1, shortest_dist_c1);
                neighbors.push_back(std::make_pair(j, cost));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with cost %f", i, j, cost);
            }
            else if (pseudo_routing_algo_name == "capacity_bottleneck")
            {
                double capacity = bandwidth * std::log2(1 + this->snrs[i][j]);
                neighbors.push_back(std::make_pair(j, capacity));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with capacity %f", i, j, capacity);
            }
            else if (pseudo_routing_algo_name == "minimize_error_rate")
            {
                // purge old packets
                while (Simulator::Now() - this->broadcast_received_packets[i][j][0] > this->broadcast_window_size)
                {
                    this->broadcast_received_packets[i][j].erase(this->broadcast_received_packets[i][j].begin());
                }
                // Compute error rate
                uint32_t expected_packets_count = this->broadcast_window_size.ToInteger(Time::US) / ns3_config.broadcast_interval;
                double success_prob = this->broadcast_received_packets[i][j].size() / expected_packets_count;
                neighbors.push_back(std::make_pair(j, -std::log(success_prob)));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with success rate %f (%d/%d)", i, j, success_prob, this->broadcast_received_packets[i][j].size(), expected_packets_count);
            }
            else if (pseudo_routing_algo_name == "hybrid_dist_error_rate")
            {
                // purge old packets
                while (Simulator::Now() - this->broadcast_received_packets[i][j][0] > this->broadcast_window_size)
                {
                    this->broadcast_received_packets[i][j].erase(this->broadcast_received_packets[i][j].begin());
                }
                // Compute error rate
                uint32_t expected_packets_count = this->broadcast_window_size.ToInteger(Time::US) / ns3_config.broadcast_interval;
                double error_rate = 1 - (this->broadcast_received_packets[i][j].size() / expected_packets_count);
                double distance = this->nodes.Get(i)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(j)->GetObject<MobilityModel>());
                double cost = hybrid_dist_error_rate_cost_function(distance, hybrid_r1, hybrid_c1, error_rate, hybrid_e1, hybrid_k1, hybrid_k2);
                neighbors.push_back(std::make_pair(j, cost));
                RCLCPP_DEBUG(this->get_logger(), "Adding edge from %d to %d with error rate %f (%d/%d)", i, j, error_rate, this->broadcast_received_packets[i][j].size(), expected_packets_count);
            }
            else if (pseudo_routing_algo_name == "predefined_chain")
            {
                continue;
            }
            else
            {
                RCLCPP_FATAL(this->get_logger(), "Pseudo routing algorithm %s not supported", pseudo_routing_algo_name);
                exit(EXIT_FAILURE);
            }
        }
        graph.push_back(neighbors);
    }

    this->mission_neighbors.clear();

    for (uint32_t source_node_id : this->ns3_config.source_robots_ids)
    {
        int source = source_node_id;
        std::vector<double> dist;
        std::vector<int> parent;

        if (pseudo_routing_algo_name == "shortest_dist" || pseudo_routing_algo_name == "minimize_error_rate")
        {
            dijkstra(source, graph, dist, parent);
        }
        else if (pseudo_routing_algo_name == "capacity_bottleneck")
        {
            dijkstra_bottleneck(source, graph, dist, parent);
            double cost = dist[ns3_config.sink_robot_id];
            // std::ofstream file;
            // file.open(capacity_cost_out_file, std::ios::app);
            // file << Simulator::Now().ToDouble(Time::MS) << "," << cost << std::endl;
            // file.close();
        }
        else if (pseudo_routing_algo_name == "hybrid_dist_error_rate")
        {
            dijkstra(source, graph, dist, parent);
        }
        else if (pseudo_routing_algo_name == "predefined_chain")
        {
            for (int i = 0; i < this->nodes.GetN() - 1; i++)
            {
                parent.push_back(i + 1);
            }
            parent[source] = -1;
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Pseudo routing algorithm %s not supported", pseudo_routing_algo_name);
            exit(EXIT_FAILURE);
        }

        std::stack<int> path;
        std::vector<int> path_list;
        int current = this->ns3_config.sink_robot_id;

        // Trace back from destination to source using the parent array
        while (current != -1)
        {
            path.push(current);
            path_list.push_back(current);
            current = parent[current];
        }

        // No path to source
        if (path.top() != source)
        {
            RCLCPP_WARN(this->get_logger(), "No path to source !!");
            continue;
        }

        if (!this->ns3_config.use_real_routing_algorithm)
        {
            updateStaticIpv4Routes(path_list);
        }

        int curr = source;
        // std::cout << "Path: " << source;
        while (path.size() > 1)
        {
            path.pop();
            int neighbor = path.top();
            double distance = this->nodes.Get(curr)->GetObject<MobilityModel>()->GetDistanceFrom(this->nodes.Get(neighbor)->GetObject<MobilityModel>());

            this->mission_neighbors[curr][neighbor] = std::make_pair(this->pathlosses[curr][neighbor], Simulator::Now());
            this->mission_neighbors[neighbor][curr] = std::make_pair(this->pathlosses[neighbor][curr], Simulator::Now());
            // std::cout << " -> " << neighbor ;
            curr = neighbor;
        }
        // std::cout << std::endl;
    }

}

void Ns3Sim::update_target_clbk(dancers_msgs::msg::Target msg)
{
    std::cout << "Target received: " << msg.target_id << std::endl;
    Ptr<ns3::Node> target_node = this->nodes.Get(msg.target_id);
    // Any Agent has a broadcast sender and receiver, prevent duplicate mission application by checking the number of applications.
    // We also do a double-check in case this agent was already considered a source agent
    if (target_node->GetNApplications() < 3 && std::find(this->ns3_config.source_robots_ids.begin(), this->ns3_config.source_robots_ids.end(), msg.target_id) == this->ns3_config.source_robots_ids.end())
    {
        // add sender application
        Ptr<Sender> sender = CreateObject<Sender>();
        target_node->AddApplication(sender);
        sender->SetStartTime(Seconds(this->ns3_config.start_mission_time));
        sender->SetStopTime(Seconds(this->ns3_config.stop_mission_time));
        sender->SetAttribute("Destination", Ipv4AddressValue(this->nodes.Get(this->ns3_config.sink_robot_id)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()));
        sender->SetAttribute("Port", UintegerValue(this->ns3_config.mission_port));
        sender->SetAttribute("PacketSize", UintegerValue(this->ns3_config.mission_packet_size));
        sender->SetAttribute("NumPackets", UintegerValue(4294967295));
        sender->SetAttribute("FlowId", UintegerValue(this->ns3_config.mission_flow_id));
        Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
        rand->SetAttribute("Constant", DoubleValue(ns3_config.mission_interval / 1000000.0)); // from us to s (because custom class Sender uses Interval as Seconds)
        sender->SetAttribute("Interval", PointerValue(rand));
        sender->TraceConnectWithoutContext("Tx", MakeCallback(&Ns3Sim::mission_flow_sender_clbk_2, this));

        this->ns3_config.source_robots_ids.push_back(msg.target_id);

        RCLCPP_INFO(this->get_logger(), "Configured mission app (source) for node %d", target_node->GetId());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Node %d already has 2 applications", msg.target_id);
    }
}

/**
 * @brief Callback for the SpectrumChannel "PathLoss" trace
 *
 * It saves the pathloss value computed by the spectrum propagation module in the pathloss matrix
 */
void Ns3Sim::SpectrumPathLossTrace(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t txId = txPhy->GetDevice()->GetNode()->GetId();
    uint32_t rxId = rxPhy->GetDevice()->GetNode()->GetId();
    this->pathlosses[txId][rxId] = -lossDb;
    // std::cout << "Tx: " << txId <<  " Rx: " << rxId << " Loss: " << lossDb << std::endl;
}

/**
 * @brief Callback for the WifiPhy "PhyTxBegin" trace
 *
 * When a packet tagged with FlowId "mission" is being sent at the PHY level, it saves the MAC layer "destination" (so, the next hop) of the packet as a neighbor.
 */
void Ns3Sim::PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPow)
{
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t destId = this->mac_to_id.find(hdr.GetAddr1())->second;

    // From a networking POV, this is cheating, we are sending a message, we can't know what is the reception power, however here it is accepted
    double rxPow = this->pathlosses[nodeId][destId];
    Time txTime = Simulator::Now();

    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->ns3_config.mission_flow_id)
        {
            // We are sending a "mission" packet, link-layer destination should be our (outgoing) neighbor
            this->mission_neighbors[nodeId][destId] = std::make_pair(rxPow, txTime);
        }
    }
}

/**
 * @brief Callback for the WifiPhy "PhyTxEnd" trace
 *
 * Used only for stats here
 */
void Ns3Sim::PhyTxEndTrace(Ptr<const Packet> packet)
{
    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->ns3_config.mission_flow_id)
        {
            this->navEffectiveTx->PacketUpdate("", packet);
        }
    }
}

/**
 * @brief Callback for the WifiPhy "PhyRxEnd" trace
 *
 * When a packet tagged with FlowId "mission" is being received at the PHY level, it saves the MAC layer "source" (so, the previous hop) of the packet as a neighbor.
 */
void Ns3Sim::PhyRxEndTrace(std::string context, Ptr<const Packet> packet)
{
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t nodeId = std::stoi(context);
    uint32_t sourceId = this->mac_to_id.find(hdr.GetAddr2())->second;
    uint32_t destId = this->mac_to_id.find(hdr.GetAddr1())->second;

    // There should be a way of reading this directly from the packet, but this is easier (?)
    double rxPow = this->pathlosses[nodeId][sourceId];
    Time rxTime = Simulator::Now();

    // We "heard" a packet from a peer, the link-layer source is a broadcast neighbor
    // this->broadcast_neighbors[nodeId][sourceId] = std::make_pair(rxPow, rxTime);

    FlowIdTag nav_flow;
    if (packet->PeekPacketTag(nav_flow))
    {
        if (nav_flow.GetFlowId() == this->ns3_config.mission_flow_id && nodeId == destId)
        {
            // We are receiving a "mission" packet, link-layer destination should be our (outgoing) neighbor
            this->mission_neighbors[nodeId][sourceId] = std::make_pair(rxPow, rxTime);
        }
    }
}

/**
 * @brief Callback for the mission flow receiver
 */
void Ns3Sim::mission_flow_receiver_clbk(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Mission flow packet received from.");

    if (this->save_mission_packets)
    {
        // print packet to file
        myTimestampTag timestamp;
        // Should never not be found since the sender is adding it, but
        // you never know.
        if (packet->FindFirstMatchingByteTag(timestamp))
        {
            Time tx = timestamp.GetTimestamp();
            Time now = Simulator::Now();
            Time delay = now - tx;

            std::ofstream f(this->m_mission_packets_file, std::ios::app);
            f << now.ToInteger(Time::US) << "," << delay.ToInteger(Time::US) << std::endl;
            f.close();
        }
    }
}

/**
 * @brief Callback for the mission flow sender
 */
void Ns3Sim::mission_flow_sender_clbk(Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Mission flow packet sent");
}

/**
 * @brief Callback for the mission flow sender
 */
void Ns3Sim::mission_flow_sender_clbk_2(Ptr<const Packet> packet)
{
    RCLCPP_INFO(this->get_logger(), "Mission flow packet sent BY DYNAMIC SOURCE NODE");
}

/**
 * @brief Callback for the reception of a packet at the application layer
 */
void Ns3Sim::flock_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    RCLCPP_DEBUG(this->get_logger(), "Flock packet received");

    Time rxTime = Simulator::Now();
    double rxPow = this->pathlosses[std::stoi(context)][peer_id];

    this->broadcast_neighbors[std::stoi(context)][peer_id] = std::make_pair(rxPow, rxTime);
    this->broadcast_received_packets[std::stoi(context)][peer_id].push_back(rxTime);
}

/**
 * @brief Callback for the transmission of a packet at the application layer
 */
void Ns3Sim::flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet)
{
    RCLCPP_DEBUG(this->get_logger(), "Flocking broadcast packet sent");
}

void Ns3Sim::MonitorSnifferRxTrace(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    uint32_t nodeId = std::stoi(context);
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    uint32_t sourceId = this->mac_to_id.find(hdr.GetAddr2())->second;

    this->snrs[nodeId][sourceId] = (double)pow(10, signalNoise.signal / 10.0) / (double)pow(10, signalNoise.noise / 10.0);

    RCLCPP_DEBUG(this->get_logger(), "[%d] Packet heard from %d, SNR = %f", nodeId, sourceId, this->snrs[nodeId][sourceId]);
}

/**
 * @brief Generates an ordered_neighbors_msg containing the neighbors of each node, sorted by pathloss, and serializes it
 *
 * @returns A string-serialized version of the "ordered_neighbors" protobuf message
 */
std::string
Ns3Sim::generateNeighborsMsg()
{
    std::map<uint32_t, std::map<uint32_t, std::pair<double, Time>>> neighbors;
    std::map<uint32_t, dancers_update_proto::OrderedNeighbors_Role> agent_roles;

    dancers_update_proto::OrderedNeighborsList ordered_neighbors_msg;

    // Assign roles based on their neighborhood type
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        bool has_broadcast_neighbor = this->broadcast_neighbors.find(i) != this->broadcast_neighbors.end() && !this->broadcast_neighbors[i].empty();
        bool has_mission_neighbor = this->mission_neighbors.find(i) != this->mission_neighbors.end() && !this->mission_neighbors[i].empty();

        if (has_mission_neighbor)
        {
            // Use mission neighbors
            // std::cout << "Using mission neighbors for node " << i << std::endl;
            neighbors[i] = this->mission_neighbors[i];
            agent_roles[i] = dancers_update_proto::OrderedNeighbors_Role_MISSION;
        }
        else if (has_broadcast_neighbor)
        {
            // Use broadcast neighbors, we don't have any mission neighbors !
            // std::cout << "Using broadcast neighbors for node " << i << std::endl;
            neighbors[i] = this->broadcast_neighbors[i];
            agent_roles[i] = dancers_update_proto::OrderedNeighbors_Role_POTENTIAL;
        }
        else if (!has_broadcast_neighbor)
        {
            // We don't have any neighbors !
            // std::cout << "No neighbors for node " << i << std::endl;
            neighbors[i] = std::map<uint32_t, std::pair<double, Time>>();
            agent_roles[i] = dancers_update_proto::OrderedNeighbors_Role_UNDEFINED;
        }
    }

    // Separate "Idle" agents (no neighbor with the role "Mission") from the "Potential" pool
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        if (agent_roles[i] == dancers_update_proto::OrderedNeighbors_Role_POTENTIAL)
        {
            bool has_neighbors_with_role_mission = false;
            for (auto const &agent : neighbors[i])
            {
                if (agent_roles[agent.first] == dancers_update_proto::OrderedNeighbors_Role_MISSION)
                {
                    has_neighbors_with_role_mission = true;
                    break;
                }
            }
            if (!has_neighbors_with_role_mission)
            {
                agent_roles[i] = dancers_update_proto::OrderedNeighbors_Role_IDLE;
            }
        }
    }

    // Create and add the ordered_neighbors protobuf message
    for (auto const &agent : neighbors)
    {
        // Sort the agents by descending pathloss
        std::vector<std::pair<uint32_t, std::pair<double, Time>>> ordered_neighbors(agent.second.begin(), agent.second.end());
        std::sort(ordered_neighbors.begin(), ordered_neighbors.end(), [](const auto &a, const auto &b)
                  { return a.second.first > b.second.first; });

        dancers_update_proto::OrderedNeighbors *neighbor_msg = ordered_neighbors_msg.add_ordered_neighbors();
        neighbor_msg->set_agentid(agent.first);
        neighbor_msg->set_role(agent_roles[agent.first]);
        // Sort the pathlosses to add them in the protobuf message in the right order.
        for (auto const &neighbor : ordered_neighbors)
        {
            uint32_t neighbor_id = neighbor.first;
            neighbor_msg->add_neighborid(neighbor_id);
            neighbor_msg->add_linkquality(neighbor.second.first);
            neighbor_msg->add_neighbortype(agent_roles[neighbor_id]);
            neighbor_msg->add_time_val(neighbor.second.second.ToInteger(Time::US));
        }
    }

    // std::cout << ordered_neighbors_msg.DebugString() << std::endl;

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
Ns3Sim::generateResponseProtobuf()
{
    // Change message type to "END"
    dancers_update_proto::DancersUpdate network_update_msg;
    network_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);
    network_update_msg.set_payload(gzip_compress(generateNeighborsMsg()));
    std::string str_response;
    network_update_msg.SerializeToString(&str_response);

    return str_response;
}

/**
 * @brief Clear timed out neighbors
 *
 * Removes neighbors that have not been heard for a while.
 */
void Ns3Sim::timeoutNeighbors()
{
    // remove timed-out neighbors from broadcast_neighbors
    for (std::pair<uint32_t, std::map<uint32_t, std::pair<double, Time>>> const &agent : this->broadcast_neighbors)
    {
        for (std::pair<uint32_t, std::pair<double, Time>> const &neigh : agent.second)
        {
            if (Simulator::Now() - neigh.second.second > MicroSeconds(this->ns3_config.broadcast_timeout))
            {
                // std::cout << "Removing broadcast neighbor " << neigh.first << " from agent " << agent.first << std::endl;
                this->broadcast_neighbors[agent.first].erase(neigh.first);
            }
        }
    }
    // remove timed-out neighbors from mission_neighbors
    for (std::pair<uint32_t, std::map<uint32_t, std::pair<double, Time>>> const &agent : this->mission_neighbors)
    {
        for (auto const &neigh : agent.second)
        {
            if (Simulator::Now() - neigh.second.second > MicroSeconds(this->ns3_config.mission_timeout))
            {
                // std::cout << "Removing mission neighbor " << neigh.first << " from agent " << agent.first << std::endl;
                this->mission_neighbors[agent.first].erase(neigh.first);
            }
        }
    }
}

/**
 * @brief Update the pathlosses matrix when the YansWifiPhy model is used
 *
 * Explanation for this is that the YansWifiPhy model does not provide the same "pathloss" traceSource that the SpectrumChannel entity provides, thus, we need to actually compute and save the pathloss values between nodes, when we use the YansWifiPhy model
 */
void Ns3Sim::updateNeighborsPathloss()
{
    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < this->nodes.GetN(); j++)
        {
            if (i != j)
            {
                Ptr<MobilityModel> mobilityA = this->nodes.Get(i)->GetObject<MobilityModel>();
                Ptr<MobilityModel> mobilityB = this->nodes.Get(j)->GetObject<MobilityModel>();

                double rxPow = this->m_propagationLossModel->CalcRxPower(18.0, mobilityA, mobilityB);
                this->pathlosses[i][j] = rxPow; // it's not really a pathloss but it's the same with a constant difference
                // std::cout << "Node " << i << " -> Node " << j << " : " << rxPow << std::endl;
            }
        }
    }
}



void Ns3Sim::updateStaticIpv4Routes(const std::vector<int> path)
{
    int sink = path[0];
    Ipv4Address sink_addr = this->nodes.Get(sink)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress();
    for (int i = 1; i < path.size(); i++)
    {
        int source = path[i];
        int next_hop = path[i - 1];

        Ipv4Address next_hop_addr = this->nodes.Get(next_hop)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress();

        Ptr<Ipv4StaticRouting> staticRouting = DynamicCast<Ipv4StaticRouting>(this->nodes.Get(source)->GetObject<Ipv4>()->GetRoutingProtocol());
        if (staticRouting == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Ipv4StaticRouting not found in node %d", source);
            return;
        }
        while (staticRouting->GetNRoutes() > 2)
        {
            RCLCPP_DEBUG(this->get_logger(), "Removing routes from node %d", source);
            staticRouting->RemoveRoute(2);
        }
        staticRouting->AddHostRouteTo(sink_addr, next_hop_addr, 1, 0);

        RCLCPP_DEBUG(this->get_logger(), "Added route %d -> %d via %d", source, sink, next_hop);
    }
}

void Ns3Sim::Loop()
{
    if (cosim_mode)
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
        RCLCPP_INFO(this->get_logger(), "\x1b[32m Socket connected with Coordinator \x1b[0m");

        uint64_t bytes_received = 0;

        // **************** MAIN SIMULATION LOOP ****************
        while ((currTime < simEndTime || simEndTime == Seconds(0.0)) && rclcpp::ok())
        {

            // Wait until reception of a message on the UDS socket
            std::string received_data = gzip_decompress(socket->receive_one_message());

            // Initialize empty protobuf message type [PhysicsUpdate]
            dancers_update_proto::DancersUpdate physics_update_msg;
            // Transform the message received from the UDS socket [string] -> [protobuf]
            physics_update_msg.ParseFromString(received_data);

            // Read the "physical" information transmitted by the NetworkCoordinator, and update the node's positions
            // Also verifies that the number of nodes sent by the NetworkCoordinator corresponds to the number of existing nodes in NS-3
            rclcpp::Clock clock;
            dancers_update_proto::PoseVector robots_positions_msg;

            // if (currTime % Seconds(0.1) == Time(0)){
            //     uint64_t bytes_received_this_iteration = mission_server->GetReceived()*packet_size - bytes_received;
            //     bytes_received = mission_server->GetReceived()*packet_size;
            //     RCLCPP_INFO(this->get_logger(), "Mission flow throughput: %f Mbps", (float)(bytes_received_this_iteration * 10 / 1000000.0));
            // }

            if (!physics_update_msg.payload().empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "Received robots positions from Coordinator");
                robots_positions_msg.ParseFromString(gzip_decompress(physics_update_msg.payload()));

                // Verify that the number of positions (vectors of 7 values [x, y, z, qw, qx, qy, qz]) sent by the robotics simulator corresponds to the number of existing nodes in NS-3
                // Then, update the node's positions (orientation is ignored for now)
                if (this->nodes.GetN() != (uint32_t)robots_positions_msg.pose_size())
                {
                    if (verbose)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(),
                                             clock,
                                             1000, // ms
                                             "Network simulator received position information of %i robots but NS-3 has %u nodes.",
                                             robots_positions_msg.pose_size(),
                                             this->nodes.GetN());
                    }
                }
                else
                {
                    for (uint32_t i = 0; i < this->nodes.GetN(); i++)
                    {
                        Vector pos;
                        pos.x = robots_positions_msg.pose(i).x();
                        pos.y = robots_positions_msg.pose(i).y();
                        pos.z = robots_positions_msg.pose(i).z();

                        // ns-s don't like negative and 0 positions
                        if (pos.z <= 0)
                        {
                            pos.z = 0.1;
                        }

                        // Set node position
                        Ptr<MobilityModel> mobility = this->nodes.Get(i)->GetObject<MobilityModel>();
                        mobility->SetPosition(pos);
                    }
                }
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Network simulator received an update message with empty robots positions");
            }

            if (this->ns3_config.wifi_phy_type == "YansWifiPhy")
            {
                this->updateNeighborsPathloss();
            }

            // Once all the events are scheduled, advance W time in the simulation and stop
            Simulator::Stop(step_size);

            if (this->save_compute_time)
            {
                this->probe.start();
            }

            Simulator::Run();

            if (this->save_compute_time)
            {
                this->probe.stop();
            }

            this->timeoutNeighbors();

            this->RunPseudoRoutingAlgorithm(this->pseudoRoutingAlgo);

            // Create an object giving the neighborhood of each node, based on the packets received by their UDP server, neighbors lifetime is 1 second.
            // std::map<uint32_t, std::map<uint32_t, double>> neighbors = create_neighbors(neighbor_timeout_value);

            std::string response = gzip_compress(generateResponseProtobuf());

            // Send the response to the network coordinator
            socket->send_one_message(response);

            currTime += step_size;
        }
    }
    else
    {
        Simulator::Stop(simEndTime);
        std::cout << "Starting simulation." << std::endl;

        if (this->save_compute_time)
        {
            this->probe.start();
        }

        Simulator::Run();

        if (this->save_compute_time)
        {
            this->probe.stop();
        }
    }
    RCLCPP_INFO(this->get_logger(), "Simulation finished.");

    if (stats_enabled)
    {
        Ptr<DataOutputInterface> output = CreateObject<SqliteDataOutput>();

        if (output)
        {
            output->SetFilePrefix(this->experience_name);
            output->Output(data);
        }

        if (mission_flow)
        {
            // std::cout << "Mission source sent " << sender->GetSent() << " packets" << std::endl;
            // std::cout << "Mission sink received " << receiver->GetReceived() << " packets ( " << (float)(100.0 * receiver->GetReceived() / (float)sender->GetSent()) << "% )" << std::endl;
        }
    }

    Simulator::Destroy();

    exit(EXIT_SUCCESS);

}

/**
 * @brief Cost function
 *
 * Here, this cost function is used for the Dijkstra algorithm costs. As a function of ditance, it is a flat function that becomes exponential at r1
 */
double Ns3Sim::costFunction(double dist, double r1, double c1)
{
    if (dist < r1)
    {
        return c1;
    }
    else
    {
        return pow((dist - r1), 2) + c1;
    }
}

/**
 * @brief Cost function
 *
 * Here, this cost function is used for the Dijkstra algorithm costs. As a function of ditance, it is a flat function that becomes exponential at r1
 */
double Ns3Sim::hybrid_dist_error_rate_cost_function(double dist, double r1, double c1, double error_rate, double e1, double k1, double k2)
{
    return k1 * Ns3Sim::costFunction(dist, r1, c1) + k2 * exp(e1 * error_rate);
}

/**
 * \brief The main function, spins the ROS2 Node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ns3Sim>());
    rclcpp::shutdown();
    return 0;
}

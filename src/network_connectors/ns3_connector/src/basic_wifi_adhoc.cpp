#include <connector_core/connector.hpp>

// ns-3 libs
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/propagation-loss-model.h>
#include "ns3/buildings-module.h"

// Home-made ns-3 apps
#include <applications/waypoint_broadcast.h>

// Common DANCERS libs
#include <agent.hpp>
#include <yaml_util.hpp>

// C++ libs
#include <shared_mutex>

// Custom ROS2 messages
#include <dancers_msgs/msg/agent_struct.hpp>

// Protobuf messages
#include <protobuf_msgs/pose_vector.pb.h>

using namespace ns3;

class BasicWifiAdhoc : public Connector
{
public:
    BasicWifiAdhoc()
    : Connector("basic_wifi_adhoc")
    {
        RCLCPP_INFO(this->get_logger(), "BasicWifiAdhoc (NET) started");

        // Read relevant information from the configuration file
        this->ReadConfigFile();

        // Configure global ns-3 objects
        this->ConfigureNs3();

        // Initialize the agents and the ad-hoc Wi-Fi network
        this->InitAgents();
        
        // Add a waypoint broadcaster to the leader (arbitrarily, agent 0)
        auto broadcast_app = this->AddWaypointBroadcaster(this->nodes.Get(0));
        // Add a waypoint receiver to the followers
        for (uint32_t i = 1; i < this->nodes.GetN(); i++)
        {
            this->AddWaypointReceiver(this->nodes.Get(i));
        }
        
        // Initiate the random waypoint generation procedure for the leader
        // verify random_waypoint exists in config
        if (this->config_["random_waypoint"].IsDefined())
        {
            RCLCPP_INFO(this->get_logger(), "Random waypoint generation enabled");

            this->RandomWaypointGeneration(broadcast_app, 
                    MicroSeconds(getYamlValue<unsigned int>(this->config_["random_waypoint"], "new_waypoint_interval")), 
                    Vector(
                        getYamlValue<double>(this->config_["random_waypoint"]["arena_corner_1"], "x"), 
                        getYamlValue<double>(this->config_["random_waypoint"]["arena_corner_1"], "y"), 
                        getYamlValue<double>(this->config_["random_waypoint"]["arena_corner_1"], "z")),
                    Vector(
                        getYamlValue<double>(this->config_["random_waypoint"]["arena_corner_2"], "x"), 
                        getYamlValue<double>(this->config_["random_waypoint"]["arena_corner_2"], "y"), 
                        getYamlValue<double>(this->config_["random_waypoint"]["arena_corner_2"], "z")));
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "random_waypoint section not found in config file, aborting.");
            exit(EXIT_FAILURE);
        }

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&BasicWifiAdhoc::Loop, this);
    }

private:
    /**
     * @brief Read config file and set variables
     * 
     * It is mandatory to set the socket variables to enable the connector
     * to communicate with the simulator !
     */
    void ReadConfigFile() override
    {
        this->use_uds = getYamlValue<bool>(this->config_, "net_use_uds");
        this->uds_server_address = getYamlValue<std::string>(this->config_, "net_uds_server_address");
        this->ip_server_address = getYamlValue<std::string>(this->config_, "net_ip_server_address");
        this->ip_server_port = getYamlValue<unsigned int>(this->config_, "net_ip_server_port");

        // Read the synchronization interval and the step length as ints to easily verify that they are compatible
        unsigned int sync_window_int = getYamlValue<unsigned int>(this->config_, "sync_window");
        unsigned int step_size_int = getYamlValue<unsigned int>(this->config_, "net_step_size");
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the network step size, aborting.");
            exit(EXIT_FAILURE);
        }
        this->step_size = step_size_int / 1000000.0f;
        this->it_end_sim = uint64_t(this->simulation_length / this->step_size);

    }

    /**
     * @brief Function called at each step, the connector is responsible to read the update message, step the simulator, 
     * and return the repsonse update message
     */
    dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate update_msg) override
    {
        this->UpdateAgentsPosition(update_msg);

        // Once all the events are scheduled, advance W time in the simulation and stop
        Simulator::Stop(Seconds(step_size));

        Simulator::Run();

        RCLCPP_DEBUG(this->get_logger(), "Finished iteration %ld", this->it);
        
        return this->GenerateResponseProtobuf();
    }

    /**
     * @brief Generate the response protobuf message to send to the coordinator
     */
    dancers_update_proto::DancersUpdate GenerateResponseProtobuf()
    {
        dancers_update_proto::DancersUpdate response_msg;
        response_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

        dancers_update_proto::PoseVector waypoints_vector;
        for (const auto& [agent_id, waypoint] : this->current_waypoints_)
        {
            std::shared_lock lock(this->agents_mutex_);
            dancers_update_proto::Pose* waypoint_msg = waypoints_vector.add_pose();
            waypoint_msg->set_x(waypoint.x + this->agents_.at(agent_id)->initial_position.x());
            waypoint_msg->set_y(waypoint.y + this->agents_.at(agent_id)->initial_position.y());
            waypoint_msg->set_z(waypoint.z + this->agents_.at(agent_id)->initial_position.z());
            waypoint_msg->set_agent_id(agent_id);
        }
        std::string waypoints_serialized;
        waypoints_vector.SerializeToString(&waypoints_serialized);

        response_msg.set_payload(gzip_compress(waypoints_serialized));

        return response_msg;
    }

    void ConfigureNs3();
    void InitAgents();
    void AddWifiNetworkStack(Ptr<ns3::Node> node, uint32_t agent_id);
    Ptr<WaypointBroadcaster> AddWaypointBroadcaster(Ptr<ns3::Node> node);
    Ptr<WaypointReceiver> AddWaypointReceiver(Ptr<ns3::Node> node);
    void RandomWaypointGeneration(Ptr<WaypointBroadcaster> app, Time interval, Vector corner1, Vector corner2);
    void UpdateAgentsPosition(const dancers_update_proto::DancersUpdate& update_msg);

    // Practical functions
    std::optional<uint32_t> FindAgentIdByNode(const Ptr<ns3::Node>& node) const;

    // ROS2 Callbacks
    void SpawnUavClbk(dancers_msgs::msg::AgentStruct msg);

    // ns-3 Trace callbacks
    void waypoint_broadcaster_clbk(Ptr<const Packet> packet);
    void waypoint_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id);

    std::map<uint32_t, std::unique_ptr<Agent>> agents_;
    mutable std::shared_mutex agents_mutex_;

    std::map<uint32_t, Vector> current_waypoints_;

    NodeContainer nodes;

    std::unique_ptr<WifiHelper> wifiHelper_;
    std::unique_ptr<WifiMacHelper> wifiMacHelper_;
    std::unique_ptr<WifiPhyHelper> wifiPhyHelper_;
    std::unique_ptr<InternetStackHelper> internetHelper_;
    std::unique_ptr<Ipv4AddressHelper> ipv4AddressHelper_;

};

/**
 * @brief Configure ns-3 helpers and set the random seed
 * 
 * This function initializes the ns-3 helpers used to create the Wi-Fi ad-hoc network.
 */
void BasicWifiAdhoc::ConfigureNs3()
{
    // Set the seed for the random number generator
    SeedManager::SetRun(getYamlValue<int>(this->config_, "seed"));

    // Initialize an empty node container
    this->nodes = NodeContainer();

    // --- Wi-Fi ---
    std::string wifi_standard = getYamlValue<std::string>(this->config_, "wifi_standard");
    this->wifiHelper_ = std::make_unique<WifiHelper>();

    if (wifi_standard == "802.11g")
    {
        this->wifiHelper_->SetStandard(WIFI_STANDARD_80211g);
    }
    else if (wifi_standard == "802.11n")
    {
        this->wifiHelper_->SetStandard(WIFI_STANDARD_80211n);
    }
    else if (wifi_standard == "802.11ax")
    {
        this->wifiHelper_->SetStandard(WIFI_STANDARD_80211ax);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported WiFi standard: %s", wifi_standard.c_str());
        exit(EXIT_FAILURE);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Using WiFi standard: " << wifi_standard.c_str());

    std::string wifi_phy_mode = getYamlValue<std::string>(this->config_, "wifi_phy_mode");
    this->wifiHelper_->SetRemoteStationManager("ns3::ConstantRateWifiManager", 
        "DataMode", StringValue(wifi_phy_mode), 
        "ControlMode", StringValue(wifi_phy_mode));

    // Fix non-unicast WiFi data rate to be the same as that of unicast
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                       StringValue(wifi_phy_mode));

    // The "Ht" configuration is for 802.11n
    // Short guard interval = 400ns (vs 800ns otherwise)
    Config::SetDefault("ns3::HtConfiguration::ShortGuardIntervalSupported", BooleanValue(getYamlValue<bool>(this->config_, "short_guard_interval_supported")));

    // --- MAC layer ---
    this->wifiMacHelper_ = std::make_unique<WifiMacHelper>();
    this->wifiMacHelper_->SetType("ns3::AdhocWifiMac");

    // --- PHY layer ---
    this->wifiPhyHelper_ = std::make_unique<YansWifiPhyHelper>();
    YansWifiChannelHelper yansChannel = YansWifiChannelHelper::Default();

    if (auto YansWifiPtr = dynamic_cast<YansWifiPhyHelper*>(this->wifiPhyHelper_.get()))
    {
        YansWifiPtr->SetChannel(yansChannel.Create());
    }

    // --- IP / ROUTING ---
    this->internetHelper_ = std::make_unique<InternetStackHelper>();
}


/**
 * @brief Initialize the agents at the start of the simulation, by building an associated ROS2 "AgentStruct" message and calling the callback that spawns agents.
 */
void BasicWifiAdhoc::InitAgents()
{
    for (unsigned int i=0; i < getYamlValue<unsigned int>(this->config_, "robots_number"); i++)
    {
        dancers_msgs::msg::AgentStruct agent_ros_msg;

        // Agent's ID
        agent_ros_msg.agent_id = i;
        // Agent's initial position and velocity is arbitrarily defined for the first iteration. After that, it comes from the physics simulator
        agent_ros_msg.state.position.x = float(i);
        agent_ros_msg.state.position.y = 0.0;
        agent_ros_msg.state.position.z = 1.0;
        agent_ros_msg.state.velocity.x = 0.0;
        agent_ros_msg.state.velocity.y = 0.0;
        agent_ros_msg.state.velocity.z = 0.0;
        agent_ros_msg.state.heading = 0.0;

        this->SpawnUavClbk(agent_ros_msg);
    }
}

/**
 * @brief Callback to spawn a new UAV in the ns-3 simulation
 * 
 * @param msg The ROS2 message containing the agent's information
 * Note: We initialize new ns-3 nodes this way with the idea that agents can be added during simulation later, by publishing an AgentStruct ROS2 message on a topic.
 */
void BasicWifiAdhoc::SpawnUavClbk(dancers_msgs::msg::AgentStruct msg)
{
    // Abort if the agent already exists.
    // Otherwise, add it to agents_
    {
        std::shared_lock lock(this->agents_mutex_);
        if (this->agents_.find(msg.agent_id) != this->agents_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Agent %d already exists, skipping spawn.", msg.agent_id);
            return;
        }
    }
    
    // Create a new ns-3 node
    uint32_t ns3_index = this->nodes.GetN();
    Ptr<ns3::Node> new_node = CreateObject<ns3::Node>();
    this->nodes.Add(new_node);

    // Set the mobility model
    Ptr<MobilityModel> nodeMob;
    nodeMob = CreateObject<ConstantVelocityMobilityModel>();
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(msg.state.position.x, msg.state.position.y, msg.state.position.z));
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(msg.state.velocity.x, msg.state.velocity.y, msg.state.velocity.z));
    new_node->AggregateObject(nodeMob);

    // Aggregate the building module to the nodes, so that we can use BuildingsPropagationLossModels with them
    BuildingsHelper::Install(new_node);

    // Add the node to the network
    this->AddWifiNetworkStack(new_node, msg.agent_id);

    // Finally, create the Agent object and add it to the map
    {
        std::unique_lock lock(this->agents_mutex_);
        auto [it, success] = this->agents_.emplace(msg.agent_id, std::make_unique<Agent>(AgentFromRosMsg(msg)));
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert new agent %d in the map. Something went wrong, aborting.", msg.agent_id);
            exit(EXIT_FAILURE);
        }
        auto& [id, agent_ptr] = *it;

        agent_ptr->node = std::make_shared<Ptr<ns3::Node>>(new_node);
        agent_ptr->node_container_index = ns3_index;
        agent_ptr->cmd_velocity = {0.0, 0.0, 0.0};
        agent_ptr->cmd_heading = 0.0;
        agent_ptr->channel_busy_time = 0.0;
    }
}

/**
 * @brief Add the wifi network stack to a node
 *
 * @param node Pointer to the ns-3 node
 * @param agent_id ID of the agent
 * 
 * This function installs the internet and Wi-Fi layers on a node, and assigns it an IPv4 address based on the agent's ID (10.0.0.<agent_id+1>)
 */
void BasicWifiAdhoc::AddWifiNetworkStack(Ptr<ns3::Node> node, uint32_t agent_id)
{
    // Verify that all the helpers pointers are initialized
    if (!this->wifiPhyHelper_ || !this->wifiMacHelper_ || !this->wifiHelper_ || !this->internetHelper_)
    {
        RCLCPP_FATAL(this->get_logger(), "Wifi helpers are not initialized, cannot add node to wifi network");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Preparing the network stack of node %d...", agent_id);

    // Install the (already configured) wifi module on the node.
    NetDeviceContainer new_net_device = (*this->wifiHelper_).Install((*this->wifiPhyHelper_), (*this->wifiMacHelper_), node);

    // Install the (already configured) internet helper on the node
    this->internetHelper_->Install(node);

    // Assign the right IPv4 address (10.0.0.<agent_id+1>) to the node
    Ipv4InterfaceAddress nodeIpv4AddressInterface(Ipv4Address(("10.0.0." + std::to_string(agent_id + 1)).c_str()), Ipv4Mask("255.255.255.0"));
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    int32_t interfaceIndex = ipv4->AddInterface(new_net_device.Get(0)); // Get the first device
    if (!ipv4->AddAddress(interfaceIndex, nodeIpv4AddressInterface))
    {
        RCLCPP_FATAL(this->get_logger(), "Couldn't assign IPv4 address to node %d", agent_id);
    }
    ipv4->SetUp(interfaceIndex);

    RCLCPP_DEBUG(this->get_logger(), "Network stack of node %d ready", agent_id);
}

/**
 * @brief Add a WaypointBroadcaster application to a node, and connect its Tx trace source to the corresponding callback
 *
 * @param node Pointer to the ns-3 node
 * @return Ptr<WaypointBroadcaster> Pointer to the created WaypointBroadcaster application
 */
Ptr<WaypointBroadcaster> BasicWifiAdhoc::AddWaypointBroadcaster(Ptr<ns3::Node> node)
{
    // Verify "waypoint_flow" exists in the configuration file
    if (!this->config_["waypoint_flow"].IsDefined())
    {
        RCLCPP_FATAL(this->get_logger(), "Waypoint flow is not defined in the configuration file, aborting.");
        exit(EXIT_FAILURE);
    }

    // Configure the waypoint broadcaster application
    Ptr<WaypointBroadcaster> waypoint_broadcaster = CreateObject<WaypointBroadcaster>();
    waypoint_broadcaster->SetStartTime(Seconds(0.1));
    waypoint_broadcaster->SetStopTime(Seconds(this->simulation_length));
    waypoint_broadcaster->SetAttribute("AdditionalSize", UintegerValue(0));
    waypoint_broadcaster->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(this->config_["waypoint_flow"], "port")));
    waypoint_broadcaster->SetAttribute("FlowId", UintegerValue(getYamlValue<uint8_t>(this->config_["waypoint_flow"], "flow_id")));
    Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
    rand->SetAttribute("Constant", DoubleValue(getYamlValue<unsigned int>(this->config_["waypoint_flow"], "interval") / 1000000.0f)); // from us to s (because custom class WaypointBroadcaster uses Interval as Seconds)
    waypoint_broadcaster->SetAttribute("Interval", PointerValue(rand));

    // Add it to the node
    node->AddApplication(waypoint_broadcaster);

    // Connect trace callback
    if (waypoint_broadcaster->TraceConnectWithoutContext("Tx", MakeCallback(&BasicWifiAdhoc::waypoint_broadcaster_clbk, this)))
    {
        RCLCPP_DEBUG(this->get_logger(), "Connected trace to waypoint broadcaster.");
    }

    return waypoint_broadcaster;
}

/**
 * @brief Add a WaypointReceiver application to a node, and connect its Rx trace source to the corresponding callback
 * 
 * @param node Pointer to the ns-3 node
 * @return Ptr<WaypointReceiver> Pointer to the created WaypointReceiver application
 */
Ptr<WaypointReceiver> BasicWifiAdhoc::AddWaypointReceiver(Ptr<ns3::Node> node)
{
    // Verify "waypoint_flow" exists in the configuration file
    if (!this->config_["waypoint_flow"].IsDefined())
    {
        RCLCPP_FATAL(this->get_logger(), "Waypoint flow is not defined in the configuration file, aborting.");
        exit(EXIT_FAILURE);
    }

    if (auto agent_id = this->FindAgentIdByNode(node))
    {
        // Configure the waypoint receiver application
        Ptr<WaypointReceiver> waypoint_receiver = CreateObject<WaypointReceiver>();
        waypoint_receiver->SetStartTime(Simulator::Now());
        waypoint_receiver->SetStopTime(Seconds(this->simulation_length));
        waypoint_receiver->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(this->config_["waypoint_flow"], "port")));
        
        // Add it to the node
        node->AddApplication(waypoint_receiver);
    
        // Connect trace callback
        if (waypoint_receiver->TraceConnect("Rx", std::to_string(agent_id.value()), MakeCallback(&BasicWifiAdhoc::waypoint_receiver_clbk, this)))
        {
            RCLCPP_DEBUG(this->get_logger(), "Connected trace to waypoint receiver on Agent %d.", agent_id.value());
        }

        return waypoint_receiver;
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(), "Could not find the agent associated to the node, aborting.");
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Callback for the reception of a packet at the application layer
 * 
 * @param context The context string provided when connecting the trace source (here, the agent ID)
 * @param packet Pointer to the packet being received
 * @param peer_id The ID of the peer that sent the packet
 */
void BasicWifiAdhoc::waypoint_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    uint32_t nodeId = std::stoi(context);

    WaypointHeader header;
    packet->PeekHeader(header);

    Vector waypoint = header.GetPosition();
    
    this->current_waypoints_[nodeId] = waypoint;
    
    RCLCPP_INFO(this->get_logger(), "Received a new waypoint from Agent %d: (%f, %f, %f)", peer_id, waypoint.x, waypoint.y, waypoint.z);
}

/**
 * @brief Callback for the transmission of a packet at the application layer
 * 
 * @param packet Pointer to the packet being transmitted
 */
void BasicWifiAdhoc::waypoint_broadcaster_clbk(Ptr<const Packet> packet)
{
    WaypointHeader header;
    packet->PeekHeader(header);

    Vector waypoint = header.GetPosition();
    
    this->current_waypoints_[0] = waypoint;

    RCLCPP_INFO(this->get_logger(), "Flocking broadcast packet sent");
}

/**
 * @brief Small utility function that finds the ID of an agent from a pointer to a node.
 * 
 * @param node Pointer to the ns-3 node
 * @return std::optional<uint32_t> The ID of the agent if found, std::nullopt otherwise
 * 
 * This function is based on the assumption that each agent has been associated with its permanent ns-3 node index, using the node_container_index value.
 */
std::optional<uint32_t> BasicWifiAdhoc::FindAgentIdByNode(const Ptr<ns3::Node>& node) const
{
    std::shared_lock lock(this->agents_mutex_);
    for (const auto& [id, agent] : this->agents_) {
        if (agent->node_container_index == node->GetId()) {
            return id;
        }
    }
    return std::nullopt;  // not found
}

/**
 * @brief Generate a random waypoint within the defined rectangle and set it in the WaypointBroadcaster application, reschedules itself every interval.
 * 
 * @param app Pointer to the WaypointBroadcaster application
 * @param interval Time interval between two waypoint generations
 * @param corner1 One corner of the rectangle defining the area in which waypoints are generated
 * @param corner2 The opposite corner of the rectangle defining the area in which waypoints are
 */
void BasicWifiAdhoc::RandomWaypointGeneration(Ptr<WaypointBroadcaster> app, Time interval, Vector corner1, Vector corner2)
{
    // Get the min and max coordinates of the rectangle
    double minX = std::min(corner1.x, corner2.x);
    double maxX = std::max(corner1.x, corner2.x);
    double minY = std::min(corner1.y, corner2.y);
    double maxY = std::max(corner1.y, corner2.y);
    double minZ = std::min(corner1.z, corner2.z);
    double maxZ = std::max(corner1.z, corner2.z);

    // Generate a random point within the rectangle
    double x = minX + (maxX - minX) * (double)rand() / RAND_MAX;
    double y = minY + (maxY - minY) * (double)rand() / RAND_MAX;
    double z = minZ + (maxZ - minZ) * (double)rand() / RAND_MAX;
    Vector waypoint(x, y, z);

    app->SetWaypoint(waypoint);
    RCLCPP_INFO(this->get_logger(), "New waypoint generated: (%f, %f, %f)", x, y, z);

    // Schedule the next waypoint generation
    Simulator::Schedule(interval, &BasicWifiAdhoc::RandomWaypointGeneration, this, app, interval, corner1, corner2);
}

/**
 * @brief Update the agents positions in ns-3 according to the protobuf message received from the coordinator
 * 
 * @param update_msg Protobuf message received from the coordinator
 */
void BasicWifiAdhoc::UpdateAgentsPosition(const dancers_update_proto::DancersUpdate& update_msg)
{
    if (!update_msg.payload().empty())
    {
        dancers_update_proto::PoseVector robots_positions;
        if (!robots_positions.ParseFromString(gzip_decompress(update_msg.payload())))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse robots positions from protobuf message.");
            exit(EXIT_FAILURE);
        }

        for (const auto& position_msg : robots_positions.pose())
        {
            std::unique_lock lock(this->agents_mutex_);
            auto agent = this->agents_.find(position_msg.agent_id());
            if (agent != this->agents_.end())
            {
                // Update the position in the agent structure
                agent->second->position = Eigen::Vector3d(position_msg.x(), position_msg.y(), position_msg.z());
                // Save the initial position of the agents at the first iteration (the first time )
                if (!agent->second->initial_position_saved)
                {
                    agent->second->initial_position = agent->second->position;
                    agent->second->initial_position_saved = true;
                }
                // Update the position of the node in ns-3
                Ptr<ConstantVelocityMobilityModel> mobility = this->nodes.Get(agent->second->node_container_index)->GetObject<ConstantVelocityMobilityModel>();
                mobility->SetPosition(Vector(position_msg.x(), position_msg.y(), position_msg.z()));
                mobility->SetVelocity(Vector(position_msg.vx(), position_msg.vy(), position_msg.vz()));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Received position for unknown agent %d, skipping.", position_msg.agent_id());
            }
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Received empty payload for dancers update, skipping.");
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicWifiAdhoc>());
    rclcpp::shutdown();
    return 0;
}
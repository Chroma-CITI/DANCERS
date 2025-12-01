#include <connector_core/connector.hpp>

// ns-3 libs
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/buildings-module.h>

// Home-made ns-3 apps
#include <applications/flocking-application.h>

// Common DANCERS libs
#include <agent.hpp>
#include <yaml_util.hpp>

// C++ libs
#include <shared_mutex>

// Custom ROS2 messages
#include <dancers_msgs/msg/agent_struct.hpp>
#include <dancers_msgs/msg/agent_struct_array.hpp>
#include <dancers_msgs/srv/get_agent_velocities.hpp>

// Protobuf messages
#include <protobuf_msgs/pose_vector.pb.h>

// Rviz ROS2 messages
#include <visualization_msgs/msg/marker_array.hpp>

using namespace ns3;
using namespace std::chrono_literals;

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

        // Initiate publishers
        if (getYamlValue<bool>(this->config_, "display_network_edges"))
        {
            this->neighborhood_links_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("neighborhood_links", 10);
        }
    

        /* ----------- Service client ----------- */
        command_client_ = this->create_client<dancers_msgs::srv::GetAgentVelocities>("get_agents_velocities");

        // Create timer for service calls ()
        this->req_cmds_timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::microseconds(getYamlValue<uint32_t>(this->config_, "request_commands_period")),
            std::bind(&BasicWifiAdhoc::RequestCommandsClbk, this));

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

        // this->RequestCommandsClbk();

        // Once all the events are scheduled, advance W time in the simulation and stop
        Simulator::Stop(Seconds(step_size));

        Simulator::Run();

        this->timeoutNeighbors();

        this->DisplayRviz();

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

        // In this file, we send velocity / yaw commands to the physics simulator at each step
        dancers_update_proto::PoseVector velocity_hdg_cmds;
        std::shared_lock lock(this->agents_mutex_);
        for (const auto& [agent_id, agent_ptr] : this->agents_)
        {
            dancers_update_proto::Pose* velocity_hdg_cmd = velocity_hdg_cmds.add_pose();
            velocity_hdg_cmd->set_vx(agent_ptr->cmd_velocity.x());
            velocity_hdg_cmd->set_vy(agent_ptr->cmd_velocity.y());
            velocity_hdg_cmd->set_vz(agent_ptr->cmd_velocity.z());
            velocity_hdg_cmd->set_agent_id(agent_id);
        }
        std::string commands_serialized;
        velocity_hdg_cmds.SerializeToString(&commands_serialized);

        response_msg.set_payload(gzip_compress(commands_serialized));

        return response_msg;
    }

    void ConfigureNs3();
    void InitAgents();
    void AddWifiNetworkStack(Ptr<ns3::Node> node, uint32_t agent_id);
    void AddFlockingApplication(Ptr<ns3::Node> node);
    void UpdateAgentsPosition(const dancers_update_proto::DancersUpdate& update_msg);
    void DisplayRviz();

    rclcpp::TimerBase::SharedPtr req_cmds_timer_;    //!< timer for the request command loop
    void RequestCommandsClbk();
    void timeoutNeighbors();

    // Practical functions
    std::optional<uint32_t> FindAgentIdByNode(const Ptr<ns3::Node>& node) const;

    // ROS2 Callbacks
    void SpawnUavClbk(dancers_msgs::msg::AgentStruct msg);

    // ROS2 Service (client)
    rclcpp::Client<dancers_msgs::srv::GetAgentVelocities>::SharedPtr command_client_;

    // ns-3 Trace callbacks
    void flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet);
    void flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id);

    // ROS2 Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr neighborhood_links_pub_;

    std::map<uint32_t, std::unique_ptr<Agent>> agents_;
    mutable std::shared_mutex agents_mutex_;

    std::map<uint32_t, Vector> current_waypoints_;

    NodeContainer nodes;

    std::unique_ptr<WifiHelper> wifiHelper_;
    std::unique_ptr<WifiMacHelper> wifiMacHelper_;
    std::unique_ptr<WifiPhyHelper> wifiPhyHelper_;
    std::unique_ptr<InternetStackHelper> internetHelper_;
    std::unique_ptr<Ipv4AddressHelper> ipv4AddressHelper_;
    Ptr<UniformRandomVariable> random_gen_start_flocking_app;


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

    // --- APP layer ---
    // If all the flocking applications of the agents start at exactly the same time, the packet loss is enormous because all packets always collide
    this->random_gen_start_flocking_app = CreateObject<UniformRandomVariable>();
    this->random_gen_start_flocking_app->SetAttribute("Min", DoubleValue(0.0));
    this->random_gen_start_flocking_app->SetAttribute("Max", DoubleValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "interval") / 1000000.0f));

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

    
    // Create the Agent object and add it to the map
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

    // Add flocking application to the node (the corresponding "Agent" object must exist in the agents_ map)
    this->AddFlockingApplication(new_node);
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
void BasicWifiAdhoc::AddFlockingApplication(Ptr<ns3::Node> node)
{
    // Verify "flocking_flow" exists in the configuration file
    if (!this->config_["flocking_flow"].IsDefined())
    {
        RCLCPP_FATAL(this->get_logger(), "Flocking flow is not defined in the configuration file, aborting.");
        exit(EXIT_FAILURE);
    }

    if (auto agent_id = this->FindAgentIdByNode(node))
    {
        // Configure the flocking application broadcaster
        Ptr<FlockingBroadcaster> flocking_broadcaster = CreateObject<FlockingBroadcaster>();
        flocking_broadcaster->SetStartTime(Simulator::Now() + Seconds(this->random_gen_start_flocking_app->GetValue()));
        flocking_broadcaster->SetAttribute("PacketSize", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "packet_size")));
        flocking_broadcaster->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "port")));
        Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
        rand->SetAttribute("Constant", DoubleValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "interval") / 1000000.0f)); // from us to s (because custom class Sender uses Interval as Seconds)
        flocking_broadcaster->SetAttribute("Interval", PointerValue(rand));
        flocking_broadcaster->SetAttribute("FlowId", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "flow_id")));

        // Configure the flocking application receiver
        Ptr<FlockingReceiver> flocking_receiver = CreateObject<FlockingReceiver>();
        flocking_receiver->SetStartTime(Seconds(0.0)); // Always have the receiver activated.
        flocking_receiver->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "port")));
        
        node->AddApplication(flocking_broadcaster);
        
        // App 1 is the Flocking receiver, it receives the heartbeat message and updates the neighbors
        node->AddApplication(flocking_receiver);
        
        if (flocking_broadcaster->TraceConnect("Tx", std::to_string(agent_id.value()), MakeCallback(&BasicWifiAdhoc::flocking_broadcaster_clbk, this)))
        {
            RCLCPP_DEBUG(this->get_logger(), "Connected broadcaster to node %d", agent_id.value());
        }
        if (flocking_receiver->TraceConnect("Rx", std::to_string(agent_id.value()), MakeCallback(&BasicWifiAdhoc::flocking_receiver_clbk, this)))
        {
            RCLCPP_DEBUG(this->get_logger(), "Connected receiver to node %d", agent_id.value());
        }
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(), "Could not find the agent associated to the node %d (ns-3 index) when trying to install an application on it, aborting.", node->GetId());
        exit(EXIT_FAILURE);
    }
}


/**
 * @brief Callback for the reception of a flocking packet at the application layer
 */
void BasicWifiAdhoc::flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    uint32_t agent_id = std::stoi(context);

    RCLCPP_DEBUG(this->get_logger(), "[%d] Flocking heartbeat received from %d.", agent_id, peer_id);

    // Read Header and update corresponding neighbor entry (the "NeighborInfo" represents the "freshest" information about a neighbor)
    FlockingHeader fl_hdr;
    packet->PeekHeader(fl_hdr);

    Vector neighbor_position = fl_hdr.GetPosition();
    Vector neighbor_velocity = fl_hdr.GetVelocity();

    // It might be the first time we see this agent, add it to the map
    std::unique_lock lock(this->agents_mutex_);
    if (this->agents_.at(agent_id)->neighbors.find(peer_id) == this->agents_.at(agent_id)->neighbors.end())
    {
        this->agents_.at(agent_id)->neighbors[peer_id] = NeighborInfo();
    }

    this->agents_.at(agent_id)->neighbors.at(peer_id).position = Eigen::Vector3d(neighbor_position.x, neighbor_position.y, neighbor_position.z);
    this->agents_.at(agent_id)->neighbors.at(peer_id).velocity = Eigen::Vector3d(neighbor_velocity.x, neighbor_velocity.y, neighbor_velocity.z);
    this->agents_.at(agent_id)->neighbors.at(peer_id).id = peer_id;
    this->agents_.at(agent_id)->neighbors.at(peer_id).last_seen = Simulator::Now().ToInteger(Time::US);
    // this->agents_.at(agent_id)->neighbors.at(peer_id).link_quality = rxPow;
    this->agents_.at(agent_id)->neighbors.at(peer_id).link_type = LinkType::FlockingLink;

    this->agents_.at(agent_id)->heartbeat_received++;

}

/**
 * @brief Callback for the transmission of a packet at the application layer
 */
void BasicWifiAdhoc::flocking_broadcaster_clbk(std::string context, Ptr<const Packet>)
{
    Time txTime = Simulator::Now();

    RCLCPP_DEBUG(this->get_logger(), "[%d] Flocking broadcast packet sent", std::stoi(context));
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

/**
 * @brief Request the robot controller to send the new command for all the robots
 *
 * @param request_msg Protobuf message to be sent to the coordinator
 * 
 * IMPORTANT NOTE: It is important to call this callback "the ROS2 way", i.e. that it be executed from within a ROS2 "executor" and not from a different thread. Otherwise, you risk concurrent access to the internal node (rclcpp::Node) such as clock, logging, parameters, etc., but also race conditions or deadlocks. The executor thread and the "Loop" thread should always exchange information through shared memory in a safe manner, i.e. using locks and mutexes.
 *  
 * This method is actually centralized (the request contains the complete state of the fleet and the response contains the commands for all the agents). This was done this way to minimize the number of ROS2 messages exchanged internally, which improves efficiency. As long as the controller implements a distributed control algorithm, the system stays distributed.
 */
void BasicWifiAdhoc::RequestCommandsClbk()
{
    /*************** Get commands ***************/
    // Wait for existence of service: 1s.
    while (!command_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_WARN_STREAM(this->get_logger(), "Service " << command_client_->get_service_name() << " not available, waiting again...");
    }

    // Create the request message
    auto request = std::make_shared<dancers_msgs::srv::GetAgentVelocities::Request>();
    
    // Fill the request message with the last known agents positions and velocities, and their neighboring information
    {    
        std::shared_lock lock(this->agents_mutex_);
        for (const auto& [agent_id, agent_ptr] : this->agents_)
        {
            if (!agent_ptr->crashed)
            {
                dancers_msgs::msg::AgentStruct agent_struct = RosMsgFromAgent(*agent_ptr);
                request->agent_structs.push_back(agent_struct);
            }
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Requesting commands for %zu agents", request->agent_structs.size());

    // Send the request and handle the response
    auto result_future = command_client_->async_send_request(request,
        [this](rclcpp::Client<dancers_msgs::srv::GetAgentVelocities>::SharedFuture future) {
            
            // Get the shared_ptr to the response (keeps ownership while we call get()).
            std::shared_ptr<dancers_msgs::srv::GetAgentVelocities::Response> response_msg = future.get();
            
            RCLCPP_DEBUG(this->get_logger(), "Service answered with %zu commands !", response_msg->velocity_headings.velocity_heading_array.size());
    
            {
                std::unique_lock lock(this->agents_mutex_);
                for (const auto& velocity_heading : response_msg->velocity_headings.velocity_heading_array)
                {
                    auto agent_it = this->agents_.find(velocity_heading.agent_id);
                    if (agent_it != this->agents_.end())
                    {
                        RCLCPP_DEBUG(this->get_logger(), "Received command for agent %d: velocity=(%.2f, %.2f, %.2f), heading=%.2f", 
                            velocity_heading.agent_id,
                            velocity_heading.velocity.x,
                            velocity_heading.velocity.y,
                            velocity_heading.velocity.z,
                            velocity_heading.heading);
                        agent_it->second->cmd_velocity = Eigen::Vector3d(velocity_heading.velocity.x, velocity_heading.velocity.y, velocity_heading.velocity.z);
                        agent_it->second->cmd_heading = velocity_heading.heading;
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Received command for unknown agent %d, skipping.", velocity_heading.agent_id);
                    }
                }
            }
        }
    );

}

/**
 * @brief Clear timed out neighbors
 *
 * Removes neighbors that have not been heard for a while.
 */
void BasicWifiAdhoc::timeoutNeighbors()
{
    std::unique_lock lock(this->agents_mutex_);
    for (auto &[agent_id, agent_ptr] : this->agents_)
    {
        std::vector<uint32_t> NeighIDsToRemove;
        for (auto const &neighbor : agent_ptr->neighbors)
        {
            if ((Simulator::Now() - MicroSeconds(neighbor.second.last_seen)) > MicroSeconds(getYamlValue<uint32_t>(this->config_["flocking_flow"], "timeout")))
            {
                NeighIDsToRemove.push_back(neighbor.first);
                RCLCPP_INFO(this->get_logger(), "Agent %d not a neighbor of %d anymore", neighbor.first, agent_id);
            }
        }
        for (auto const &neighbor : NeighIDsToRemove)
        {
            agent_ptr->neighbors.erase(neighbor);
        }
    }
}

void BasicWifiAdhoc::DisplayRviz()
{
    // Display edges between neighbors
    if (getYamlValue<bool>(this->config_, "display_network_edges"))
    {
        visualization_msgs::msg::Marker network_marker{};
        network_marker.header.frame_id = "map";
        network_marker.id = 1011;
        network_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        network_marker.action = visualization_msgs::msg::Marker::MODIFY;
        network_marker.scale.x = 0.1;
        network_marker.color.r = 0.0;
        network_marker.color.g = 0.0;
        network_marker.color.b = 1.0; // Blue color
        network_marker.color.a = 1.0; // Fully opaque
        // network_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        for (auto& [agent_id, agent_ptr] : this->agents_)
        {
            Eigen::Vector3d agent_pose(agent_ptr->position);
            for (auto& [neighbor_id, neighbor_struct] : agent_ptr->neighbors)
            {
                if (agent_id == neighbor_id)
                    continue; // (should never happen as we check this in GetNeighbors)
                // Note that we take the "real" position of the neighbor and not the position as it is known by the current agent (would be neighbor_struct.position), 
                // otherwise, the visualization is weird
                Eigen::Vector3d other_agent_pose(this->agents_.at(neighbor_id)->position);

                geometry_msgs::msg::Point p1{};
                p1.x = agent_pose.x();
                p1.y = agent_pose.y();
                p1.z = agent_pose.z();
                geometry_msgs::msg::Point p2{};
                p2.x = other_agent_pose.x();
                p2.y = other_agent_pose.y();
                p2.z = other_agent_pose.z();

                network_marker.points.push_back(p1);
                network_marker.points.push_back(p2);
            }
        }

        this->neighborhood_links_pub_->publish(network_marker);
    }

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicWifiAdhoc>());
    rclcpp::shutdown();
    return 0;
}
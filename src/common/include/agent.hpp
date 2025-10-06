#pragma once

#include <Eigen/Dense>
#include <dancers_msgs/msg/agent_struct.hpp>

/**
 * @brief Possible roles that an agent can have. This role influence their flocking behaviors.
 */
enum AgentRoleType
{
    Undefined = 0,
    Mission,
    Potential,
    Idle
};

/**
 * @brief Possible link types between agents. This influence their flocking behaviors.
 */
enum LinkType
{
    DataLink = 0,
    FlockingLink
};

/**
 * @brief Structure containing the information about the neighbors.
 */
struct NeighborInfo
{
    int id;
    double link_quality;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    AgentRoleType role;
    LinkType link_type;
    uint32_t last_seen; //us
};

// Forward declaration only
// We use opaque pointers in the Agent struct to avoid having to include the "real" classes, which would create cross-dependency (ns-3 having to include UavSystem for example)
namespace ns3 {
    template <typename T> 
    class Ptr;
    class Node;
}
namespace mrs_multirotor_simulator { class UavSystem; }


struct Agent
{
    uint32_t id;
    AgentRoleType role;

    // Physics related info
    std::shared_ptr<mrs_multirotor_simulator::UavSystem> uav_system;    // Opaque pointer to UavSystem (only used in the physics simulation)
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d initial_position;
    bool initial_position_saved = false;
    double heading;
    bool crashed = false;
    Eigen::Vector3d cmd_velocity;
    double cmd_heading;

    // Network related info
    std::shared_ptr<ns3::Ptr<ns3::Node>> node;      // Opaque pointer to ns3::Node (only used in the network simulation)
    uint32_t heartbeat_received;
    uint32_t heartbeat_sent;
    std::map<uint32_t, NeighborInfo> neighbors;
    uint32_t node_container_index;
    double channel_busy_time;
};

Agent AgentFromRosMsg(dancers_msgs::msg::AgentStruct agent)
{
    Agent a;
    a.id = agent.agent_id;
    a.role = static_cast<AgentRoleType>(agent.agent_role);
    a.position = Eigen::Vector3d(agent.state.position.x, agent.state.position.y, agent.state.position.z);
    a.velocity = Eigen::Vector3d(agent.state.velocity.x, agent.state.velocity.y, agent.state.velocity.z);
    a.heading = agent.state.heading;
    a.heartbeat_received = agent.heartbeat_received;
    a.heartbeat_sent = agent.heartbeat_sent;
    a.neighbors = std::map<uint32_t, NeighborInfo>();

    for (const auto &neighbor : agent.neighbor_array.neighbors)
    {
        NeighborInfo n;
        n.id = neighbor.agent_id;
        n.link_quality = neighbor.link_quality;
        n.position = Eigen::Vector3d(neighbor.position.x, neighbor.position.y, neighbor.position.z);
        n.velocity = Eigen::Vector3d(neighbor.velocity.x, neighbor.velocity.y, neighbor.velocity.z);
        n.role = static_cast<AgentRoleType>(neighbor.agent_role);

        a.neighbors[neighbor.agent_id] = n;
    }

    return a;
}

dancers_msgs::msg::AgentStruct RosMsgFromAgent(Agent agent)
{
    dancers_msgs::msg::AgentStruct msg;
    msg.agent_id = agent.id;
    msg.agent_role = static_cast<uint8_t>(agent.role);
    msg.state.position.x = agent.position.x();
    msg.state.position.y = agent.position.y();
    msg.state.position.z = agent.position.z();
    msg.state.velocity.x = agent.velocity.x();
    msg.state.velocity.y = agent.velocity.y();
    msg.state.velocity.z = agent.velocity.z();
    msg.state.heading = agent.heading;
    msg.heartbeat_received = agent.heartbeat_received;
    msg.heartbeat_sent = agent.heartbeat_sent;
    msg.neighbor_array.neighbors.clear();

    for (const auto &pair : agent.neighbors)
    {
        const NeighborInfo &n = pair.second;
        dancers_msgs::msg::Neighbor neighbor_msg;
        neighbor_msg.agent_id = n.id;
        neighbor_msg.link_quality = n.link_quality;
        neighbor_msg.position.x = n.position.x();
        neighbor_msg.position.y = n.position.y();
        neighbor_msg.position.z = n.position.z();
        neighbor_msg.velocity.x = n.velocity.x();
        neighbor_msg.velocity.y = n.velocity.y();
        neighbor_msg.velocity.z = n.velocity.z();
        neighbor_msg.agent_role = static_cast<uint8_t>(n.role);

        msg.neighbor_array.neighbors.push_back(neighbor_msg);
    }

    return msg;
}
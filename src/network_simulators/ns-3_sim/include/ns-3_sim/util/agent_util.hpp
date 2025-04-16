#pragma once

#include <Eigen/Dense>
#include <vector>

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
struct NeighborInfo_t
{
    int id;
    double link_quality;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    AgentRoleType role;
    LinkType link_type;
    uint32_t last_seen; //us
};

/**
 * @struct The struct holding information for a connected UAV agent
 * 
 * It holds the UavSystem object, its id, the list(s) of its neighbors and its objectives (targets)
 */
struct agent_t
{
    int id;
    AgentRoleType role;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d cmd_velocity;
    double cmd_heading;
    std::map<int, NeighborInfo_t> neighbors;
};
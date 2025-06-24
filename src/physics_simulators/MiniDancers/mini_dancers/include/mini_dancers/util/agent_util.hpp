#pragma once

#include "../uav_system.hpp"
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
 * @brief Structure containing the information about the neighbors.
 */
struct NeighborInfo_t
{
    int id;
    double link_quality;
    AgentRoleType role;
};

/**
 * @struct The struct holding information for a connected UAV agent
 * 
 * It holds the UavSystem object, its id, the list(s) of its neighbors and its objectives (targets)
 */
struct agent_t
{
    mrs_multirotor_simulator::UavSystem uav_system;
    int id;
    AgentRoleType role;
    std::vector<NeighborInfo_t> neighbors;
    // std::vector<int> neighbors_mission;
    // std::vector<int> neighbors_potential;
    // std::vector<int> neighbors_routing;
    // std::vector<double> link_qualities;
    std::optional<Eigen::Vector3d> secondary_objective;
    uint16_t obstacles_collisions = 0;
    uint16_t uavs_collisions = 0;
};
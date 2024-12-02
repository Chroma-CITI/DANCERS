#pragma once

#include "../uav_system.hpp"
#include <vector>

/**
 * @struct The struct holding information for a connected UAV agent
 * 
 * It holds the UavSystem object, its id, the list(s) of its neighbors and its objectives (targets)
 */
struct agent_t
{
    mrs_multirotor_simulator::UavSystem uav_system;
    int id;
    std::vector<int> neighbors;
    std::vector<int> neighbors_mission;
    std::vector<int> neighbors_potential;
    std::vector<int> neighbors_routing;
    std::vector<double> link_qualities;
    std::optional<Eigen::Vector3d> secondary_objective;
};
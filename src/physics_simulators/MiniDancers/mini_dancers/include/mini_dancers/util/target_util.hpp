#pragma once

#include <Eigen/Core>
/**
 * @brief Structure containing the information for a target area
 */
struct target_t
{
    Eigen::Vector3d position;
    bool is_sink;
    std::vector<int> assigned_agents;
};
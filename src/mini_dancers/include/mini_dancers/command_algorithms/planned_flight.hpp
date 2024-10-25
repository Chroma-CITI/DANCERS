#include <Eigen/Core>
#include <uav_system.hpp>
#include <util.hpp>

struct circle_params_t
{
    double radius;
    double center_x;
    double center_y;
    double omega;
    double target_altitude;
};

using namespace mrs_multirotor_simulator;

std::vector<reference::VelocityHdg> ComputeDesiredVelocities(std::vector<agent_t *> uavs, double timestamp, circle_params_t params)
{
    std::vector<reference::VelocityHdg> controllers;
    for (int i=0; i < uavs.size(); i++)
    {
        reference::VelocityHdg controller;
        controller.velocity = Eigen::Vector3d::Zero();
        
        // Circular motion
        controller.velocity[0] = params.radius * cos(params.omega * timestamp) - params.center_x;
        controller.velocity[1] = params.radius * sin(params.omega * timestamp) - params.center_y;

        // Dumb proportional altitude controller (z up)
        Eigen::Vector3d agent_position = uavs[i]->uav_system.getState().x;
        if (agent_position[2] > params.target_altitude + 0.5)
        {
            controller.velocity[2] = 0.1 * (params.target_altitude - agent_position[2]);
        }
        else if (agent_position[2] < params.target_altitude - 0.5)
        {
            controller.velocity[2] = 0.1 * (params.target_altitude - agent_position[2]);
        }
        else
        {
            controller.velocity[2] = 0.0;
        }

        controller.heading = 0.0;

        controllers.push_back(controller);

    }
    return controllers;
}
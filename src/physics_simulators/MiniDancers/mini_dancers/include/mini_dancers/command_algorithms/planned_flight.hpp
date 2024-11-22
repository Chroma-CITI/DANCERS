#include <Eigen/Core>
#include <uav_system.hpp>
#include <util.hpp>

struct circle_params_t
{
    double gain = 10.0;
    double omega = 0.1;
};

using namespace mrs_multirotor_simulator;

std::vector<reference::VelocityHdg> ComputeCircleVelocities(std::vector<agent_t *> uavs, double timestamp, circle_params_t params)
{
    std::vector<reference::VelocityHdg> controllers;
    for (int i=0; i < uavs.size(); i++)
    {
        reference::VelocityHdg controller;
        controller.velocity = Eigen::Vector3d::Zero();
        
        // Circular motion
        controller.velocity[0] = params.gain * cos(params.omega * timestamp);
        controller.velocity[1] = params.gain * sin(params.omega * timestamp);

        // Dumb proportional altitude controller (z up)
        controller.heading = 0.0;

        controllers.push_back(controller);

    }
    return controllers;
}
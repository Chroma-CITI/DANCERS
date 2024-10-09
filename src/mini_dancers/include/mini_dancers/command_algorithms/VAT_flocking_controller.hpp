#include <Eigen/Core>
#include <uav_system.hpp>
#include <util.hpp>

struct VAT_params_t
{
    float v_flock = 1.5;
    float v_max = 1;
    float a_frict = 4.16;
    float p_frict = 3.2;
    float r_0_frict = 85.3;
    float C_frict = 0.8;
    float v_frict = 0.63;
    float p_att = 0.08;
    float r_0_att = 15;
    float p_rep = 0.13;
    float r_0_rep = 15;
    float a_shill = 53;
    float p_shill = 3.55;
    float r_0_shill = 0.3;
    float v_shill = 13.622;
};

using namespace mrs_multirotor_simulator;

Eigen::Vector3d alignment_term(std::vector<agent_t> uavs, int which_agent, VAT_params_t params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    MultirotorModel::State agent_state = uavs[which_agent].uav_system.getState();
    for (int neighbor_id : uavs[which_agent].neighbors)
    {
        if (neighbor_id != which_agent)
        {
            MultirotorModel::State neighbor_state = uavs[neighbor_id].uav_system.getState();
            double distance = (agent_state.x - neighbor_state.x).norm();
            double velDiffNorm = (neighbor_state.v - agent_state.v).norm();

            double v_frictmax = std::max((double)params.v_frict, SigmoidLin(distance - params.r_0_frict, params.a_frict, params.p_frict));
            if (velDiffNorm > v_frictmax)
            {
                result += params.C_frict * (velDiffNorm - v_frictmax) * (neighbor_state.v - agent_state.v) / velDiffNorm;
            }
        }
    }
    return result;
}

Eigen::Vector3d attraction_term(std::vector<agent_t> uavs, int which_agent, VAT_params_t params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    MultirotorModel::State agent_state = uavs[which_agent].uav_system.getState();
    for (int neighbor_id : uavs[which_agent].neighbors)
    {
        if (neighbor_id != which_agent)
        {
            MultirotorModel::State neighbor_state = uavs[neighbor_id].uav_system.getState();
            double distance = (agent_state.x - neighbor_state.x).norm();
            Eigen::Vector3d relative_position = agent_state.x - neighbor_state.x;
            if (distance > params.r_0_att)
            {
                result += params.p_att * (params.r_0_att - distance) * (relative_position) / distance;
            }
        }
    }
    return result;   
}

Eigen::Vector3d repulsion_term(std::vector<agent_t> uavs, int which_agent, VAT_params_t params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    MultirotorModel::State agent_state = uavs[which_agent].uav_system.getState();
    for (int neighbor_id : uavs[which_agent].neighbors)
    {
        if (neighbor_id != which_agent)
        {
            MultirotorModel::State neighbor_state = uavs[neighbor_id].uav_system.getState();
            double distance = (agent_state.x - neighbor_state.x).norm();
            Eigen::Vector3d relative_position = agent_state.x - neighbor_state.x;
            if (distance < params.r_0_rep && distance > 0.0)
            {
                result += params.p_rep * (params.r_0_rep - distance) * (relative_position) / distance;
            }
        }
    }
    return result;   
}

Eigen::Vector3d shill_term(std::vector<agent_t> uavs, int which_agent, VAT_params_t params, std::vector<obstacle_t> obstacles)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    MultirotorModel::State agent_state = uavs[which_agent].uav_system.getState();
    for (obstacle_t obstacle : obstacles)
    {
        Eigen::Vector3d shill_agent_position = GetNearestPointFromObstacle(agent_state.x, obstacle);
        Eigen::Vector3d shill_agent_velocity = params.v_shill * -(shill_agent_position - agent_state.x).normalized();
        double velDiffNorm = (shill_agent_velocity - agent_state.v).norm();
        double v_shillmax = SigmoidLin((shill_agent_position - agent_state.x).norm() - params.r_0_shill, params.a_shill, params.p_shill);
        if (velDiffNorm > v_shillmax)
        {
            result += -(velDiffNorm - v_shillmax) * (agent_state.v - shill_agent_velocity) / velDiffNorm;
        }
    }

    return result;   
}

Eigen::Vector3d secondary_objective(std::vector<agent_t> uavs, int which_agent, VAT_params_t params, Eigen::Vector3d goal)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    MultirotorModel::State agent_state = uavs[which_agent].uav_system.getState();
    
    Eigen::Vector3d relative_position = agent_state.x - goal;

    result += params.v_flock * -(relative_position);

    return result;
}

std::vector<reference::VelocityHdg> ComputeVATFlockingDesiredVelocities(const std::vector<agent_t> uavs, const std::vector<obstacle_t> obstacles, VAT_params_t flocking_params)
{
    std::vector<reference::VelocityHdg> controllers;
    for (int i=0; i < uavs.size(); i++)
    {
        reference::VelocityHdg controller;
        controller.velocity = Eigen::Vector3d::Zero();

        controller.velocity += alignment_term(uavs, i, flocking_params);
        controller.velocity += attraction_term(uavs, i, flocking_params);
        controller.velocity += repulsion_term(uavs, i, flocking_params);
        controller.velocity += shill_term(uavs, i, flocking_params, obstacles);
        if (uavs[i].secondary_objective)
        {
            controller.velocity += secondary_objective(uavs, i, flocking_params, *uavs[i].secondary_objective);
        }

        if (controller.velocity.norm() > flocking_params.v_max)
        {
            controller.velocity = flocking_params.v_max * controller.velocity.normalized();
        }

        
        controller.heading = 0.0;

        controllers.push_back(controller);

    }
    return controllers;
}
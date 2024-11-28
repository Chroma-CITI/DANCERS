#include "vat_controller.hpp"
#include "rclcpp/rclcpp.hpp"

VATController::VATController(const int id, std::optional<Eigen::Vector3d> secondary_objective): id_(id), secondary_objective_(secondary_objective)
{
    // Iddle role parameter initialization
    VAT_params_iddle_.v_flock = 1.5;
    VAT_params_iddle_.v_max = 1;
    VAT_params_iddle_.a_frict = 4.16;
    VAT_params_iddle_.p_frict = 3.2;
    VAT_params_iddle_.r_0_frict = 85.3;
    VAT_params_iddle_.C_frict = 0.8;
    VAT_params_iddle_.v_frict = 0.63;
    VAT_params_iddle_.p_att = 0.08;
    VAT_params_iddle_.r_0_att = 15;
    VAT_params_iddle_.p_rep = 0.13;
    VAT_params_iddle_.r_0_rep = 15;
    VAT_params_iddle_.a_shill = 53;
    VAT_params_iddle_.p_shill = 3.55;
    VAT_params_iddle_.r_0_shill = 0.3;
    VAT_params_iddle_.v_shill = 13.622;

    // Mission role parameter initialization
    VAT_params_mission_.v_flock = 1.5;
    VAT_params_mission_.v_max = 1;
    VAT_params_mission_.a_frict = 4.16;
    VAT_params_mission_.p_frict = 3.2;
    VAT_params_mission_.r_0_frict = 85.3;
    VAT_params_mission_.C_frict = 0.8;
    VAT_params_mission_.v_frict = 0.63;
    VAT_params_mission_.p_att = 0.08;
    VAT_params_mission_.r_0_att = 15;
    VAT_params_mission_.p_rep = 0.13;
    VAT_params_mission_.r_0_rep = 15;
    VAT_params_mission_.a_shill = 53;
    VAT_params_mission_.p_shill = 3.55;
    VAT_params_mission_.r_0_shill = 0.3;
    VAT_params_mission_.v_shill = 13.622;
}

dancers_msgs::msg::VelocityHeading VATController::getVelocityHeading(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const std::vector<cuboid::obstacle_t>& obstacles)
{
    dancers_msgs::msg::VelocityHeading velocity_heading;
    VAT_params_t *role_params;
    if (self_agent.role_type == agent_util::AgentRoleType::Iddle)
    {
        role_params = &VAT_params_iddle_;
    }
    else if (self_agent.role_type == agent_util::AgentRoleType::Mission)
    {
        role_params = &VAT_params_mission_;
    }
    else
    {
        // Should not happen, add error logging
        return velocity_heading;
    }

    Eigen::Vector3d summed_velocity = Eigen::Vector3d::Zero();;

    summed_velocity = summed_velocity + alignmentTerm(self_agent, neighbors, role_params) +
                    attractionTerm(self_agent, neighbors, role_params) + 
                    repulsionTerm(self_agent, neighbors, role_params) +
                    shillTerm(self_agent, obstacles, role_params);

    // Verify if the agent has a secondary objective.
    if (secondary_objective_.has_value())
    {
        summed_velocity += secondaryObjective(self_agent, secondary_objective_.value(), role_params);
    }

    if (summed_velocity.norm() > role_params->v_max)
    {
        summed_velocity = role_params->v_max * summed_velocity.normalized();
    }
 
    velocity_heading.velocity.x = summed_velocity[0];
    velocity_heading.velocity.y = summed_velocity[1];
    velocity_heading.velocity.y = summed_velocity[2];

    
    return velocity_heading;
}

Eigen::Vector3d VATController::alignmentTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t* role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    if(role_params)
    {
        for (std::shared_ptr<const agent_util::AgentState_t> neighbor: neighbors)
        {
            const double distance = (self_agent.position - neighbor->position).norm();
            const double velDiffNorm = (self_agent.velocity - neighbor->velocity).norm();

            double v_frictmax = std::max((double)role_params->v_frict, sigmoidLin(distance - role_params->r_0_frict, role_params->a_frict, role_params->p_frict));
            if (velDiffNorm > v_frictmax)
            {
                result += role_params->C_frict * (velDiffNorm - v_frictmax) * (neighbor->velocity - self_agent.velocity) / velDiffNorm;
            }
        }
    }

    return result;
}

Eigen::Vector3d VATController::attractionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t* role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    if(role_params)
    {
        for (std::shared_ptr<const agent_util::AgentState_t> neighbor: neighbors)
        {
            const Eigen::Vector3d relative_position = self_agent.position  - neighbor->position;
            const double distance = relative_position.norm();

            if (distance > role_params->r_0_att)
            {
                result += role_params->p_att * (role_params->r_0_att - distance) * (relative_position) / distance;
            }
        }
    }
    return result;   
}

Eigen::Vector3d VATController::repulsionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t* role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    if(role_params)
    {
        for (std::shared_ptr<const agent_util::AgentState_t> neighbor: neighbors)
        {
            const Eigen::Vector3d relative_position = self_agent.position  - neighbor->position;
            const double distance = relative_position.norm();
            if (distance < role_params->r_0_rep && distance > 0.0)
            {
                result += role_params->p_rep * (role_params->r_0_rep - distance) * (relative_position) / distance;
            }
        }
    }

    return result;  
}

Eigen::Vector3d VATController::shillTerm(const agent_util::AgentState_t& self_agent, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t* role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    if(role_params)
    {
        for (const cuboid::obstacle_t& obstacle : obstacles)
        {
            Eigen::Vector3d shill_agent_position = cuboid::get_nearest_point_from_obstacle(self_agent.position, obstacle);
            const Eigen::Vector3d relative_shill_position = shill_agent_position - self_agent.position;

            Eigen::Vector3d shill_agent_velocity = role_params->v_shill * - relative_shill_position.normalized();
            const double velDiffNorm = (self_agent.velocity - shill_agent_velocity).norm();
            const double v_shillmax = sigmoidLin(relative_shill_position.norm() - role_params->r_0_shill, role_params->a_shill, role_params->p_shill);
            if (velDiffNorm > v_shillmax)
            {
                result += -(velDiffNorm - v_shillmax) * (self_agent.velocity - shill_agent_velocity) / velDiffNorm;
            }
        }
    }

    return result;   
}

Eigen::Vector3d VATController::secondaryObjective(const agent_util::AgentState_t& self_agent, const Eigen::Vector3d& goal, const VATController::VAT_params_t* role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    if(role_params)
    {
        Eigen::Vector3d relative_position = self_agent.position - goal;

        result += role_params->v_flock * -(relative_position);

        if (result.norm() > role_params->v_max)
        {
            result = role_params->v_max * result.normalized();
        }
    }

    return result;
}
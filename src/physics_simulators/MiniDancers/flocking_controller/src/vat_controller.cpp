#include "vat_controller.hpp"
#include "rclcpp/rclcpp.hpp"

VATController::VATController(const VATController::ControllerOptions_t& options): id_(options.id), 
                                                                                 VAT_params_iddle_(options.VAT_params_iddle),
                                                                                 VAT_params_mission_(options.VAT_params_mission),
                                                                                 desired_fixed_altitude_(options.desired_fixed_altitude),
                                                                                 secondary_objective_(options.secondary_objective) {}

dancers_msgs::msg::VelocityHeading VATController::getVelocityHeading(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const std::vector<cuboid::obstacle_t>& obstacles)
{
    dancers_msgs::msg::VelocityHeading velocity_heading;
    VAT_params_t *role_params;
    if (self_agent.role_type == agent_util::AgentRoleType::Idle)
    {
        role_params = &VAT_params_iddle_;
    }
    else if (self_agent.role_type == agent_util::AgentRoleType::Mission)
    {
        role_params = &VAT_params_mission_;
    }
    else
    {
        // Should not happen
        std::cout<<"Tried to access the parameters of a role that doesn't exist."<<std::endl;
        return velocity_heading;
    }

    Eigen::Vector3d summed_velocity = Eigen::Vector3d::Zero();;

    summed_velocity = shillTerm(self_agent, obstacles, role_params)
                    + alignmentTerm(self_agent, neighbors, role_params)
                    + attractionTerm(self_agent, neighbors, role_params)
                    + repulsionTerm(self_agent, neighbors, role_params);

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

    if (desired_fixed_altitude_.has_value())
    {
        // Proportional controller with deadband
        const float altitude_error = desired_fixed_altitude_.value() - self_agent.position[2];

        if (altitude_error > altitude_deadband_ || altitude_error < -altitude_deadband_)
        {
            velocity_heading.velocity.z = 0.1 * altitude_error;
        }
        else
        {
            velocity_heading.velocity.z = 0.0;
        }
    }
    else
    {
        velocity_heading.velocity.z = summed_velocity[2];
    }


    
    
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
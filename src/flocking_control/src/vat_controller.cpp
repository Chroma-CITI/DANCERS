#include "vat_controller.hpp"

#include <iostream>

VATController::VATController(VATController::VATControllerOptions_t options)
{
    this->VAT_options_ = std::move(options);
}

void VATController::SetTarget(const Eigen::Vector3d& new_target_position)
{
    this->VAT_options_.target_position = new_target_position;
}

dancers_msgs::msg::VelocityHeading VATController::getVelocityHeading(std::shared_ptr<const Agent>& self_agent, const std::vector<cuboid::obstacle_t>& obstacles)
{
    // Create the return ROS2 message
    dancers_msgs::msg::VelocityHeading velocity_heading;
    velocity_heading.agent_id = this->VAT_options_.id;
    Eigen::Vector3d summed_velocity = Eigen::Vector3d::Zero();

    // The id of agent should match the id expected from the controller.
    assert(this->VAT_options_.id == self_agent->id);

    // Clear the agent's last known neighbors list only if it still has agents
    if (!self_agent->neighbors.empty())
    {
        this->last_known_neighbors_.clear();
    }

    // Prepare the list of current neighbors as a vector
    std::vector<std::shared_ptr<const NeighborInfo>> current_neighbors;
    for (const auto& [neigh_id, neigh] : self_agent->neighbors)
    {
        std::shared_ptr<const NeighborInfo> neighbor_ptr = std::make_shared<const NeighborInfo>(neigh);
        current_neighbors.push_back(neighbor_ptr);
        this->last_known_neighbors_.push_back(neighbor_ptr);
    }

    // Compute the resultant flocking force
    summed_velocity +=  alignmentTerm(self_agent, current_neighbors, this->VAT_options_.VAT_params) + 
                        attractionTerm(self_agent, current_neighbors, this->VAT_options_.VAT_params) +
                        repulsionTerm(self_agent, current_neighbors, this->VAT_options_.VAT_params) +
                        shillTerm(self_agent, obstacles, this->VAT_options_.VAT_params);

    // Add the target attraction if this agent has a defined target
    if (this->VAT_options_.target_position.has_value())
    {
        summed_velocity += targetAttraction(self_agent, this->VAT_options_.target_position.value(), this->VAT_options_.VAT_params);
    }

    // Limit speed if too high
    if (summed_velocity.norm() > VAT_options_.VAT_params.v_max)
    {
        summed_velocity = VAT_options_.VAT_params.v_max * summed_velocity.normalized();
    }

    // Transfer to the ROS2 message
    velocity_heading.velocity.x = summed_velocity[0];
    velocity_heading.velocity.y = summed_velocity[1];

    // Altitude controller if defined.
    if (this->VAT_options_.desired_fixed_altitude.has_value())
    {
        // Proportional controller with deadband for fixed altitude
        const float altitude_error = this->VAT_options_.desired_fixed_altitude.value() - self_agent->position[2];

        if (altitude_error > this->altitude_deadband_ || altitude_error < -this->altitude_deadband_)
        {
            velocity_heading.velocity.z = 0.1 * altitude_error;
        }
        else
        {
            velocity_heading.velocity.z = 0.0;
        }
    }
    else if (this->VAT_options_.desired_min_altitude.has_value() && self_agent->position[2] < this->VAT_options_.desired_min_altitude.value())
    {
        // Proportional controller for minimal altitude
        const float altitude_error = this->VAT_options_.desired_min_altitude.value() - self_agent->position[2];

        velocity_heading.velocity.z = 0.1 * altitude_error;
    }
    else
    {
        // Otherwise, unbiased 3D flocking
        velocity_heading.velocity.z = summed_velocity[2];
    }

    return velocity_heading;
}



Eigen::Vector3d VATController::alignmentTerm(std::shared_ptr<const Agent>& self_agent, std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    for (std::shared_ptr<const NeighborInfo> neighbor: neighbors)
    {
        const double distance = (self_agent->position - neighbor->position).norm();
        const double velDiffNorm = (self_agent->velocity - neighbor->velocity).norm();

        double v_frictmax = std::max((double)role_params.v_frict, sigmoidLin(distance - role_params.r_0_frict, role_params.a_frict, role_params.p_frict));
        if (velDiffNorm > v_frictmax)
        {
            result += role_params.C_frict * (velDiffNorm - v_frictmax) * (neighbor->velocity - self_agent->velocity) / velDiffNorm;
        }
    }

    return result;
}

Eigen::Vector3d VATController::attractionTerm(std::shared_ptr<const Agent>& self_agent, std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const NeighborInfo> neighbor: neighbors)
    {
        const Eigen::Vector3d relative_position = neighbor->position - self_agent->position;
        const double distance = relative_position.norm();

        if (distance > role_params.r_0_att)
        {
            result += role_params.p_att * (distance - role_params.r_0_att) * (relative_position) / distance;
        }
    }
    return result;   
}

Eigen::Vector3d VATController::repulsionTerm(std::shared_ptr<const Agent>& self_agent, std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const NeighborInfo> neighbor: neighbors)
    {
        const Eigen::Vector3d relative_position = self_agent->position  - neighbor->position;
        const double distance = relative_position.norm();
        if (distance < role_params.r_0_rep && distance > 0.0)
        {
            result += role_params.p_rep * (role_params.r_0_rep - distance) * (relative_position) / distance;
        }
    }

    return result;  
}

Eigen::Vector3d VATController::shillTerm(std::shared_ptr<const Agent>& self_agent, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (const cuboid::obstacle_t& obstacle : obstacles)
    {
        Eigen::Vector3d shill_agent_position = cuboid::get_nearest_point_from_obstacle(self_agent->position, obstacle);
        const Eigen::Vector3d relative_shill_position = shill_agent_position - self_agent->position;
        if (relative_shill_position.norm() < role_params.r_obstacle_perception)
        {   
            Eigen::Vector3d shill_agent_velocity = role_params.v_shill * - relative_shill_position.normalized();
            const double velDiffNorm = (self_agent->velocity - shill_agent_velocity).norm();
            const double v_shillmax = sigmoidLin(relative_shill_position.norm() - role_params.r_0_shill, role_params.a_shill, role_params.p_shill);
            if (velDiffNorm > v_shillmax)
            {
                result += -(velDiffNorm - v_shillmax) * (self_agent->velocity - shill_agent_velocity) / velDiffNorm;
            }
        }
        else
        {
            // obstacle is out of the perception range of the agent
        }
    }

    return result;   
}

Eigen::Vector3d VATController::targetAttraction(std::shared_ptr<const Agent>& self_agent, const Eigen::Vector3d& goal, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    Eigen::Vector3d relative_position = goal - self_agent->position;

    if (role_params.r_0_tar < relative_position.norm())
    {
        result += role_params.p_tar*relative_position.normalized();
    }
    else if(0.0f < role_params.r_0_tar)
    {
        result += (relative_position.norm()/role_params.r_0_tar)*role_params.p_tar*relative_position.normalized();
    }

    return result;
}

Eigen::Vector3d VATController::losConservationTerm(std::shared_ptr<const Agent>& self_agent, const std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const NeighborInfo> neighbor: neighbors)
    {
        for (const cuboid::obstacle_t& obstacle : obstacles){
            cuboid::obstacle_t inflated_obstacle = cuboid::inflate_obst(obstacle, role_params.r_los_obst_inflation);

            std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> enter_exit_points = cuboid::segment_cuboid_intersection(self_agent->position, neighbor->position, inflated_obstacle);
            
            if(enter_exit_points)
            {
                double intersection_distance = (enter_exit_points->first - enter_exit_points->second).norm();
                
                std::pair<Eigen::Vector3d, Eigen::Vector3d> shifted_points = cuboid::computeNonIntersectingLine(self_agent->position, neighbor->position, inflated_obstacle);

                result = (shifted_points.first - self_agent->position).normalized();
                                
                // Scale force
                result = result * (intersection_distance*role_params.p_los);
            }
            else
            {
                // No intersection, no force.
            }
        }
    }
    return result;
}

double VATController::sigmoidLin(const double r, const double a, const double p)
{
    if (r <= 0)
    {
        return 0;
    }
    else if (r * p > 0 && r * p < a / p)
    {
        return r * p;
    }
    else
    {
        return std::sqrt(2 * a * r - std::pow(a, 2) / std::pow(p, 2));
    }
}
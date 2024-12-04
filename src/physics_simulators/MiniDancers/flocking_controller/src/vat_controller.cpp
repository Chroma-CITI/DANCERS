#include "vat_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <utility>

#define DEBUG_ID_TO_LOG 100

VATController::VATController(const VATController::ControllerOptions_t& options): id_(options.id), 
                                                                                 VAT_params_(std::move(options.VAT_params)),
                                                                                 desired_fixed_altitude_(options.desired_fixed_altitude),
                                                                                 secondary_objective_(options.secondary_objective) {}

dancers_msgs::msg::VelocityHeading VATController::getVelocityHeading(std::vector<std::shared_ptr<const agent_util::AgentState_t>>& agent_list, const std::vector<cuboid::obstacle_t>& obstacles)
{
    dancers_msgs::msg::VelocityHeading velocity_heading;
    Eigen::Vector3d summed_velocity = Eigen::Vector3d::Zero();

    const agent_util::AgentState_t& self_agent = *agent_list[id_];

    // The id of agent should match the id expected from the controller.
    assert(id_ == self_agent.id);

    // Create the list of neighbors based on role
    std::vector<std::shared_ptr<const agent_util::AgentState_t>> mission_neighbors;
    std::vector<std::shared_ptr<const agent_util::AgentState_t>> potential_neighbors;
    std::vector<std::shared_ptr<const agent_util::AgentState_t>> idle_neighbors;

    std::cout<<"Current agent ID: " << id_ << std::endl;
    for (auto neighbor : self_agent.neighbors)
    {
        if (neighbor.id != id_)
        {
            const agent_util::AgentState_t& neighbor_agent = *agent_list[neighbor.id];

            std::cout<<"Nieghbor Id: " << neighbor.id << ". Link quality: "<< neighbor.link_quality;
            if( neighbor_agent.role_type == agent_util::AgentRoleType::Undefined)
            {
                //Should not happen by definition of the Undefined role
                std::cout << "Agent of id " << id_ << " has an neighbor of id" << neighbor.id << " that has an undefined role. It shouldn't happen."<<std::endl;
            }
            else if (neighbor_agent.role_type == agent_util::AgentRoleType::Mission)
            {
                mission_neighbors.push_back(agent_list[neighbor.id]);
                std::cout<<" Role: Mission"<<std::endl;
                if (id_ == DEBUG_ID_TO_LOG)
                {
                    std::cout<<"Has neighbor "<< neighbor.id <<" as " <<" Mission"<<std::endl;
                }
            }
            else if (neighbor_agent.role_type == agent_util::AgentRoleType::Potential)
            {
                potential_neighbors.push_back(agent_list[neighbor.id]);
                std::cout<<" Role: Potential"<<std::endl;
                if (id_ == DEBUG_ID_TO_LOG)
                {
                    std::cout<<"Has neighbor "<< neighbor.id <<" as " <<" Potential"<<std::endl;
                }
            }
            else if (neighbor_agent.role_type == agent_util::AgentRoleType::Idle)
            {
                std::cout<<" Role: Idle"<<std::endl;
                idle_neighbors.push_back(agent_list[neighbor.id]);
                if (id_ == DEBUG_ID_TO_LOG)
                {
                    std::cout<<"Has neighbor "<< neighbor.id <<" as " <<" Idle"<<std::endl;
                }
            }
        }
    }

    if (self_agent.role_type == agent_util::AgentRoleType::Undefined)
    {
        // TODO: Find behavior for undefined roles
        if (id_ == DEBUG_ID_TO_LOG)
        {
            std::cout<<"Self role: Undefined"<<std::endl;
        }
    }
    else if(self_agent.role_type == agent_util::AgentRoleType::Mission)
    {     
        if (id_ == DEBUG_ID_TO_LOG)
        {
            std::cout<<"Self role: Mission"<<std::endl;
        }
        // Only interacts with mission neihbors
        summed_velocity += alignmentTerm(self_agent, mission_neighbors, VAT_params_[self_agent.role_type])
                        + attractionTerm(self_agent, mission_neighbors, VAT_params_[self_agent.role_type])
                        + repulsionTerm(self_agent, mission_neighbors, VAT_params_[self_agent.role_type]);
    }
    else if(self_agent.role_type == agent_util::AgentRoleType::Potential)
    {
        if (id_ == DEBUG_ID_TO_LOG)
        {
            std::cout<<"Self role: Potential"<<std::endl;
        }
        // Attracted to at most the two best (based on linked quality) mission neighbor and repulsed by other mission
        std::vector<std::shared_ptr<const agent_util::AgentState_t>> two_best_mission_neighbors;
        std::vector<std::shared_ptr<const agent_util::AgentState_t>> other_mission_neighbors;
        if (mission_neighbors.size()==1)
        {
            two_best_mission_neighbors.push_back(mission_neighbors[0]);
            std::cout<<"Has neighbor "<< mission_neighbors[0]->id <<" as " <<" best Mission"<<std::endl;
        }
        else if (mission_neighbors.size() >= 2)
        {
            two_best_mission_neighbors.push_back(mission_neighbors[0]);
            two_best_mission_neighbors.push_back(mission_neighbors[1]);
            std::cout<<"Has neighbor "<< mission_neighbors[0]->id <<" as " <<" best Mission"<<std::endl;
            std::cout<<"Has neighbor "<< mission_neighbors[1]->id <<" as " <<" best Mission"<<std::endl;

            for (int index = 2; index < mission_neighbors.size(); index++)
            {
                other_mission_neighbors.push_back(mission_neighbors[index]);
            }
        }
        else
        {
            //Should not happen by definition.
        }

        summed_velocity += attractionTerm(self_agent, two_best_mission_neighbors, VAT_params_[self_agent.role_type])
                        += alignmentTerm(self_agent, two_best_mission_neighbors, VAT_params_[self_agent.role_type]);
        
        // Repulsed by potential and idle neighbors.
        summed_velocity += repulsionTerm(self_agent, potential_neighbors, VAT_params_[self_agent.role_type])
                         + repulsionTerm(self_agent, idle_neighbors, VAT_params_[self_agent.role_type])
                         + repulsionTerm(self_agent, mission_neighbors, VAT_params_[self_agent.role_type]);
    }
    else if (self_agent.role_type == agent_util::AgentRoleType::Idle)
    {
        if (id_ == DEBUG_ID_TO_LOG)
        {
            std::cout<<"Self role: Idle"<<std::endl;
        }
        // Interacts normally with idle neighbors.
        summed_velocity += alignmentTerm(self_agent, idle_neighbors, VAT_params_[self_agent.role_type])
                         + attractionTerm(self_agent, idle_neighbors, VAT_params_[self_agent.role_type])
                         + repulsionTerm(self_agent, idle_neighbors, VAT_params_[self_agent.role_type]);

        // Interacts normally with potential neighbors.
        summed_velocity += alignmentTerm(self_agent, potential_neighbors, VAT_params_[self_agent.role_type])
                         + attractionTerm(self_agent, potential_neighbors, VAT_params_[self_agent.role_type])
                         + repulsionTerm(self_agent, potential_neighbors, VAT_params_[self_agent.role_type]);
    }
    else
    {
        std::cout<<"Error: Agent has an undefined role. The role as a casted int value of "<< self_agent.role_type << ". Returning a velocity of 0."<<std::endl;
        return velocity_heading;
    }

    // Obstacles avoidance
    summed_velocity += shillTerm(self_agent, obstacles, VAT_params_[self_agent.role_type]);

    // Verify if the agent has a secondary objective.
    if (secondary_objective_.has_value())
    {
        summed_velocity += secondaryObjective(self_agent, secondary_objective_.value(), VAT_params_[self_agent.role_type]);
    }

    // Limit speed if too high.
    if (summed_velocity.norm() > VAT_params_[self_agent.role_type].v_max)
    {
        summed_velocity = VAT_params_[self_agent.role_type].v_max * summed_velocity.normalized();
    }

    velocity_heading.velocity.x = summed_velocity[0];
    velocity_heading.velocity.y = summed_velocity[1];

    // Altitude controller if defined.
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

Eigen::Vector3d VATController::alignmentTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    for (std::shared_ptr<const agent_util::AgentState_t> neighbor: neighbors)
    {
        const double distance = (self_agent.position - neighbor->position).norm();
        const double velDiffNorm = (self_agent.velocity - neighbor->velocity).norm();

        double v_frictmax = std::max((double)role_params.v_frict, sigmoidLin(distance - role_params.r_0_frict, role_params.a_frict, role_params.p_frict));
        if (velDiffNorm > v_frictmax)
        {
            result += role_params.C_frict * (velDiffNorm - v_frictmax) * (neighbor->velocity - self_agent.velocity) / velDiffNorm;
        }
    }

    return result;
}

Eigen::Vector3d VATController::attractionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const agent_util::AgentState_t> neighbor: neighbors)
    {
        const Eigen::Vector3d relative_position = self_agent.position  - neighbor->position;
        const double distance = relative_position.norm();

        if (distance > role_params.r_0_att)
        {
            result += role_params.p_att * (role_params.r_0_att - distance) * (relative_position) / distance;
        }
    }
    return result;   
}

Eigen::Vector3d VATController::repulsionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const agent_util::AgentState_t> neighbor: neighbors)
    {
        const Eigen::Vector3d relative_position = self_agent.position  - neighbor->position;
        const double distance = relative_position.norm();
        if (distance < role_params.r_0_rep && distance > 0.0)
        {
            result += role_params.p_rep * (role_params.r_0_rep - distance) * (relative_position) / distance;
        }
    }

    return result;  
}

Eigen::Vector3d VATController::shillTerm(const agent_util::AgentState_t& self_agent, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (const cuboid::obstacle_t& obstacle : obstacles)
    {
        Eigen::Vector3d shill_agent_position = cuboid::get_nearest_point_from_obstacle(self_agent.position, obstacle);
        const Eigen::Vector3d relative_shill_position = shill_agent_position - self_agent.position;

        Eigen::Vector3d shill_agent_velocity = role_params.v_shill * - relative_shill_position.normalized();
        const double velDiffNorm = (self_agent.velocity - shill_agent_velocity).norm();
        const double v_shillmax = sigmoidLin(relative_shill_position.norm() - role_params.r_0_shill, role_params.a_shill, role_params.p_shill);
        if (velDiffNorm > v_shillmax)
        {
            result += -(velDiffNorm - v_shillmax) * (self_agent.velocity - shill_agent_velocity) / velDiffNorm;
        }
    }

    return result;   
}

Eigen::Vector3d VATController::secondaryObjective(const agent_util::AgentState_t& self_agent, const Eigen::Vector3d& goal, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d relative_position = self_agent.position - goal;

    result += role_params.v_flock * -(relative_position);

    if (result.norm() > role_params.v_max)
    {
        result = role_params.v_max * result.normalized();
    }

    return result;
}
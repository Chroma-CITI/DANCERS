#include "vat_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <utility>

#define DEBUG_ID_TO_LOG 1


void printVectorDebug(Eigen::Vector3d v)
{
    bool colorX=false, colorY=false, colorZ=false;
    if (v.x() > 1000)
    {
        colorX = true;
    }
    if (v.y() > 1000)
    {
        colorY = true;
    }
    if (v.z() > 1000)
    {
        colorZ = true;
    }
    std::cout << "( " << (colorX ? "\033[1;31m" : "") << v.x() << (colorX ? "\033[0m" : "") << ", " << (colorY ? "\033[1;31m" : "") << v.y() << (colorY ? "\033[0m" : "") << ", " << (colorZ ? "\033[1;31m" : "") << v.z() << (colorZ ? "\033[0m" : "") << " )" << std::endl;
}

VATController::VATController(const VATController::ControllerOptions_t& options): id_(options.id), 
                                                                                 VAT_params_(std::move(options.VAT_params)),
                                                                                 desired_fixed_altitude_(options.desired_fixed_altitude),
                                                                                 desired_min_altitude_(options.desired_min_altitude),
                                                                                 secondary_objective_(options.secondary_objective),
                                                                                 path_planner_params_(options.path_planner_params_) {}

void VATController::SetSecondaryObjective(const Eigen::Vector3d& secondary_objective)
{
    secondary_objective_ = secondary_objective;
}


dancers_msgs::msg::VelocityHeading VATController::getVelocityHeading(const agent_util::AgentState_t& agent_state, const std::vector<cuboid::obstacle_t>& obstacles)
{
    dancers_msgs::msg::VelocityHeading velocity_heading;
    velocity_heading.agent_id = id_;
    Eigen::Vector3d summed_velocity = Eigen::Vector3d::Zero();

    // The id of agent should match the id expected from the controller.
    assert(id_ == agent_state.id);

    // Create the list of neighbors based on role
    std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>> mission_neighbors;
    std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>> potential_neighbors;
    std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>> idle_neighbors;

    if (!agent_state.neighbors.empty())
    {
        this->last_known_neighbors_.clear();
    }

    //std::cout<<"Current agent ID: " << id_ << std::endl;
    for (auto neighbor : agent_state.neighbors)
    {
        if (neighbor.id != id_)
        {
            //std::cout<<"Neighbor Id: " << neighbor.id << ". Link quality: "<< neighbor.link_quality;
            if(neighbor.role_type == agent_util::AgentRoleType::Undefined)
            {
                //Should not happen by definition of the Undefined role
                //std::cout << "Agent of id " << id_ << " has an neighbor of id" << neighbor.id << " that has an undefined role. It shouldn't happen."<<std::endl;
            }
            else if (neighbor.role_type == agent_util::AgentRoleType::Mission)
            {
                mission_neighbors.push_back(std::make_shared<const agent_util::NeighborInfo_t>(neighbor));
                this->last_known_neighbors_.push_back(std::make_shared<const agent_util::NeighborInfo_t>(neighbor));
                //std::cout<<" Role: Mission"<<std::endl;
            }
            else if (neighbor.role_type == agent_util::AgentRoleType::Potential)
            {
                potential_neighbors.push_back(std::make_shared<const agent_util::NeighborInfo_t>(neighbor));
                this->last_known_neighbors_.push_back(std::make_shared<const agent_util::NeighborInfo_t>(neighbor));
                //std::cout<<" Role: Potential"<<std::endl;
            }
            else if (neighbor.role_type == agent_util::AgentRoleType::Idle)
            {
                //std::cout<<" Role: Idle"<<std::endl;
                idle_neighbors.push_back(std::make_shared<const agent_util::NeighborInfo_t>(neighbor));
                this->last_known_neighbors_.push_back(std::make_shared<const agent_util::NeighborInfo_t>(neighbor));
            }
        }
    }

    if (agent_state.role_type == agent_util::AgentRoleType::Undefined)
    {
        if (id_ == DEBUG_ID_TO_LOG)
        {
            //std::cout<<"Self role: Undefined"<<std::endl;
        }
        // In this role, if we the neighbor list of the agent is not empty, it represents the last neighborhood known by the agent, before it switched to undefined role. In this case, we are attracted by our last neighborhood.
        summed_velocity += attractionTerm(agent_state, this->last_known_neighbors_, VAT_params_[agent_state.role_type]);
    }
    else if(agent_state.role_type == agent_util::AgentRoleType::Mission)
    {     
        if (id_ == DEBUG_ID_TO_LOG)
        {
            //std::cout<<"Self role: Mission"<<std::endl;
        }
        // Only interacts with mission neihbors
        summed_velocity += alignmentTerm(agent_state, mission_neighbors, VAT_params_[agent_state.role_type])
                        + attractionTerm(agent_state, mission_neighbors, VAT_params_[agent_state.role_type])
                        + repulsionTerm(agent_state, mission_neighbors, VAT_params_[agent_state.role_type])
                        + losConservationTerm(agent_state, mission_neighbors, obstacles, VAT_params_[agent_state.role_type]);
    }
    else if(agent_state.role_type == agent_util::AgentRoleType::Potential)
    {
        if (id_ == DEBUG_ID_TO_LOG)
        {
            //std::cout<<"Self role: Potential"<<std::endl;
        }
        // Attracted to at most the two best (based on linked quality) mission neighbor and repulsed by other mission
        std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>> two_best_mission_neighbors;
        std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>> other_mission_neighbors;
        if (mission_neighbors.size()==1)
        {
            two_best_mission_neighbors.push_back(mission_neighbors[0]);
            //std::cout<<"Has neighbor "<< mission_neighbors[0]->id <<" as " <<" best Mission"<<std::endl;
        }
        else if (mission_neighbors.size() >= 2)
        {
            two_best_mission_neighbors.push_back(mission_neighbors[0]);
            two_best_mission_neighbors.push_back(mission_neighbors[1]);
            //std::cout<<"Has neighbor "<< mission_neighbors[0]->id <<" as " <<" best Mission"<<std::endl;
            //std::cout<<"Has neighbor "<< mission_neighbors[1]->id <<" as " <<" best Mission"<<std::endl;

            for (int index = 2; index < mission_neighbors.size(); index++)
            {
                other_mission_neighbors.push_back(mission_neighbors[index]);
            }
        }
        else
        {
            //Should not happen by definition.
        }

        summed_velocity += attractionTerm(agent_state, two_best_mission_neighbors, VAT_params_[agent_state.role_type])
                        += alignmentTerm(agent_state, two_best_mission_neighbors, VAT_params_[agent_state.role_type]);
        
        // Repulsed by potential and idle neighbors.
        summed_velocity += repulsionTerm(agent_state, potential_neighbors, VAT_params_[agent_state.role_type])
                         + repulsionTerm(agent_state, idle_neighbors, VAT_params_[agent_state.role_type])
                         + repulsionTerm(agent_state, mission_neighbors, VAT_params_[agent_state.role_type]);
    }
    else if (agent_state.role_type == agent_util::AgentRoleType::Idle)
    {
        if (id_ == DEBUG_ID_TO_LOG)
        {
            //std::cout<<"Self role: Idle"<<std::endl;
        }

        // Interacts normally with idle neighbors.
        summed_velocity += alignmentTerm(agent_state, idle_neighbors, VAT_params_[agent_state.role_type])
                         + attractionTerm(agent_state, idle_neighbors, VAT_params_[agent_state.role_type])
                         + repulsionTerm(agent_state, idle_neighbors, VAT_params_[agent_state.role_type]);

        // Interacts normally with potential neighbors.
        summed_velocity += alignmentTerm(agent_state, potential_neighbors, VAT_params_[agent_state.role_type])
                         + attractionTerm(agent_state, potential_neighbors, VAT_params_[agent_state.role_type])
                         + repulsionTerm(agent_state, potential_neighbors, VAT_params_[agent_state.role_type]);
    }
    else
    {
        //std::cout<<"Error: Agent has an undefined role. The role has a casted int value of "<< self_agent.role_type << ". Returning a velocity of 0."<<std::endl;
        return velocity_heading;
    }

    // Obstacles avoidance
    summed_velocity += shillTerm(agent_state, obstacles, VAT_params_[agent_state.role_type]);

    // Verify if the agent has a secondary objective. (Deactivate the secondary objective if we are undefined).
    if (secondary_objective_.has_value() 
            // && agent_state.role_type != agent_util::AgentRoleType::Undefined
            )
    {
        if(path_planner_params_.use_planner_)
        {
            summed_velocity += pathFollowing(agent_state, secondary_objective_.value(), VAT_params_[agent_state.role_type]);
        }
        else
        {
            summed_velocity += secondaryObjective(agent_state, secondary_objective_.value(), VAT_params_[agent_state.role_type]);
        }
    }

    // Limit speed if too high.
    if (summed_velocity.norm() > VAT_params_[agent_state.role_type].v_max)
    {
        summed_velocity = VAT_params_[agent_state.role_type].v_max * summed_velocity.normalized();
    }

    velocity_heading.velocity.x = summed_velocity[0];
    velocity_heading.velocity.y = summed_velocity[1];

    // Altitude controller if defined.
    if (desired_fixed_altitude_.has_value())
    {
        // Proportional controller with deadband
        const float altitude_error = desired_fixed_altitude_.value() - agent_state.position[2];

        if (altitude_error > altitude_deadband_ || altitude_error < -altitude_deadband_)
        {
            velocity_heading.velocity.z = 0.1 * altitude_error;
        }
        else
        {
            velocity_heading.velocity.z = 0.0;
        }
    }
    else if (desired_min_altitude_.has_value() && agent_state.position[2] < desired_min_altitude_.value())
    {
        const float altitude_error = desired_min_altitude_.value() - agent_state.position[2];

        velocity_heading.velocity.z = 0.1 * altitude_error;
    }
    else
    {
        velocity_heading.velocity.z = summed_velocity[2];
    }

    return velocity_heading;
}

Eigen::Vector3d VATController::alignmentTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    for (std::shared_ptr<const agent_util::NeighborInfo_t> neighbor: neighbors)
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

Eigen::Vector3d VATController::attractionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const agent_util::NeighborInfo_t> neighbor: neighbors)
    {
        const Eigen::Vector3d relative_position = neighbor->position - self_agent.position;
        const double distance = relative_position.norm();

        if (distance > role_params.r_0_att)
        {
            if (self_agent.role_type == agent_util::AgentRoleType::Mission && role_params.use_squared_attraction_term)
            {
                result += role_params.p_att * pow((distance - role_params.r_0_att), 2) * (relative_position) / distance;
            }
            else
            {
                result += role_params.p_att * (distance - role_params.r_0_att) * (relative_position) / distance;
            }
        }
    }
    return result;   
}

Eigen::Vector3d VATController::repulsionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>>& neighbors, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const agent_util::NeighborInfo_t> neighbor: neighbors)
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

    Eigen::Vector3d relative_position = goal - self_agent.position;

    if (role_params.r_0_sec < relative_position.norm())
    {
        result += role_params.v_sec_max*relative_position.normalized();
    }
    else if(0.0f < role_params.r_0_sec)
    {
        result += (relative_position.norm()/role_params.r_0_sec)*role_params.v_sec_max*relative_position.normalized();
    }

    return result;
}

Eigen::Vector3d VATController::pathFollowing(const agent_util::AgentState_t& self_agent, const Eigen::Vector3d& goal, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d waypoint = goal;

    //Change goal for next point on the trajectory if using a planner.
    if(path_planner_params_.path_planner_ != nullptr)
    {
        waypoint = path_planner_params_.path_planner_->getNextWaypoint(self_agent.position, goal, 
                                                                       path_planner_params_.goal_radius_tolerance_, 
                                                                       path_planner_params_.distance_to_path_tolerance_, 
                                                                       path_planner_params_.lookup_ahead_pursuit_distance_).position;
    }

    Eigen::Vector3d relative_position = waypoint - self_agent.position;

    if (role_params.r_0_sec_path < relative_position.norm())
    {
        result += role_params.v_sec_max_path*relative_position.normalized();
    }
    else if(0.0f < role_params.r_0_sec_path)
    {
        result += relative_position.norm()/role_params.r_0_sec_path*role_params.v_sec_max_path * relative_position.normalized();
    }

    return result;
}


Eigen::Vector3d VATController::losConservationTerm(const agent_util::AgentState_t& self_agent, const std::vector<std::shared_ptr<const agent_util::NeighborInfo_t>>& neighbors, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (std::shared_ptr<const agent_util::NeighborInfo_t> neighbor: neighbors)
    {
        for (const cuboid::obstacle_t& obstacle : obstacles){
            cuboid::obstacle_t inflated_obstacle = cuboid::inflate_obst(obstacle, role_params.r_los_obst_inflation);

            std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> enter_exit_points = cuboid::segment_cuboid_intersection(self_agent.position, neighbor->position, inflated_obstacle);
            
            if(enter_exit_points)
            {
                double intersection_distance = (enter_exit_points->first - enter_exit_points->second).norm();
                
                std::pair<Eigen::Vector3d, Eigen::Vector3d> shifted_points = cuboid::computeNonIntersectingLine(self_agent.position, neighbor->position, inflated_obstacle);

                result = (shifted_points.first - self_agent.position).normalized();
                                
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
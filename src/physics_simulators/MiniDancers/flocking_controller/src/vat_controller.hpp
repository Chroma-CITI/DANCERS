#pragma once

#include <map>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include <dancers_msgs/msg/agent_state.hpp>
#include <dancers_msgs/msg/velocity_heading.hpp>

/**
 * @brief Flocking controller based on the VAT controller for a single agent with parameters 
 * for iddle neighbors and mission neighbors. 
 * @details The VAT mathematical model uses the model from https://hal.science/hal-03365129.
 */
class VATController
{
    public:
        /**
         * @brief Role of agent. Iddle means the agent is not part of the mission's data routhing path while Mission is.
         */
        enum AgentRoleType
        {
            Iddle=0,
            Mission
        };
        
        /**
         * @brief Parameters of the VAT described in https://hal.science/hal-03365129.
         */
        struct VAT_params_t
        {
            float v_flock;
            float v_max;
            float a_frict;
            float p_frict;
            float r_0_frict;
            float C_frict;
            float v_frict;
            float p_att;
            float r_0_att;
            float p_rep;
            float r_0_rep;
            float a_shill;
            float p_shill;
            float r_0_shill;
            float v_shill;
        };

        /**
         * @brief Computes the velocity command of the agent based on the VAT controller.
         * @param agent Agent containinig its position, velocities and neighbors information.
         * @param obstacles Array of 3D obstacles in the environement.
         * @return Return the computed velocity command as a ROS message. 
         */
        dancers_msgs::msg::VelocityHeading getVelocityHeading(const agent_t& agent, const std::vector<obstacle_t>& obstacles);
    
        /**
         * @brief Constructor of the VAT controller that initialize the internal states of the controller.
         * @param id The id of the agent among the swarm.
         */
        VATController(const int id);

    private:
        /**
         * @brief Id of the agent among the swarm. 
         */
        int id_;

        /**
         * @brief Role of the current agent.
         */
         AgentRoleType self_role_ = AgentRoleType::Iddle;

        /**
         * @brief Objective point in space that attracts the agent, if defined.
         */
        std::optional<Eigen::Vector3d> secondary_objective_;

        /**
         * @brief VAT parameters for the iddle role.
         */
        VAT_params_t VAT_params_iddle_;

        /**
         * @brief VAT parameters for the mission role.
         */
        VAT_params_t VAT_params_mission_;
        
        /**
         * @brief VAT parameters for the iddle neighbors.
         */
        VATController::VAT_params_t iddle_params_;


        /* ----------- Flocking behaviors ----------- */
        /**
         * @brief Flocking behavior that aligns the agent with the alignement of its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors of the given type.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d alignmentTerm(const agent_t& self_agent, const std::vector<agent_t *>& neighbors);

        /**
         * @brief Flocking behavior that attracts the agent towards its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors of the given type.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d attraction_term(const agent_t& self_agent, const std::vector<agent_t *>& neighbors);

        /**
         * @brief Flocking behavior that repulse the agent from its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors of the given type.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d repulsion_term(const agent_t& self_agent, const std::vector<agent_t *>& neighbors);

        /**
         * @brief Flocking behavior that pushes the heading of the agent perpendicaly from the nearest obstacle surfaces.
         * @param self_agent Current agent states.
         * @param obstacles 3D Obstacles that present in the environment.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d shill_term(const agent_t& self_agent, const std::vector<obstacle_t>& obstacles);
        
        /**
         * @brief Flocking behavior that aligns the agent with the alignement of its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d secondary_objective(const agent_t& self_agent);
};
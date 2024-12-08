#pragma once

#include <map>
#include <optional>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include <dancers_msgs/msg/agent_state.hpp>
#include <dancers_msgs/msg/neighbor_array.hpp>
#include <dancers_msgs/msg/velocity_heading.hpp>

#include <flocking_controller/agent_util.hpp>
#include <flocking_controller/cuboid_obstacle_util.hpp>

/**
 * @brief Flocking controller based on the VAT controller for a single agent with parameters 
 * for idle neighbors and mission neighbors.
 * @details The VAT mathematical model uses the model from https://hal.science/hal-03365129.
 */
class VATController
{
    public:
        /**
         * @brief Parameters of the VAT described in https://hal.science/hal-03365129.
         */
        struct VAT_params_t
        {
            float v_flock;
            float v_max;
            float v_sec_max;
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

            float expected_deconnexion_distance;
            bool use_deconnexion_distance_instead_of_p_att = false;
        };

        /**
         * @struct ControllerOptions_t
         * @brief Options passed in the VATController constructor to initialized it.
         */
        struct ControllerOptions_t
        {
            /**
             * @brief Id of the agent
             */
            int id;

            /**
             * @brief VAT parameters for all roles.
             */
            std::map<agent_util::AgentRoleType, VAT_params_t> VAT_params = {
                {agent_util::AgentRoleType::Undefined, {}},
                {agent_util::AgentRoleType::Mission, {}},
                {agent_util::AgentRoleType::Potential, {}},
                {agent_util::AgentRoleType::Idle, {}}
            };

            /**
             * @brief Optional fixed altitude that, if set, will overide the flocking behavior to control the altitude of the drones within a 1m deadband of the set altitude.
             * Default value is std::nullopt.
             */
            std::optional<float> desired_fixed_altitude = std::nullopt;

            /**
             * @brief Objective point in space that attracts the agent, if defined.
             * Default value is std::nullopt.
             */
            std::optional<Eigen::Vector3d> secondary_objective = std::nullopt;
        };

        /**
         * @brief Computes the velocity command of the agent based on the VAT controller.
         * @param agent_list List of all agents states and their neighbors.
         * @param obstacles Array of 3D obstacles in the environement.
         * @return Return the computed velocity command as a ROS message. 
         */
        dancers_msgs::msg::VelocityHeading getVelocityHeading(std::vector<std::shared_ptr<const agent_util::AgentState_t>>& agent_list, const std::vector<cuboid::obstacle_t>& obstacles);
    
        /**
         * @brief Constructor of the VAT controller that initialize the internal states of the controller.
         * @param options Struct containing all the options used in the initialization.
         */
        VATController(const VATController::ControllerOptions_t& options);

    private:
        /**
         * @brief Id of the agent among the swarm. 
         */
        int id_;

        /**
         * @brief VAT parameter dictionnaries that contains the parameter for each role of neighbors.
         */
        std::map<agent_util::AgentRoleType, VAT_params_t> VAT_params_ = {
            {agent_util::AgentRoleType::Undefined, {}},
            {agent_util::AgentRoleType::Mission, {}},
            {agent_util::AgentRoleType::Potential, {}},
            {agent_util::AgentRoleType::Idle, {}}
        };

        /**
         * @brief Objective point in space that attracts the agent, if defined.
         */
        std::optional<Eigen::Vector3d> secondary_objective_;

        /**
         * @brief Optional fixed altitude that, if set, will overide the flocking behavior to control the altitude of the drones within a 1m deadband of the set altitude.
         * It's mainly used to constrained the flocking in 2D.
         */
        std::optional<float> desired_fixed_altitude_;

        /**
         * @brief Deadband used in the altitude controller if desired_fixed_altitude is defined. The deadband is applied above and below the target. 
         * Thus, the deadband is between "desired_fixed_altitude - altitude_deadband_" and "desired_fixed_altitude + altitude_deadband_"
         */
        const float altitude_deadband_ = 0.5;

        /* ----------- Flocking behaviors ----------- */
        /**
         * @brief Flocking behavior that aligns the agent with the alignement of its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d alignmentTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that attracts the agent towards its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d attractionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that repulse the agent from its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d repulsionTerm(const agent_util::AgentState_t& self_agent, std::vector<std::shared_ptr<const agent_util::AgentState_t>>& neighbors, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that pushes the heading of the agent perpendicaly from the nearest obstacle surfaces.
         * @param self_agent Current agent states.
         * @param obstacles 3D Obstacles that present in the environment.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d shillTerm(const agent_util::AgentState_t& self_agent, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params);
        
        /**
         * @brief Flocking behavior that attracts the agent towards a goal position.
         * @param self_agent Current agent states.
         * @param goal Goal position that attracts the agent.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d secondaryObjective(const agent_util::AgentState_t& self_agent, const Eigen::Vector3d& goal, const VATController::VAT_params_t& role_params);


        /**
         * @brief Function used for flocking computation, see curve in Vásárhelyi 2018 Fig.6. for parameters.
         */
        double sigmoidLin(const double r, const double a, const double p)
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
};
#pragma once

// Standard C++ libs
#include <optional>

// Eigen lib
#include <Eigen/Dense>

#include <dancers_msgs/msg/agent_struct.hpp>
#include <dancers_msgs/msg/velocity_heading.hpp>

// Custom libs
#include <agent.hpp>
#include <cuboid_obstacle_util.hpp>

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
            float v_max;
            float p_tar;
            float r_0_tar;
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
            float p_los;
            float r_los_obst_inflation;
            float r_obstacle_perception;
        };

        /**
         * @struct VATControllerOptions_t
         * @brief Options passed in the VATController constructor to initialized it.
         */
        struct VATControllerOptions_t
        {
            /**
             * @brief Id of the agent
             */
            uint32_t id;

            /**
             * @brief VAT parameters.
             */
            VAT_params_t VAT_params = {};

            /**
             * @brief Optional fixed altitude that, if set, will override the flocking behavior to control the altitude of the drones within a 1m deadband of the set altitude.
             * Default value is std::nullopt.
             */
            std::optional<float> desired_fixed_altitude = std::nullopt;

            /**
             * @brief Optional minimal altitude that, if set, will add an upward force if the UAV drops below the minimal altitude. Above this threshold, the UAV moves in 3D.
             * Default value is std::nullopt.
             */
            std::optional<float> desired_min_altitude = std::nullopt;

            /**
             * @brief Objective point in space that attracts the agent, if defined.
             * Default value is std::nullopt.
             */
            std::optional<Eigen::Vector3d> target_position = std::nullopt;
        };

        /**
         * @brief Constructor of the VAT controller that initialize the internal states of the controller.
         * @param options Struct containing all the options used in the initialization.
         */
        VATController(VATController::VATControllerOptions_t options);

        /**
         * @brief Computes the velocity command of the agent based on the VAT controller.
         * @param self_agent Pointer to the agent state including its neighbors.
         * @param obstacles Array of 3D obstacles in the environement.
         * @return Return the computed velocity command as a ROS message. 
         */
        dancers_msgs::msg::VelocityHeading getVelocityHeading(std::shared_ptr<const Agent>& self_agent, const std::vector<cuboid::obstacle_t>& obstacles);

        /**
         * @brief Set the secondary objective of the agent.
         * @param secondary_objective New secondary objective of the agent.
         */
        void SetTarget(const Eigen::Vector3d& secondary_objective);

    private:
        /**
         * @brief VAT parameters.
         */
        VATControllerOptions_t VAT_options_;

        /**
         * @brief Deadband used in the altitude controller if desired_fixed_altitude is defined. The deadband is applied above and below the target. 
         * Thus, the deadband is between "desired_fixed_altitude - altitude_deadband_" and "desired_fixed_altitude + altitude_deadband_"
         */
        const float altitude_deadband_ = 0.5;

        /**
         * @brief A list of neighbors that is a snapshot of the last known positions of our neighbors. It is used to navigate back to their position in case of complete lost of connectivity
         */
        std::vector<std::shared_ptr<const NeighborInfo>> last_known_neighbors_;

        /* ----------- Flocking behaviors ----------- */
        /**
         * @brief Flocking behavior that aligns the agent with the alignement of its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d alignmentTerm(std::shared_ptr<const Agent>& self_agent, std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that attracts the agent towards its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d attractionTerm(std::shared_ptr<const Agent>& self_agent, std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that repulse the agent from its neighbors of a given neighbor type.
         * @param self_agent Current agent states.
         * @param neighbors List of all neighbors.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d repulsionTerm(std::shared_ptr<const Agent>& self_agent, std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that pushes the heading of the agent perpendicaly from the nearest obstacle surfaces.
         * @param self_agent Current agent states.
         * @param obstacles 3D Obstacles that present in the environment.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d shillTerm(std::shared_ptr<const Agent>& self_agent, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params);
        
        /**
         * @brief Flocking behavior that attracts the agent towards a goal position.
         * @param self_agent Current agent states.
         * @param goal Goal position that attracts the agent.
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d targetAttraction(std::shared_ptr<const Agent>& self_agent, const Eigen::Vector3d& goal, const VATController::VAT_params_t& role_params);

        /**
         * @brief Flocking behavior that helps an agent to conserve LOS with its neighbors
         * @param self_agent Current agent state.
         * @param neighbors List of all neighbors.
         * @param obstacles 3D Obstacles that present in the environment (AABB).
         * @param params Flocking parameters to use.
         * @return Velocity command of the behavior.
         */
        Eigen::Vector3d losConservationTerm(std::shared_ptr<const Agent>& self_agent, const std::vector<std::shared_ptr<const NeighborInfo>>& neighbors, const std::vector<cuboid::obstacle_t>& obstacles, const VATController::VAT_params_t& role_params);

        /**
         * @brief Function used for flocking computation, see curve in Vásárhelyi 2018 Fig.6. for parameters.
         */
        double sigmoidLin(const double r, const double a, const double p);

};
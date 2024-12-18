#pragma once

#include <mutex>

#include <Eigen/Core>

#include <nav_msgs/msg/path.hpp>

/**
 * @brief Base path planner class.
 */
class PathPlanner
{
    public:
        
        /**
         *  @brief Structure of waypoints to navigate to.
         */
        struct Waypoint
        {
            Eigen::Vector3d position;
        };

        /**
         * @brief Computes the next waypoint that a agent should take based on a global planner.
         * @param agent_position Current position of the agent from which the path will be made and
         *  to find out where on the trajectory the agent is to compute the nex waypoint.
         * @param goal_position Position of the goal that the path needs to finish to.
         * @param goal_radius_tolerance Radius around the goal in which cells will be considered as
         *  a valid end point for the path planning algorithm.
         * @param distance_to_path_tolerance Maximum distance that an agent can be from the path before 
         * retriggering a planning since the path might be invalid.
         * @param lookup_ahead_pursuit_distance Distance on the path towards the goal from the agent's 
         * closest point on the trajectory to determine the next waypoint. It's the length of the stick 
         * in the analogy of a carrot on a stick.
         * @return The next waypoint on the trajectory that the agent should follow to follow the path.
         */
        virtual Waypoint getNextWaypoint(Eigen::Vector3d agent_position, 
                                         Eigen::Vector3d goal_position, 
                                         const float goal_radius_tolerance,
                                         const float distance_to_path_tolerance,
                                         float lookup_ahead_pursuit_distance) = 0;

        /**
         * @brief Returns the current waypoint which is the last computed waypoint with PathPlanner::getNextWaypoint.
         * @return The current waypoint.
         */
        virtual Waypoint getCurrentWaypoint() = 0;

        /**
         * @brief Returns the current path.
         * @return The path.
         */
        inline nav_msgs::msg::Path getPath()
        {
            std::lock_guard<std::mutex> paht_lock(path_manipulation_mutex_);
            return current_path_;
        }
    
    protected:
        /**
         * @brief The last computed internal path.   
         */
        nav_msgs::msg::Path current_path_;

        /**
         * @brief Indicates if the current path is empty.
         * @return Returns if the path is empty or not.
         */
        inline bool isPathEmpty()
        {
            if (current_path_.poses.size() == 0)
            {
                return true;
            }
            return false;
        }

        /**
         * @brief Mutex used by base and derived class to protect the access to current_path_.
         */
        std::mutex path_manipulation_mutex_;
};
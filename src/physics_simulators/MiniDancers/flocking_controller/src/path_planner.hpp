#pragma once

#include <mutex>

#include <Eigen/Core>

#include <nav_msgs/msg/path.hpp>

class PathPlanner
{
    public:
        
        struct Waypoint
        {
            Eigen::Vector3d position;
        };

        virtual Waypoint getNextWaypoint(Eigen::Vector3d starting_point, 
                                                        Eigen::Vector3d goal_point, 
                                                        const float goal_radius_tolerance,
                                                        const float distance_to_path_tolerance,
                                                        float lookup_ahead_pursuit_distance) = 0;
    
    protected:
        nav_msgs::msg::Path current_path_;

        inline bool isPathEmpty()
        {
            if (current_path_.poses.size() == 0)
            {
                return true;
            }
            return false;
        }

        std::mutex path_manipulation_mutex_;

        inline nav_msgs::msg::Path getPath()
        {
            std::lock_guard<std::mutex> paht_lock(path_manipulation_mutex_);
            return current_path_;
        }
};
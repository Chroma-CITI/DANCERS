#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "occupancy_grid.hpp"
#include "path_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Core>

class GridPathPlanner: public PathPlanner
{
    public:

        class SearchNode
        {
            public:
                SearchNode(const OccupancyGrid2D::CellCoordinates& coordinates,const Eigen::Vector3d& center_of_cell_pos): 
                          coordinates_(coordinates), center_of_cell_pos_(center_of_cell_pos) {}
                
                OccupancyGrid2D::CellCoordinates coordinates_;
                
                Eigen::Vector3d center_of_cell_pos_;

                std::shared_ptr<SearchNode> parent_;
                
                float cost_ = 0.0f;

                bool is_start_node_ = false; 
        };

        struct LesserCostEvaluator
        {
            bool operator()(const std::shared_ptr<SearchNode>& lhs, const std::shared_ptr<SearchNode>& rhs)
            {
                return lhs->cost_ < rhs->cost_;
            }
        };

        GridPathPlanner(std::shared_ptr<OccupancyGrid2D> grid): occupancy_grid_ptr_(grid) {}

        virtual Waypoint getNextWaypoint(Eigen::Vector3d agent_position, 
                                         Eigen::Vector3d goal_point,  
                                         const float goal_radius_tolerance,
                                         const float distance_to_path_tolerance,
                                         float lookup_ahead_pursuit_distance) override
        {
            
            std::lock_guard<std::mutex> paht_lock(path_manipulation_mutex_);
            
            Waypoint waypoint;

            bool path_is_valid = true;

            // First path
            if(isPathEmpty())
            {
                std::cout<<"First path planning"<<std::endl;
                path_is_valid = createNewPath(agent_position, goal_point, goal_radius_tolerance);
                std::cout<<"Post First path planning pruning:"<< pruning_index_<<std::endl;
            }

            if(path_is_valid)
            {
                // Look at 5 trajectory point above to find if the agent progressed on the path and pruned the path based on
                // the closest point. 
                prunePath(agent_position, 5);
                std::cout<<"Purninig index: " <<pruning_index_<<" Path length: "<< current_path_.poses.size() <<std::endl;
                // Do the path needs to be recomputed because the agent is to far from the path.
                Eigen::Vector3d  position_of_pruning_node = poseToVector(current_path_.poses[pruning_index_].pose);
                float current_distance_to_pruned_path = (agent_position - position_of_pruning_node).norm();

                if(distance_to_path_tolerance < current_distance_to_pruned_path)
                {
                    path_is_valid = createNewPath(agent_position, goal_point, goal_radius_tolerance);
                }

                // Compute the waypoint
                if(path_is_valid)
                {
                    position_of_pruning_node = poseToVector(current_path_.poses[pruning_index_].pose);
                    current_distance_to_pruned_path = (agent_position - position_of_pruning_node).norm();
                    
                    
                    int index = pruning_index_;
                    lookup_ahead_pursuit_distance -= current_distance_to_pruned_path;
                    while((0.0f < lookup_ahead_pursuit_distance) || (0 < index) )
                    {
                        lookup_ahead_pursuit_distance -= (poseToVector(current_path_.poses[index].pose) - poseToVector(current_path_.poses[index-1].pose)).norm();
                        index --;
                    }
                    
                    waypoint.position = poseToVector(current_path_.poses[index].pose);
                }
                
            }
            else
            {
                // Return the agent position if the planner fails to produce a plan.
                // TODO add a mechanism to indicate the planner caller that the plan failed.
                waypoint.position = agent_position;
            }
            return waypoint;
        }



    private:
        std::shared_ptr<OccupancyGrid2D> occupancy_grid_ptr_;
        
        std::priority_queue<std::shared_ptr<SearchNode>, std::vector<std::shared_ptr<SearchNode>>, LesserCostEvaluator> processing_node_queue_;
        std::map<unsigned int, std::shared_ptr<SearchNode>> existing_nodes_map_;

        int pruning_index_ = 0 ;

        bool isNodeInGoal(const SearchNode& node);

        unsigned int getUniqueIdOfCoordinates(const OccupancyGrid2D::CellCoordinates coordinates)
        {
            OccupancyGrid2D::CellCoordinates max_coordinates = occupancy_grid_ptr_->getMaxCoordinates();
            
            return coordinates.x + coordinates.y * (max_coordinates.x +1);
        }

        void addChildrenNodesToQueue(std::shared_ptr<SearchNode> current_node,
                                                                        const Eigen::Vector3d goal_position)
        {
            /* To be valid, a children should be an unoccupied cell and should be inside the grid.
             * If the potential children node has already a parent, add it to the current children list only if the expected cost
             * is lower than the one already there. 
             */

            // Eight neighbors coordinates
            std::vector<OccupancyGrid2D::CellCoordinates> potential_children_coordinates_list;
            OccupancyGrid2D::CellCoordinates temp_coordinates;
            temp_coordinates.x = current_node->coordinates_.x+1;
            temp_coordinates.y = current_node->coordinates_.y;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x+1;
            temp_coordinates.y = current_node->coordinates_.y+1;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x;
            temp_coordinates.y = current_node->coordinates_.y+1;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x-1;
            temp_coordinates.y = current_node->coordinates_.y+1;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x-1;
            temp_coordinates.y = current_node->coordinates_.y;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x-1;
            temp_coordinates.y = current_node->coordinates_.y-1;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x;
            temp_coordinates.y = current_node->coordinates_.y-1;
            potential_children_coordinates_list.push_back(temp_coordinates);

            temp_coordinates.x = current_node->coordinates_.x+1;
            temp_coordinates.y = current_node->coordinates_.y-1;
            potential_children_coordinates_list.push_back(temp_coordinates);

            for (auto potential_child_coordinates: potential_children_coordinates_list)
            {
                if (occupancy_grid_ptr_)
                {
                    std::optional<OccupancyGrid2D::CellStatus> child_status = occupancy_grid_ptr_->getCellStatus(potential_child_coordinates);

                    // If child status is an std::nullopt, the coordinates are out of bound of the occupancy grid.
                    // Only consider the coordinates that are free cells.
                    // TODO: Maybe extend to unknown cells.
                    if (child_status.has_value() && (child_status.value() == OccupancyGrid2D::CellStatus::Free))
                    {
                        unsigned int unique_id = getUniqueIdOfCoordinates(potential_child_coordinates);
                        
                        // Verify if the cell was already visited.
                        if(existing_nodes_map_.find(unique_id) != existing_nodes_map_.end())
                        {
                            std::shared_ptr<SearchNode> potential_child_ptr = existing_nodes_map_[unique_id];
                            float cost = computeCostFunction(current_node ,potential_child_ptr, goal_position);
                            
                            // If the new cost is better, replace existant parent and cost.
                            if(cost < potential_child_ptr->cost_)
                            {
                                potential_child_ptr->parent_ = current_node;
                                potential_child_ptr->cost_ = cost;
                                processing_node_queue_.push(potential_child_ptr);
                            }
                        }
                        else // The node was never created, create it.
                        {
                            std::shared_ptr<SearchNode> potential_child_ptr = std::make_shared<SearchNode>(potential_child_coordinates, 
                                                            occupancy_grid_ptr_->getCenterOfCellFromCoordinates(potential_child_coordinates));
                            potential_child_ptr->parent_ = current_node;
                            potential_child_ptr->cost_ = computeCostFunction(current_node ,potential_child_ptr, goal_position);

                            processing_node_queue_.push(potential_child_ptr);
                            existing_nodes_map_.insert({unique_id, potential_child_ptr});
                        }
                    }
                }
            }
        }

        float computeCostFunction(std::shared_ptr<const SearchNode> from_node, std::shared_ptr<const SearchNode> to_node,
                                  const Eigen::Vector3d& goal_point)
        {
            float cost = from_node->cost_; 
            
            // Cost of traveling between the cells
            cost += (to_node->center_of_cell_pos_ - from_node->center_of_cell_pos_).norm();

            // Distance heuristic
            cost += (goal_point - to_node->center_of_cell_pos_).norm();

            return cost;
        }

        Eigen::Vector3d poseToVector(const geometry_msgs::msg::Pose& pose)
        {
            return {pose.position.x, pose.position.y, pose.position.z};
        }

        geometry_msgs::msg::Pose vectorToPose(const Eigen::Vector3d& vector_3d)
        {
            geometry_msgs::msg::Pose pose;
            
            pose.position.x = vector_3d[0];
            pose.position.y = vector_3d[1];
            pose.position.z = vector_3d[2];
            
            return pose;
        }

        int findClosestIndexOnTrajectory(const Eigen::Vector3d& agent_position, int pruning_horizon)
        {
            if(pruning_index_ <= 0)
            {
                // Pruning index is already the closest
                return 0;
            }
            else
            {
                float smallest_distance = (poseToVector(current_path_.poses[pruning_index_].pose) - agent_position).norm();
                int smallest_distance_index = pruning_index_;

                for (int index = pruning_index_-1; (index != -1) && (pruning_index_-pruning_horizon < index); index--)
                {
                    float agent_to_path_node_distance = (poseToVector(current_path_.poses[index].pose) - agent_position).norm();
                    if(agent_to_path_node_distance < smallest_distance)
                    {
                        smallest_distance = agent_to_path_node_distance;
                        smallest_distance_index = index;
                    }
                }
                return smallest_distance_index;
            } 
        }

        void prunePath(const Eigen::Vector3d& agent_position, int pruning_horizon)
        {
            pruning_index_ = findClosestIndexOnTrajectory(agent_position, pruning_horizon);
        }

        bool createNewPath(Eigen::Vector3d starting_point, Eigen::Vector3d goal_point, float goal_radius_tolerance)
        {
            std::optional<nav_msgs::msg::Path> new_path = computePath(starting_point, goal_point, goal_radius_tolerance);
            
            if (new_path.has_value())
            {
                current_path_ = new_path.value();
                if(isPathEmpty())
                {
                    return false;
                }
                pruning_index_ = current_path_.poses.size()-1;
                return true;
            }
            else
            {
                std::cout<<"Failed to generate new path"<<std::endl;
                return false;
            }
        }

        std::optional<nav_msgs::msg::Path> computePath(Eigen::Vector3d starting_point, Eigen::Vector3d goal_point, float goal_radius_tolerance)
        {
            if (occupancy_grid_ptr_)
            {
                bool search_failed = false;

                std::lock_guard<std::mutex> grid_lock(occupancy_grid_ptr_->external_access_mutex_);

                std::cout<<"Starting point"<<std::endl;
                std::optional<OccupancyGrid2D::CellCoordinates> starting_coordinates_opt = 
                                occupancy_grid_ptr_->getCellCoordinatesFromPosition(starting_point);
                
                std::cout<<"Goal point"<<std::endl;
                std::optional<OccupancyGrid2D::CellCoordinates> goal_coordinates_opt = 
                                occupancy_grid_ptr_->getCellCoordinatesFromPosition(goal_point);
                
                std::shared_ptr<SearchNode> last_node = nullptr;

                if(!starting_coordinates_opt.has_value())
                {
                    std::cout<<"Error: The starting point is not in the occupancy grid. Can't perform planning."<<std::endl;
                    OccupancyGrid2D::CellCoordinates max_cells = occupancy_grid_ptr_->getMaxCoordinates();
                    std::cout<<"Size of occupancy grid x:"<<max_cells.x<<  "y: " << max_cells.y<<std::endl;
                    search_failed = true;
                }
                if(!goal_coordinates_opt.has_value())
                {
                    std::cout<<"Error: The goal point is not in the occupancy grid. Can't perform planning."<<std::endl;
                    search_failed = true;
                }
                
                if (!search_failed)
                {
                    OccupancyGrid2D::CellCoordinates starting_coordinates = starting_coordinates_opt.value();
                    OccupancyGrid2D::CellCoordinates goal_coordinates = goal_coordinates_opt.value();

                    std::shared_ptr<SearchNode> starting_node = std::make_shared<SearchNode>(starting_coordinates, 
                                                                occupancy_grid_ptr_->getCenterOfCellFromCoordinates(starting_coordinates));
                    starting_node->cost_ =0.0f;
                    starting_node->is_start_node_ = true;

                    // Add first node in the queue and existing map.
                    processing_node_queue_.push(starting_node);
                    existing_nodes_map_.insert({getUniqueIdOfCoordinates(starting_node->coordinates_), starting_node});
                    
                    const Eigen::Vector3d projected_goal_position = occupancy_grid_ptr_->getCenterOfCellFromCoordinates(goal_coordinates);
                    
                    while(processing_node_queue_.size()!=0)
                    {
                        std::shared_ptr<SearchNode> current_node = processing_node_queue_.top();
                        processing_node_queue_.pop();
                        
                        // Verify if current node is at goal
                        float euclidean_distance_to_goal = (projected_goal_position - current_node->center_of_cell_pos_).norm();
                        if (euclidean_distance_to_goal > goal_radius_tolerance)
                        {
                            // Goal found
                            //std::cout<<"A* found a path"<<std::endl;
                            // Add path generation.
                            last_node = current_node;
                            break;
                        }

                        // Using getCenterOfCellFromCoordinates() to get the goal position to get its project position on the occupancy grid.
                        addChildrenNodesToQueue(current_node, projected_goal_position);
                    }

                }

                if (!search_failed && (last_node != nullptr))
                {
                    // Generate path
                    std::shared_ptr<SearchNode> path_node = last_node;

                    nav_msgs::msg::Path path;
                    if (!path_node->is_start_node_)
                    {
                        while(true)
                        {
                            geometry_msgs::msg::PoseStamped pose_stamped;
                            pose_stamped.pose = vectorToPose(path_node->center_of_cell_pos_);
                            path.poses.push_back(pose_stamped);
                            
                            path_node = path_node->parent_;
                            
                            if(path_node->is_start_node_)
                            {
                                
                                break;
                            }
                            
                        }
                    }

                    return path;
                }
                
            }
            return std::nullopt;
        }

};
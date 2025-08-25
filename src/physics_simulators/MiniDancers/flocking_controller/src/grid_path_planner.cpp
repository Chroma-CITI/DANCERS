#include "grid_path_planner.hpp"

GridPathPlanner::Waypoint GridPathPlanner::getNextWaypoint(Eigen::Vector3d agent_position, 
                                         Eigen::Vector3d goal_position,  
                                         const float goal_radius_tolerance,
                                         const float distance_to_path_tolerance,
                                         float lookup_ahead_pursuit_distance)
{
    
    std::lock_guard<std::mutex> paht_lock(path_manipulation_mutex_);
    
    Waypoint waypoint;

    bool path_is_valid = true;

    // Verify the starting point and goal are in the occupancy grid plane.
    std::optional<OccupancyGrid2D::CellCoordinates> starting_coordinates_opt = 
                        occupancy_grid_ptr_->getCellCoordinatesFromPosition(agent_position);
    std::optional<OccupancyGrid2D::CellCoordinates> goal_coordinates_opt = 
                        occupancy_grid_ptr_->getCellCoordinatesFromPosition(goal_position);
    if (!starting_coordinates_opt.has_value())
    {
        std::cout<<"Planning failed: Starting point is outside of occupancy grid"<<std::endl;
        path_is_valid = false;
    }
    if (!goal_coordinates_opt.has_value())
    {
        std::cout<<"Planning failed: Goal point is outside of occupancy grid"<<std::endl;
        path_is_valid = false;
    }

    // Verify if the starting point and goal points are not in obstacles.
    if(starting_coordinates_opt.has_value() && goal_coordinates_opt.has_value())
    {
        std::optional<OccupancyGrid2D::CellStatus> stating_point_status = occupancy_grid_ptr_->getCellStatus(starting_coordinates_opt.value());
        std::optional<OccupancyGrid2D::CellStatus> goal_position_status = occupancy_grid_ptr_->getCellStatus(goal_coordinates_opt.value());
        
        if(stating_point_status.has_value() && (stating_point_status.value() == OccupancyGrid2D::CellStatus::Occupied))
        {
            std::cout<<"Planning failed: Starting point is not in a free cell"<<std::endl;
            path_is_valid = false;
        }
        if(goal_position_status.has_value() && (goal_position_status.value() == OccupancyGrid2D::CellStatus::Occupied))
        {
            std::cout<<"Planning failed: Goal point is not in a free cell"<<std::endl;
            path_is_valid = false;
        }
    }

    // Projected agent and goal position
    const Eigen::Vector3d  projected_starting_position = occupancy_grid_ptr_->getProjectedVectorOnGridPlan(agent_position);
    const Eigen::Vector3d  projected_goal_position = occupancy_grid_ptr_->getProjectedVectorOnGridPlan(goal_position);

    // First path
    if(isPathEmpty() && path_is_valid)
    {
        std::cout<<"Planning because no plan exist"<<std::endl;
        path_is_valid = createNewPath(projected_starting_position, projected_goal_position, goal_radius_tolerance);
    }

    if(path_is_valid)
    {
        // Look at 5 trajectory point above to find if the agent progressed on the path and pruned the path based on
        // the closest point. 
        prunePath(projected_starting_position, 5);

        // Do the path needs to be recomputed because the agent is to far from the path.
        Eigen::Vector3d  position_of_pruning_node = poseToVector(current_path_.poses[pruning_index_].pose);
        float current_distance_to_pruned_path = (projected_starting_position - position_of_pruning_node).norm();

        if(distance_to_path_tolerance < current_distance_to_pruned_path)
        {
            std::cout<<"Replanning since the agent is too far from trajectory. ("<< current_distance_to_pruned_path <<"m)"<<std::endl;
            path_is_valid = createNewPath(projected_starting_position, projected_goal_position, goal_radius_tolerance);
        }

        // Compute the waypoint
        if(path_is_valid)
        {
            position_of_pruning_node = poseToVector(current_path_.poses[pruning_index_].pose);
            current_distance_to_pruned_path = (projected_starting_position - position_of_pruning_node).norm();
            
            
            int index = pruning_index_;
            lookup_ahead_pursuit_distance -= current_distance_to_pruned_path;
            while((0.0f < lookup_ahead_pursuit_distance) && (0 < index) )
            {
                lookup_ahead_pursuit_distance -= (poseToVector(current_path_.poses[index].pose) - poseToVector(current_path_.poses[index-1].pose)).norm();
                index --;
            }
            
            waypoint.position = poseToVector(current_path_.poses[index].pose);
        }
        
    }

    if (!path_is_valid)
    {
        // Return the agent position if the planner fails to produce a plan.
        // TODO add a mechanism to indicate the planner caller that the plan failed.
        std::cout<<"Planning failed: path planning will give a stationary command"<<std::endl;
        waypoint.position = agent_position;
    }

    current_waypoint_ = waypoint;
    return waypoint;
}

GridPathPlanner::Waypoint GridPathPlanner::getCurrentWaypoint()
{
    std::lock_guard<std::mutex> paht_lock(path_manipulation_mutex_);
    return current_waypoint_; 
}

unsigned int GridPathPlanner::getUniqueIdOfCoordinates(const OccupancyGrid2D::CellCoordinates coordinates)
{
    OccupancyGrid2D::CellCoordinates max_coordinates = occupancy_grid_ptr_->getMaxCoordinates();
    
    return coordinates.x + coordinates.y * (max_coordinates.x +1);
}

void GridPathPlanner::addChildrenNodesToQueue(std::shared_ptr<SearchNode> current_node,
                                              const Eigen::Vector3d goal_position,
                                              std::priority_queue<std::shared_ptr<SearchNode>, std::vector<std::shared_ptr<SearchNode>>, LesserCostEvaluator>& processing_node_queue,
                                              std::unordered_map<unsigned int, std::shared_ptr<SearchNode>>& existing_nodes_map)
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

    if (occupancy_grid_ptr_)
    {
        for (auto potential_child_coordinates: potential_children_coordinates_list)
        {
            std::optional<OccupancyGrid2D::CellStatus> child_status_opt = occupancy_grid_ptr_->getCellStatus(potential_child_coordinates);

            // If child status is an std::nullopt, the coordinates are out of bound of the occupancy grid.
            // Only consider the coordinates that are free cells.
            // TODO: Maybe extend to unknown cells.
            if (child_status_opt.has_value())
            {
                OccupancyGrid2D::CellStatus child_status = child_status_opt.value();

                if(child_status == OccupancyGrid2D::CellStatus::Free || child_status == OccupancyGrid2D::CellStatus::Inflated)
                {
                    unsigned int unique_id = getUniqueIdOfCoordinates(potential_child_coordinates);

                    // Verify if the cell was already visited.
                    if(existing_nodes_map.find(unique_id) != existing_nodes_map.end())
                    {
                        std::shared_ptr<SearchNode> potential_child_ptr = existing_nodes_map[unique_id];
                        float cost = computeCostFunction(current_node ,potential_child_ptr, goal_position, child_status);
                        
                        // If the new cost is better, replace existant parent and cost.
                        if(cost < potential_child_ptr->cost_)
                        {
                            potential_child_ptr->parent_ = current_node;
                            potential_child_ptr->cost_ = cost;
                            processing_node_queue.push(potential_child_ptr);
                        }
                    }
                    else // The node was never created, create it.
                    {
                        std::shared_ptr<SearchNode> potential_child_ptr = std::make_shared<SearchNode>(potential_child_coordinates, 
                                                        occupancy_grid_ptr_->getCenterOfCellFromCoordinates(potential_child_coordinates));
                        potential_child_ptr->parent_ = current_node;
                        potential_child_ptr->cost_ = computeCostFunction(current_node ,potential_child_ptr, goal_position, child_status);

                        processing_node_queue.push(potential_child_ptr);
                        existing_nodes_map.insert({unique_id, potential_child_ptr});
                    }
                }
            }
        }
    }
}

float GridPathPlanner::computeCostFunction(std::shared_ptr<const SearchNode> source_node, std::shared_ptr<const SearchNode> target_node,
                            const Eigen::Vector3d& goal_position, OccupancyGrid2D::CellStatus target_node_status)
{
    float cost = source_node->cost_; 
    
    // Cost of traveling between the cells
    cost += (target_node->center_of_cell_pos_ - source_node->center_of_cell_pos_).norm();

    // Distance heuristic
    cost += (goal_position - target_node->center_of_cell_pos_).norm();

    // Add cost if the status of the cell is inflatec
    cost += (target_node_status == OccupancyGrid2D::CellStatus::Inflated)? 100.0f: 0.0f;

    return cost;
}

Eigen::Vector3d GridPathPlanner::poseToVector(const geometry_msgs::msg::Pose& pose)
{
    return {pose.position.x, pose.position.y, pose.position.z};
}

geometry_msgs::msg::Pose GridPathPlanner::vectorToPose(const Eigen::Vector3d& vector_3d)
{
    geometry_msgs::msg::Pose pose;
    
    pose.position.x = vector_3d[0];
    pose.position.y = vector_3d[1];
    pose.position.z = vector_3d[2];
    
    return pose;
}

int GridPathPlanner::findClosestIndexOnTrajectory(const Eigen::Vector3d& agent_position, int pruning_horizon)
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

void GridPathPlanner::prunePath(const Eigen::Vector3d& agent_position, int pruning_horizon)
{
    pruning_index_ = findClosestIndexOnTrajectory(agent_position, pruning_horizon);
}

bool GridPathPlanner::createNewPath(Eigen::Vector3d starting_point, Eigen::Vector3d goal_position, float goal_radius_tolerance)
{
    std::optional<nav_msgs::msg::Path> new_path = computePath(starting_point, goal_position, goal_radius_tolerance);
    
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

std::optional<nav_msgs::msg::Path> GridPathPlanner::computePath(Eigen::Vector3d starting_point, Eigen::Vector3d goal_position, float goal_radius_tolerance)
{
    std::priority_queue<std::shared_ptr<SearchNode>, std::vector<std::shared_ptr<SearchNode>>, LesserCostEvaluator> processing_node_queue;
    std::unordered_map<unsigned int, std::shared_ptr<SearchNode>> existing_nodes_map;

    if (occupancy_grid_ptr_)
    {
        bool search_failed = false;

        std::lock_guard<std::mutex> grid_lock(occupancy_grid_ptr_->external_access_mutex_);

        std::optional<OccupancyGrid2D::CellCoordinates> starting_coordinates_opt = 
                        occupancy_grid_ptr_->getCellCoordinatesFromPosition(starting_point);
        
        std::optional<OccupancyGrid2D::CellCoordinates> goal_coordinates_opt = 
                        occupancy_grid_ptr_->getCellCoordinatesFromPosition(goal_position);
        
        std::shared_ptr<SearchNode> last_node = nullptr;

        if(!starting_coordinates_opt.has_value())
        {
            std::cout<<"Error: The starting point is not in the occupancy grid. Can't perform planning."<<std::endl;
            OccupancyGrid2D::CellCoordinates max_cells = occupancy_grid_ptr_->getMaxCoordinates();
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
            processing_node_queue.push(starting_node);
            existing_nodes_map.insert({getUniqueIdOfCoordinates(starting_node->coordinates_), starting_node});
            
            const Eigen::Vector3d projected_goal_position = occupancy_grid_ptr_->getCenterOfCellFromCoordinates(goal_coordinates);
            while(processing_node_queue.size()!=0)
            {
                std::shared_ptr<SearchNode> current_node = processing_node_queue.top();
                processing_node_queue.pop();
                
                // Verify if current node is at goal
                float euclidean_distance_to_goal = (projected_goal_position - current_node->center_of_cell_pos_).norm();
                if (euclidean_distance_to_goal < goal_radius_tolerance)
                {
                    // Goal found.
                    last_node = current_node;
                    break;
                }

                // Using getCenterOfCellFromCoordinates() to get the goal position to get its project position on the occupancy grid.
                addChildrenNodesToQueue(current_node, projected_goal_position, processing_node_queue, existing_nodes_map);
            }
            if(last_node == nullptr)
            {
                std::cout<<"Planning failed: A* explored the whole space without finding path."<<std::endl;
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
#pragma once

#include <iostream>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "occupancy_grid.hpp"
#include "path_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Core>

/**
 * @brief Path planner that uses an occupancy grid as a configuration space and A* to compute a path.
 */
class GridPathPlanner: public PathPlanner
{
    public:

        /**
         * @brief Searchable node for a path planning algorithm that points towards a 
         * corresponding cell in a given occupancy grid through coordinates.
         */
        class SearchNode
        {
            public:
                /**
                 * @brief Construct a node that points towards an occupancy grid cell through coordinates.
                 * @param coordinates The coordinates in the occupancy grid.
                 * @param center_of_cell_pos Center in meter of the cell in the occupancy grid. Used by cost functions of the search algorith.
                 */
                inline SearchNode(const OccupancyGrid2D::CellCoordinates& coordinates,const Eigen::Vector3d& center_of_cell_pos): 
                          coordinates_(coordinates), center_of_cell_pos_(center_of_cell_pos) {}


                /**
                 * @brief Coordinates of a cell in an occupancy grid that this class represents for the search algorithm.
                 */
                OccupancyGrid2D::CellCoordinates coordinates_;
                
                /**
                 * @brief Position in meters in the global frame of the center of the grid cell that this class represents.
                 * It's used to compute the distance between nodes to be used as costs.
                 */
                Eigen::Vector3d center_of_cell_pos_;

                /**
                 * @brief Nieghbor nodes that represent the direction to go to find a path between a goal and starting point. 
                 */
                std::shared_ptr<SearchNode> parent_;
                
                /**
                 * @brief Cost of getting to this node computed by a search algorithm.
                 */
                float cost_ = 0.0f;

                /**
                 * @brief Indicates if this node is the start node of the search algorithm. 
                 * By definition, it doesn't have a parent.
                 */
                bool is_start_node_ = false; 
        };

        /**
         * @brief Compare functor used for the priority queue to get the behavior 
         * where the lowest cost node will be processed first.
         */
        struct LesserCostEvaluator
        {
            inline bool operator()(const std::shared_ptr<SearchNode>& lhs, const std::shared_ptr<SearchNode>& rhs)
            {
                return lhs->cost_ > rhs->cost_;
            }
        };

        /**
         * @brief The grid path planner that takes a pointer to an occupancy grid in which the planning will be done.
         */
        GridPathPlanner(std::shared_ptr<OccupancyGrid2D> grid): occupancy_grid_ptr_(grid) {}

        /**
         * @brief Computes the next waypoint that a agent should take based on a global A* planner in the given occupancy grid.
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
                                         float lookup_ahead_pursuit_distance) override;

        /**
         * @brief Returns the last computed waypoint.
         * @return The last computed waypoint.
         */
        virtual Waypoint getCurrentWaypoint() override;

    private:
        /**
         * @brief Occupancy grid from which the path will be computed.
         */
        std::shared_ptr<OccupancyGrid2D> occupancy_grid_ptr_;

        /**
         * @brief Index indicating the progress of the agent on the trajectory. 
         * Since the path is stored backwards, a high value means the agent is 
         * at the begining of the trajectory.
         */
        int pruning_index_ = 0 ;

        /**
         * @brief Stores the last computed waypoint.
         */
        Waypoint current_waypoint_;

        /**
         * @brief Computes an identifier that is unique for each OccupancyGrid2D::CellCoordinates.
         * It used as a hash to stored the visited nodes.
         */
        unsigned int getUniqueIdOfCoordinates(const OccupancyGrid2D::CellCoordinates coordinates);

        /**
         * @brief Methods that adds to a given priority queue and map the 8-neighbors cells of the current cell in the occupancy grid
         * if they are free cells (no in obstacles) and if they are inside the bounds of the occupancy grid.
         * @param current_node Current node from which the neighbors will be extracted from.
         * @param goal_position Goal position to compute the heuristic cost function.
         * @param processing_node_queue Reference to a priority queue that stores the nodes based on their cost and that will be 
         * processed. The neighbors node will be added in this queue if they are valid.
         * @param existing_nodes_map Reference to a map used to keep track of visited node. It's used to know if the neighbors 
         * already exist and changed their cost and parents if the current node offers a lower cost. 
         * Neighbors are added in the map if they aren't already there.
         */
        void addChildrenNodesToQueue(std::shared_ptr<SearchNode> current_node,
                                     const Eigen::Vector3d goal_position,
                                     std::priority_queue<std::shared_ptr<SearchNode>, std::vector<std::shared_ptr<SearchNode>>, LesserCostEvaluator>& processing_node_queue,
                                     std::unordered_map<unsigned int, std::shared_ptr<SearchNode>>& existing_nodes_map);

        /**
         * @brief Computes the cost of going from a source node towards a target node. The cost of the target is the cost 
         * of the source node + the distance between the center of the source node cell and the target cell + an heuristic which
         * is the euclidean distance between the target node and the goal.
         * @param source_node Node to compute the cost from.
         * @param target_node Node for which the cost is computed for.
         * @param goal_position Position of the goal position to compute the heuristic.
         * @return The cost of going to the the target_node from the source node.
         */
        float computeCostFunction(std::shared_ptr<const SearchNode> source_node, std::shared_ptr<const SearchNode> target_node,
                                  const Eigen::Vector3d& goal_position);

        /**
         * @brief Transforms a Eigen::Vector3d to a ROS geometry pose message.
         * @param pose Pose to transform in a 3D vector.
         * @return The vector corresponding to the given pose.
         */
        Eigen::Vector3d poseToVector(const geometry_msgs::msg::Pose& pose);

        /**
         * @brief Transforms a ROS geometry pose message to Eigen::Vector3d.
         * @param vector_3d The 3D vector to transform in a Pose.
         * @return The ROS Pose message corresponding to the given 3D vector.
         */
        geometry_msgs::msg::Pose vectorToPose(const Eigen::Vector3d& vector_3d);

        /**
         * @brief Finds the index of the path corresponding to the agent's closest point on the trajectory.
         * @param pruning_horizon Number of index from the current pruning_index_ to search for the closest point.
         * @return The indexes of the closest trajectory grid cell from the agent in the given horizon.
         */
        int findClosestIndexOnTrajectory(const Eigen::Vector3d& agent_position, int pruning_horizon);
    
        /**
         * @brief Updates the pruning_index_ to progress along the path.
         * @param agent_position The current agent position.
         * @param pruning_horizon Number of indexes from the current pruning_index_ to search for the closest point.
         */
        void prunePath(const Eigen::Vector3d& agent_position, int pruning_horizon);

        /**
         * @brief Replace the old path for a new path starting at the starting_point and finishing at the goal_position if possible.
         * Also resets the pruning_index to the start of the path. 
         * The path will failed to be computed if there is no possible path between the start and finish point.
         * @param starting_point Point from which to start the path.
         * @param goal_position Point where the path should lead to.
         * @param goal_radius_tolerance Radius around the goal in which cells will be considered as
         *  a valid end point for the path planning algorithm.
         */
        bool createNewPath(Eigen::Vector3d starting_point, Eigen::Vector3d goal_position, float goal_radius_tolerance);

        /**
         * @brief Computes a path in the provided internal occupancy from the starting_point to the goal_position using A*.
         * @param starting_point Point from which to start the path.
         * @param goal_position Point where the path should lead to.
         * @param goal_radius_tolerance Radius around the goal in which cells will be considered as
         *  a valid end point for the path planning algorithm.
         * @return The compute path in the form of the ROS Path message which is a series of Poses.
         */
        std::optional<nav_msgs::msg::Path> computePath(Eigen::Vector3d starting_point, Eigen::Vector3d goal_position, float goal_radius_tolerance);
};
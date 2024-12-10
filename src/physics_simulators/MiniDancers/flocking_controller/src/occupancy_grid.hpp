#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include <flocking_controller/cuboid_obstacle_util.hpp>

class OccupancyGrid2D
{
    public:
        enum class CellStatus
        {
            UNKNOWN=0,
            FREE,
            OCCUPIED
        };

        enum class Axis
        {
            X=0,
            Y
        };

        struct Cell_t
        {
            CellStatus status = CellStatus::UNKNOWN;
        };

        /**
         * @brief Constructor that builds an occupancy grid at the origin expanding to the opposite_corner where all cells are unknown.
         * @param origin Position in the global frame corresponding to the (0,0) of the occupancy grid. 
         * The z value of the origin is used as to know the heigh of the map. 
         * @param opposite_corner The x and y value are used to know to where the map should extend to. 
         * Its x and y component should be greater than the x and y origin.
         * @param init_cell_status Initial status of all the cells in the occupancy grid. 
         * @param cell_side_size Is used to specified the size of the cells. Is related to the resolution.
         */
        OccupancyGrid2D(const Eigen::Vector3d& origin,const Eigen::Vector3d& opposite_corner, const float cell_side_size, CellStatus init_cell_status=CellStatus::UNKNOWN);

    private:

        /**
         * @brief Size in meter of the side of a cell.
         */
        float cell_side_size_;

        /**
         * @brief Origin of cell grid in the global frame. This origin position correspond to the (0,0) of the cell grid.
         */
        Eigen::Vector3d map_origin_;

        /**
         * @brief 2D Occupancy grid where the first index is the x axis and the second is the y axis.
         */
        std::vector<std::vector<Cell_t>> grid_;

        /**
         * @brief Get the index in the occupancy grid of the distance in the given axis.
         * @param distance The distance to get it's corresponding index from.
         * @param axis The axis in which the distance is referenced in.
         * @return Return the index in the right axis or returns an std::nullopt if 
         * the distance is out of bounds of the occupancy grid. 
         */
        std::optional<int> getIndexFromDistance(const float distance, Axis axis);
        
        /**
         * @brief Set the status of the cells in the occupancy grid based on the given obstacles.
         * @param obstacles List of obstacles with their position in the global frame.  
         */
        void populateGridFromObstacles(std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles);

        /**
         * @brief Verify if the obstacle crosses the height of the occupancy grid in the global frame.
         * @param obstacle Obstacle with it's position defined in the global frame.
         * @return Returns true if the obstacle crosses the plane and false if it doesn't.
         */
        bool doObstacleCrossGridPlane(const cuboid::obstacle_t& obstacle);
};
#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include <flocking_controller/cuboid_obstacle_util.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * @brief Simple 2D Occupancy grid aligned in the global frame that can be populated with cuboid obstacles.
 */
class OccupancyGrid2D
{
    public:
        enum class CellStatus
        {
            Unknown=0,
            Free,
            Occupied
        };

        enum class Axis
        {
            X=0,
            Y
        };

        struct Cell_t
        {
            CellStatus status = CellStatus::Unknown;
        };

        struct CellCoordinates
        {
            int x=0;
            int y =0;
        };

        /**
         * @brief Constructor that builds an occupancy grid at the origin expanding to the opposite_corner where all cells are Unknown.
         * @param origin Position in the global frame corresponding to the (0,0) of the occupancy grid. 
         * The z value of the origin is used as to know the heigh of the map. 
         * @param opposite_corner The x and y value are used to know to where the map should extend to. 
         * Its x and y component should be greater than the x and y origin.
         * @param obstacle_inflation Extra size to add on each side of obstacles to virtually inflate them for safe navigation.
         * @param init_cell_status Initial status of all the cells in the occupancy grid. 
         * @param cell_side_size Is used to specified the size of the cells. Is related to the resolution.
         */
        OccupancyGrid2D(const Eigen::Vector3d& origin, const Eigen::Vector3d& opposite_corner, const float cell_side_size, const float obstacle_inflation = 0.0f, CellStatus init_cell_status=CellStatus::Unknown);

        /**
         * @brief Returns the maximum x and y coordinatees of the occupancy grid.
         * @return Coordinates.
         */
        CellCoordinates getMaxCoordinates();

        /**
         * @brief Returns a copy of the internal occupancy grid
         * @return Occupancy grid.
         */
        std::vector<std::vector<Cell_t>> getGrid();

        /**
         * @brief Get the occupancy grid formated in a nav_msgs::msg::OccupancyGrid message 
         * without the ::info::load_time and the ::header::stamp set.
         * @brief The occupancy grid ROS message.
         */
        nav_msgs::msg::OccupancyGrid getOccupancyGridMsg();

        /**
         * @brief Set the status of the cells in the occupancy grid based on the given obstacles.
         * @param obstacles List of obstacles with their position in the global frame.  
         */
        void populateGridFromObstacles(std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles);

        /**
         * @brief Return the occupancy grid coordinates of a given projeted point on the occupancy grid plane if possible.
         * @param point The point in the global frame.
         * @return The corresponding coordinates of the point in the occupancy grid. 
         * Return an std::nullopt if the point porjected on the grid's plan is out of the occupancy grid.
         */
        std::optional<CellCoordinates> getCellCoordinatesFromPosition(const Eigen::Vector3d& point);

        /**getCellStatus
         * @brief Return the center of the cell in the global frame in meters.
         * @param coordinates The coordinates in the occupancy grid.
         * @return The corresponding euclidean position of the coordinates on the grid plane in the global frame.
         */
        Eigen::Vector3d getCenterOfCellFromCoordinates(const CellCoordinates& coordinates);

        /**
         * @brief Return the status of the cell at the specified coordinates.
         * @param coordinates The coordinates in the occupancy grid.
         * @return Return the status of the cell or an std::nullopt if the cell doesn't exist
         */
        std::optional<CellStatus> getCellStatus(const CellCoordinates& coordinates);

        /**
         * @brief Mutex that can be used by external processes to protect access to the whole object.  
         */
        std::mutex external_access_mutex_;

    private:

        /**
         * @brief Size in meter of the side of a cell.
         */
        float cell_side_size_;

        /**
         * @brief Size added to each side of obstacles to virtually inflate them for safe navigation.
         */
        float obstacle_inflation_;

        /**
         * @brief Origin of cell grid in the global frame. This origin position correspond to the (0,0) of the cell grid.
         */
        Eigen::Vector3d map_origin_;

        /**
         * @brief 2D Occupancy grid where the first index is the x axis and the second is the y axis.
         */
        std::vector<std::vector<Cell_t>> grid_;

        /**
         * @brief Mutex to access the grid.
         */
        std::mutex grid_access_mutex_;

        /**
         * @brief Get the index in the occupancy grid of the distance in the given axis.
         * @param distance The distance to get it's corresponding index from.
         * @param axis The axis in which the distance is referenced in.
         * @return Return the index in the right axis or returns an std::nullopt if 
         * the distance is out of bounds of the occupancy grid. 
         */
        std::optional<int> getIndexFromDistance(const float distance, Axis axis);

        /**
         * @brief Get the index in the occupancy grid of the distance in the given axis 
         * or its closest index if the distance is out of bounds.
         * @param distance The distance to get it's corresponding index from.
         * @param axis The axis in which the distance is referenced in.
         * @return Return the index corresponding to the distance or the closest index if its out of bounds.
         */
        int getClosestIndexFromDistance(const float distance, Axis axis);

        /**
         * @brief Verify if the obstacle crosses the height of the occupancy grid in the global frame.
         * @param obstacle Obstacle with it's position defined in the global frame.
         * @return Returns true if the obstacle crosses the plane and false if it doesn't.
         */
        bool doObstacleCrossGridPlane(const cuboid::obstacle_t& obstacle);

        /**
         * @brief Verify if the obstacle's projection on the XY plane of the grid overlaps the area of the grid.
         * @param obstacle Obstacle with it's position defined in the global frame.
         * @return Returns true if the obstacle projection on XY plane overlaps the area of the grid.
         */
        bool isObstacleInGridPlane(const cuboid::obstacle_t& obstacle);
};
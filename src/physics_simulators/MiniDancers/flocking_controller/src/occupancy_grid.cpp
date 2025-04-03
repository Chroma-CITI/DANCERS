#include "occupancy_grid.hpp"

#include <cassert>
#include <iostream>

OccupancyGrid2D::OccupancyGrid2D(const Eigen::Vector3d& origin,
                  const Eigen::Vector3d& opposite_corner, 
                  const float cell_side_size,
                  const float obstacle_inflation,
                  CellStatus init_cell_status): 
                  map_origin_(origin), cell_side_size_(cell_side_size),
                  obstacle_inflation_(obstacle_inflation)

{
    assert(0.0f < cell_side_size_ && "Cell size is either 0.0 or negative");

    Eigen::Vector3d origin_to_opposite_corner = opposite_corner - origin;
    
    float map_size_x = origin_to_opposite_corner[0];
    float map_size_y = origin_to_opposite_corner[1];

    assert((map_size_x > 0.0f) && (map_size_y > 0.0f));

    int cell_count_x = static_cast<int>(map_size_x/cell_side_size_);
    int cell_count_y = static_cast<int>(map_size_y/cell_side_size_);

    Cell_t init_cell = Cell_t({.status=init_cell_status});

    grid_ = std::vector<std::vector<Cell_t>>(cell_count_x, std::vector<Cell_t>(cell_count_y, init_cell));
}

std::optional<int> OccupancyGrid2D::getIndexFromDistance(const float distance, Axis axis)
{
    int cell_index = static_cast<int>(distance/cell_side_size_);

    if (cell_index < 0)
    {
        return std::nullopt;
    }

    int axis_max_cell = 0;
    
    if (axis == Axis::X)
    {
        axis_max_cell = grid_.size()-1;
    }
    else if (axis == Axis::Y)
    {
        if (grid_.size() != 0)
        {
            axis_max_cell = grid_[0].size()-1;
        }
        else
        {
            return std::nullopt;
        }
    }
    else
    {
        // Should not happen
        assert(false && "Unknown axis in the get index function");
    }

    if (axis_max_cell < cell_index)
    {
        return std::nullopt;
    }

    return cell_index;
}

int OccupancyGrid2D::getClosestIndexFromDistance(const float distance, Axis axis)
{
    if (distance < 0.0f)
    {
        return 0;
    }

    std::optional index_opt = getIndexFromDistance(distance, axis);

    if(!index_opt.has_value())
    {
        if (axis == Axis::X)
        {
            return grid_.size();
        }
        else if (axis == Axis::Y)
        {
            if (grid_.size() != 0)
            {
                return grid_[0].size();
            }
            else
            {
                return 0;
            }
        }
    }

   return index_opt.value();
}

std::optional<OccupancyGrid2D::CellCoordinates> OccupancyGrid2D::getCellCoordinatesFromPosition(const Eigen::Vector3d& point)
{
    Eigen::Vector3d point_in_grid_frame = point - map_origin_;
    
    std::optional<int> x_index_opt = getIndexFromDistance(point_in_grid_frame[0], Axis::X);
    std::optional<int> y_index_opt = getIndexFromDistance(point_in_grid_frame[1], Axis::Y);
    
    // Is the point wihtin the existing x and y limits of the grid.
    if (x_index_opt.has_value() && y_index_opt.has_value())
    {
        CellCoordinates coordinates = {.x = x_index_opt.value(), .y = y_index_opt.value()};
        return coordinates;
    }
    return std::nullopt;
}

void OccupancyGrid2D::populateGridFromObstacles(std::shared_ptr<std::vector<cuboid::obstacle_t>> obstacles)
{
    if (obstacles)
    {
        for (cuboid::obstacle_t& obstacle: *obstacles)
        {
            if(doObstacleCrossGridPlane(obstacle) && isObstacleInGridPlane(obstacle))
            {
                Eigen::Vector3d obstacle_center_in_grid_frame = obstacle.center-map_origin_;

                int min_x = getClosestIndexFromDistance(obstacle_center_in_grid_frame[0]-obstacle.size_x/2, Axis::X);

                int min_x_obstacles = getClosestIndexFromDistance(obstacle_center_in_grid_frame[0]-obstacle.size_x/2 - obstacle_inflation_,
                                                        Axis::X);
                
                int max_x = getClosestIndexFromDistance(obstacle_center_in_grid_frame[0]+obstacle.size_x/2, Axis::X);

                int max_x_obstacles = getClosestIndexFromDistance(obstacle_center_in_grid_frame[0]+obstacle.size_x/2 + obstacle_inflation_,
                                                        Axis::X);
                
                int min_y = getClosestIndexFromDistance(obstacle_center_in_grid_frame[1]-obstacle.size_y/2, Axis::Y);

                int min_y_obstacles = getClosestIndexFromDistance(obstacle_center_in_grid_frame[1]-obstacle.size_y/2 - obstacle_inflation_,
                                                        Axis::Y);   

                int max_y = getClosestIndexFromDistance(obstacle_center_in_grid_frame[1]+obstacle.size_y/2, Axis::Y);

                int max_y_obstacles = getClosestIndexFromDistance(obstacle_center_in_grid_frame[1]+obstacle.size_y/2 + obstacle_inflation_,
                                                        Axis::Y);

                for(int x_index = min_x; x_index < max_x; x_index++)
                {
                    for(int y_index = min_y; y_index < max_y; y_index++)
                    {
                        const std::lock_guard<std::mutex> lock(grid_access_mutex_);
                        grid_[x_index][y_index].status = CellStatus::Occupied;
                    }
                }

                for (int x_index = min_x_obstacles; x_index < max_x_obstacles; x_index++)
                {
                    for(int y_index = min_y_obstacles; y_index < max_y_obstacles; y_index++)
                    {
                        const std::lock_guard<std::mutex> lock(grid_access_mutex_);
                        if(grid_[x_index][y_index].status != CellStatus::Occupied)
                        {
                            grid_[x_index][y_index].status = CellStatus::Inflated;
                        }
                    }
                }
            }
        }
    }
}

Eigen::Vector3d OccupancyGrid2D::getCenterOfCellFromCoordinates(const CellCoordinates& coordinates)
{
    Eigen::Vector3d center_of_cell;
    // Compute the euclidean position in the grid frame.
    center_of_cell[0] = coordinates.x*cell_side_size_+ cell_side_size_/2;
    center_of_cell[1] = coordinates.y*cell_side_size_+ cell_side_size_/2;

    // Add the map origin to get the position in the global frame.
    return center_of_cell + map_origin_;
}

bool OccupancyGrid2D::doObstacleCrossGridPlane(const cuboid::obstacle_t& obstacle)
{
    // If the height of the occupancy grid is in between the bottom and top part of the obstacle, the obstacle is in the grid.
    if( ((obstacle.center[2] - obstacle.size_z/2 - obstacle_inflation_) <= map_origin_[2]) && (map_origin_[2] <= (obstacle.center[2] + obstacle.size_z/2 + obstacle_inflation_)))
    {
        return true;
    }

    return false;
}

bool OccupancyGrid2D::isObstacleInGridPlane(const cuboid::obstacle_t& obstacle)
{
    if (grid_.size() == 0)
    {
        return false;
    }
    float map_max_x = grid_.size()*cell_side_size_;
    float map_max_y = grid_[0].size()*cell_side_size_;

    Eigen::Vector3d obstacle_center_in_grid_frame = obstacle.center - map_origin_;

    float obst_min_x = obstacle_center_in_grid_frame[0] - obstacle.size_x/2- obstacle_inflation_;
    float obst_max_x = obstacle_center_in_grid_frame[0] + obstacle.size_x/2+ obstacle_inflation_;

    float obst_min_y = obstacle_center_in_grid_frame[1] - obstacle.size_y/2 - obstacle_inflation_;
    float obst_max_y = obstacle_center_in_grid_frame[1] + obstacle.size_y/2 + obstacle_inflation_;

    // Is the obstacle overlaping the x part of the grid
    if (!((obst_max_x < 0.0f) || (map_max_x < obst_min_x)))
    { 
        // Is the obstacle overlaping the y part of the grid
        if(!((obst_max_y < 0.0f) || (map_max_y < obst_min_y)))
        {
            return true;
        }
    }
    return false;
}

std::vector<std::vector<OccupancyGrid2D::Cell_t>> OccupancyGrid2D::getGrid()
{
    const std::lock_guard<std::mutex> lock(grid_access_mutex_);

    return grid_;
}

nav_msgs::msg::OccupancyGrid OccupancyGrid2D::getOccupancyGridMsg()
{
    const std::lock_guard<std::mutex> lock(grid_access_mutex_);
    
    nav_msgs::msg::OccupancyGrid grid_msg;
    
    // Header
    grid_msg.header.frame_id = "map";

    //Meta information
    grid_msg.info.resolution = cell_side_size_;
    grid_msg.info.width =  grid_.size();
    grid_msg.info.height = grid_[0].size();
    
    geometry_msgs::msg::Pose origin;
    grid_msg.info.origin.position.x = map_origin_[0];
    grid_msg.info.origin.position.y = map_origin_[1];
    grid_msg.info.origin.position.z = map_origin_[2];

    // Encode the grid
    for(int y_index = 0; y_index < grid_msg.info.height; y_index++)
    {
        for(int x_index = 0; x_index < grid_msg.info.width; x_index++)
        {
            //[x_index + grid_msg.width*y_index]
            int cell_value=0;
            if (grid_[x_index][y_index].status == CellStatus::Free)
            {
                grid_msg.data.emplace_back(0);
            }
            else if(grid_[x_index][y_index].status == CellStatus::Occupied)
            {
                grid_msg.data.emplace_back(1);
            }
            else if(grid_[x_index][y_index].status == CellStatus::Inflated)
            {
                grid_msg.data.emplace_back(100);
            }
            else
            {
                grid_msg.data.emplace_back(-1);
            }
        }
    }
    return grid_msg;
}

OccupancyGrid2D::CellCoordinates OccupancyGrid2D::getMaxCoordinates()
{
    CellCoordinates coordinates;
    coordinates.x = grid_.size()-1;

    if (grid_.size() == 0)
    {
        coordinates.y = 0;
    }
    else
    {
        coordinates.y = grid_[0].size()-1;
    }
    return coordinates;
}

std::optional<OccupancyGrid2D::CellStatus> OccupancyGrid2D::getCellStatus(const CellCoordinates& coordinates)
{
    if((coordinates.x < 0) || ( grid_.size()-1 < coordinates.x) ||
       (coordinates.y < 0) || ( grid_[0].size()-1 < coordinates.y))
    {
        return std::nullopt;
    }
    else
    {
        return grid_[coordinates.x][coordinates.y].status;
    }
       
}

Eigen::Vector3d OccupancyGrid2D::getProjectedVectorOnGridPlan(Eigen::Vector3d vector_in_global_frame)
{
    vector_in_global_frame[2] = map_origin_[2];
    return vector_in_global_frame;
}
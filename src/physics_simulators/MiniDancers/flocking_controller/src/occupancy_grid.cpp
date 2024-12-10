#include "occupancy_grid.hpp"

#include <cassert>

OccupancyGrid2D::OccupancyGrid2D(const Eigen::Vector3d& origin,
                  const Eigen::Vector3d& opposite_corner, 
                  const float cell_side_size,
                  CellStatus init_cell_status): 
                  map_origin_(origin), cell_side_size_(cell_side_size)

{
    assert(cell_side_size_ <= 0.0f && "Cell size is either 0.0 or negative");

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
    assert(distance<0.0f);

    int cell_index = static_cast<int>(distance/cell_side_size_);

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
        return std::nullopt;
    }

    if (axis_max_cell < cell_index)
    {
        return std::nullopt;
    }
    return cell_index;
}


#ifndef OBsss_HPP
#define OBsss_HPP
#include <Eigen/Core>
#include <vector>

/**
 * @struct Obstacle structure
 * 
 * Obstacles are only cuboïds defined by their center and their sizes along each axis
 */
struct obstacle_t
{
    int id;
    Eigen::Vector3d center;
    double size_x, size_y, size_z;
};

/**
 * @struct Triangle structure
 *
 * Triangles are defined by their 3 vertices
 */
struct triangle
{
    Eigen::Vector3d p1, p2, p3;
};

/**
 * @brief Convert an obstacle to a list of triangles
 *
 * @param obst Obstacle to convert
 * @return std::vector<triangle> List of triangles
 */
std::vector<triangle> obstacle_to_triangles_list(obstacle_t obst)
{
    std::vector<triangle> triangles{};
    
    // Half sizes along each axis
    double half_x = obst.size_x / 2.0f;
    double half_y = obst.size_y / 2.0f;
    double half_z = obst.size_z / 2.0f;

    // Define the 8 vertices of the cuboid
    Eigen::Vector3d vertices[8] = {
        {obst.center.x() - half_x, obst.center.y() - half_y, obst.center.z() - half_z}, // V0 (bottom-left-front)
        {obst.center.x() + half_x, obst.center.y() - half_y, obst.center.z() - half_z}, // V1 (bottom-right-front)
        {obst.center.x() + half_x, obst.center.y() + half_y, obst.center.z() - half_z}, // V2 (top-right-front)
        {obst.center.x() - half_x, obst.center.y() + half_y, obst.center.z() - half_z}, // V3 (top-left-front)
        {obst.center.x() - half_x, obst.center.y() - half_y, obst.center.z() + half_z}, // V4 (bottom-left-back)
        {obst.center.x() + half_x, obst.center.y() - half_y, obst.center.z() + half_z}, // V5 (bottom-right-back)
        {obst.center.x() + half_x, obst.center.y() + half_y, obst.center.z() + half_z}, // V6 (top-right-back)
        {obst.center.x() - half_x, obst.center.y() + half_y, obst.center.z() + half_z}  // V7 (top-left-back)
    };

    // Define the 12 triangles (2 per face of the cuboid)
    // Front face (V0, V1, V2, V3)
    triangles.push_back({vertices[0], vertices[1], vertices[2]});
    triangles.push_back({vertices[0], vertices[2], vertices[3]});

    // Back face (V4, V5, V6, V7)
    triangles.push_back({vertices[4], vertices[6], vertices[5]});
    triangles.push_back({vertices[4], vertices[7], vertices[6]});

    // Left face (V0, V3, V7, V4)
    triangles.push_back({vertices[0], vertices[3], vertices[7]});
    triangles.push_back({vertices[0], vertices[7], vertices[4]});

    // Right face (V1, V5, V6, V2)
    triangles.push_back({vertices[1], vertices[6], vertices[5]});
    triangles.push_back({vertices[1], vertices[2], vertices[6]});

    // Top face (V3, V2, V6, V7)
    triangles.push_back({vertices[3], vertices[2], vertices[6]});
    triangles.push_back({vertices[3], vertices[6], vertices[7]});

    // Bottom face (V0, V4, V5, V1)
    triangles.push_back({vertices[0], vertices[5], vertices[4]});
    triangles.push_back({vertices[0], vertices[1], vertices[5]});
    return triangles;
}

/**
 * @brief Get the nearest point from an obstacle
 *
 * @param point The point to check
 * @param obst The obstacle
 * @return Eigen::Vector3d The nearest point from the obstacle
 */
Eigen::Vector3d GetNearestPointFromObstacle(Eigen::Vector3d point, obstacle_t obst)
{
    double half_x = obst.size_x / 2.0f;
    double half_y = obst.size_y / 2.0f;
    double half_z = obst.size_z / 2.0f;

    Eigen::Vector3d nearest_point(
        std::clamp(point.x(), obst.center.x() - half_x, obst.center.x() + half_x),
        std::clamp(point.y(), obst.center.y() - half_y, obst.center.y() + half_y),
        std::clamp(point.z(), obst.center.z() - half_z, obst.center.z() + half_z));

    return nearest_point;
}

#endif 
#pragma once
#include <Eigen/Core>
#include <vector>

namespace cuboid{
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
    struct triangle_t
    {
        Eigen::Vector3d p1, p2, p3;
    };

    /**
    * @brief Convert an obstacle to a list of triangles
    *
    * @param obst Obstacle to convert
    * @return std::vector<triangle> List of triangles
    */
    inline std::vector<triangle_t> obstacle_to_triangles_list(obstacle_t obst)
    {
        std::vector<triangle_t> triangles{};
        
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
    inline Eigen::Vector3d get_nearest_point_from_obstacle(const Eigen::Vector3d& point, const obstacle_t& obst)
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

    // Check whether the segment [p0, p1] intersects the AABB using the slab method.
    inline bool segment_cuboid_intersects(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const obstacle_t &box) {
        float tmin = 0.0f, tmax = 1.0f;
        Eigen::Vector3d min = Eigen::Vector3d(box.center.x() - box.size_x / 2.0f, box.center.y() - box.size_y / 2.0f, box.center.z() - box.size_z / 2.0f);
        Eigen::Vector3d max = Eigen::Vector3d(box.center.x() + box.size_x / 2.0f, box.center.y() + box.size_y / 2.0f, box.center.z() + box.size_z / 2.0f);
        Eigen::Vector3d d = p1 - p0;
        for (int i = 0; i < 3; i++) {
            // For each coordinate axis
            float p = (i == 0 ? p0.x() : (i == 1 ? p0.y() : p0.z()));
            float di = (i == 0 ? d.x() : (i == 1 ? d.y() : d.z()));
            float bmin = (i == 0 ? min.x() : (i == 1 ? min.y() : min.z()));
            float bmax = (i == 0 ? max.x() : (i == 1 ? max.y() : max.z()));
            
            if (std::abs(di) < 1e-6f) {
                // The segment is nearly parallel to this axis – if p is not within the slab, no hit.
                if (p < bmin || p > bmax)
                    return false;
            } else {
                float t1 = (bmin - p) / di;
                float t2 = (bmax - p) / di;
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmin > tmax)
                    return false;
            }
        }
        return true;
    }

    inline std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segment_cuboid_intersection(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const obstacle_t& obst)
    {
        Eigen::Vector3d min = Eigen::Vector3d(obst.center.x() - obst.size_x / 2.0f, obst.center.y() - obst.size_y / 2.0f, obst.center.z() - obst.size_z / 2.0f);
        Eigen::Vector3d max = Eigen::Vector3d(obst.center.x() + obst.size_x / 2.0f, obst.center.y() + obst.size_y / 2.0f, obst.center.z() + obst.size_z / 2.0f);

        Eigen::Vector3d direction = p2 - p1; // Segment direction
        Eigen::Vector3d tMin, tMax;

        for (int i = 0; i < 3; ++i) {
            if (direction[i] != 0) {
                tMin[i] = (min[i] - p1[i]) / direction[i];
                tMax[i] = (max[i] - p1[i]) / direction[i];
                if (tMin[i] > tMax[i]) std::swap(tMin[i], tMax[i]);
            } else {
                // If the direction is zero, the segment is parallel to the axis
                if (p1[i] < min[i] || p1[i] > max[i]) {
                    return std::nullopt; // No intersection if outside bounds
                }
                tMin[i] = -std::numeric_limits<double>::infinity();
                tMax[i] = std::numeric_limits<double>::infinity();
            }
        }

        double tEnter = std::max({tMin[0], tMin[1], tMin[2]});
        double tExit = std::min({tMax[0], tMax[1], tMax[2]});

        if (tEnter > tExit || tEnter < 0 || tExit > 1) {
            return std::nullopt; // No intersection
        }

        Eigen::Vector3d intersectionStart = p1 + tEnter * direction;
        Eigen::Vector3d intersectionEnd = p1 + tExit * direction;

        return std::make_pair(intersectionStart, intersectionEnd);
    }

    // Given a candidate direction, 
    // use binary search to compute the minimal translation distance such that 
    // the shifted segment no longer intersects the AABB.
    inline float computeTranslationDistance(const Eigen::Vector3d &u1, const Eigen::Vector3d &u2, const cuboid::obstacle_t &obst, const Eigen::Vector3d &dir) {
        float low = 0.0f, high = 1000.0f; // Start with an arbitrary high value.
        const int iterations = 20;
        for (int i = 0; i < iterations; ++i) {
            float mid = (low + high) * 0.5f;
            Eigen::Vector3d offset = dir * mid;
            if (cuboid::segment_cuboid_intersects(u1 + offset, u2 + offset, obst))
                low = mid;
            else
                high = mid;
        }
        return high;
    }

    inline std::pair<Eigen::Vector3d, Eigen::Vector3d> computeNonIntersectingLine(const Eigen::Vector3d &u1, const Eigen::Vector3d &u2, const obstacle_t &box) {
        if (!segment_cuboid_intersects(u1, u2, box))
            return {u1, u2};
        
        Eigen::Vector3d d = (u1 - u2).normalized();
        
        Eigen::Vector3d bestOffset;
        float minTranslation = std::numeric_limits<float>::max(); 

        // Candidate directions (the coordinate axes). We later project these onto the plane perpendicular to d.
        std::vector<Eigen::Vector3d> candidates = { 
            Eigen::Vector3d(1,0,0), Eigen::Vector3d(-1,0,0),
            Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,-1,0),
            Eigen::Vector3d(0,0,1), Eigen::Vector3d(0,0,-1)
        };

        // Evaluate each candidate direction.
        for (const auto &v : candidates) {
            // Remove any component along d so that the translation is perpendicular.
            Eigen::Vector3d vProj = v - d * v.dot(d);
            if (vProj.norm() < 1e-6f)
                continue;  // Skip if nearly zero.
            vProj = vProj.normalized();
            
            float translation = cuboid::computeTranslationDistance(u1, u2, box, vProj);
            if (translation < minTranslation) {
                minTranslation = translation;
                bestOffset = vProj * translation;
            }
        }

        // Return the shifted endpoints.
        return {u1 + bestOffset, u2 + bestOffset};
    }


    inline bool point_inside_cuboid(const Eigen::Vector3d& point, const obstacle_t& obst)
    {
        Eigen::Vector3d min = Eigen::Vector3d(obst.center.x() - obst.size_x / 2.0f, obst.center.y() - obst.size_y / 2.0f, obst.center.z() - obst.size_z / 2.0f);
        Eigen::Vector3d max = Eigen::Vector3d(obst.center.x() + obst.size_x / 2.0f, obst.center.y() + obst.size_y / 2.0f, obst.center.z() + obst.size_z / 2.0f);

        return (point.x() >= min.x() && point.x() <= max.x() &&
                point.y() >= min.y() && point.y() <= max.y() &&
                point.z() >= min.z() && point.z() <= max.z());
    }

    inline obstacle_t inflate_obst(const obstacle_t& obst, double inflation)
    {
        obstacle_t inflated_obst = obst;
        inflated_obst.size_x += inflation;
        inflated_obst.size_y += inflation;
        inflated_obst.size_z += inflation;
        return inflated_obst;
    }
}

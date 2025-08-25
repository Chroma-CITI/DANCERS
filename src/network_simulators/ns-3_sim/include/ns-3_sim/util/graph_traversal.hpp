#pragma once

#include <queue>

/**
 * @brief Shortest path dijkstra algorithm (recursive)
 *
 * \param source The source node
 * \param graph The graph
 * \param dist The distance vector
 * \param parent The parent vector
 */
void dijkstra(int source, std::map<uint32_t, std::map<uint32_t, double>> &graph, std::map<uint32_t, double> &dist, std::map<uint32_t, uint32_t> &parent)
{
    double inf = std::numeric_limits<double>::infinity();
    // Iterate through the graph's keys to initialize distances and parents
    for (const auto& pair : graph) {
        dist[pair.first] = inf;
        parent[pair.first] = -1;
    }
    dist[source] = 0; // Distance to source is 0

    // Priority queue to store (distance, vertex)
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    pq.push({0, source}); // Push source node with distance 0

    while (!pq.empty())
    {
        int u = pq.top().second;   // Get vertex with smallest distance
        double d = pq.top().first; // Get the smallest distance
        pq.pop();

        if (d > dist[u])
        {
            continue; // Ignore if we already found a shorter path
        }

        // Explore neighbors
        for (auto &edge : graph[u])
        {
            int v = edge.first;          // Neighbor vertex
            double weight = edge.second; // Edge weight

            // Relax the edge if we find a shorter path
            if (dist[u] + weight < dist[v])
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Dijkstra algo"), "Node %d -> Node %d : %f\n(Better than %f)", u, v, dist[u] + weight, dist[v]);
                dist[v] = dist[u] + weight;
                parent[v] = u; // Update parent of v to be u
                pq.push({dist[v], v});
            }
        }
    }
    // Print result with ROS2 log
    for (const auto& [i, d] : dist)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("Dijkstra algo"), "Node %d -> Node %d : %f", source, i, d);
    }
}

/**
 * @brief Modified version of the Dijkstra algorithm to find the path with the widest bottleneck
 *
 * \param source The source node
 * \param graph The graph
 * \param dist The distance vector (here, capacity)
 * \param parent The parent vector
 */
void dijkstra_bottleneck(int source, std::map<uint32_t, std::map<uint32_t, double>> &graph, std::map<uint32_t, double> &dist, std::map<uint32_t, uint32_t> &parent)
{
    double inf = std::numeric_limits<double>::infinity();
    int n = graph.size();    // Number of vertices in the graph
    // Iterate through the graph's keys to initialize distances and parents
    for (const auto& pair : graph) {
        dist[pair.first] = inf;
        parent[pair.first] = -1;
    }
    dist[source] = inf;  // Distance to source is 0

    // Priority queue to store (distance, vertex)
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    pq.push({0, source}); // Push source node with distance 0

    while (!pq.empty())
    {
        int u = pq.top().second;   // Get vertex with smallest distance
        double d = pq.top().first; // Get the smallest distance
        pq.pop();

        if (d > dist[u])
        {
            RCLCPP_INFO(rclcpp::get_logger("Dijkstra algo"), "Discard node %d because we already found a shorter path.", u);
            continue; // Ignore if we already found a shorter path
        }

        // Explore neighbors
        for (auto &edge : graph[u])
        {
            int v = edge.first;          // Neighbor vertex
            double weight = edge.second; // Edge weight

            double bottleneck = std::max(dist[v], std::min(dist[u], weight));

            // Relax the edge if we find a shorter path
            if (bottleneck > dist[v])
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Dijkstra algo"), "Node %d -> Node %d : %f\n(Better than %f)", u, v, bottleneck, dist[v]);
                dist[v] = bottleneck;
                parent[v] = u; // Update parent of v to be u
                pq.push({dist[v], v});
            }
        }
    }
}

/**
 * @brief Modified version of the Dijkstra algorithm to find the path with the widest bottleneck
 *
 * \param source The source node
 * \param graph The graph
 * \param dist The distance vector (here, capacity)
 * \param parent The parent vector
 */
void dijkstra_error_rate(int source, std::vector<std::vector<std::pair<int, double>>> &graph, std::vector<double> &dist, std::vector<int> &parent)
{
    // double inf = std::numeric_limits<double>::infinity();
    int n = graph.size();                                    // Number of vertices in the graph
    dist.assign(n, std::numeric_limits<double>::infinity()); // Initialize distances with infinity
    parent.assign(n, -1);                                    // Initialize parent array with -1
    dist[source] = 0.0;                                      // Distance to source is 0

    // Priority queue to store (distance, vertex)
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    pq.push({0, source}); // Push source node with distance 0

    while (!pq.empty())
    {
        int u = pq.top().second;   // Get vertex with smallest distance
        double d = pq.top().first; // Get the smallest distance
        pq.pop();

        if (d > dist[u])
        {
            RCLCPP_INFO(rclcpp::get_logger("Dijkstra algo"), "Discard node %d because we already found a shorter path.", u);
            continue; // Ignore if we already found a shorter path
        }

        // Explore neighbors
        for (auto &edge : graph[u])
        {
            int v = edge.first;                         // Neighbor vertex
            double weight = -std::log(1 - edge.second); // Edge weight

            // Relax the edge if we find a shorter path
            if (dist[u] + weight < dist[v])
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Dijkstra algo"), "Node %d -> Node %d : %f\n(Better than %f)", u, v, dist[u] + weight, dist[v]);
                dist[v] = dist[u] + weight;
                parent[v] = u; // Update parent of v to be u
                pq.push({dist[v], v});
            }
        }
    }
}
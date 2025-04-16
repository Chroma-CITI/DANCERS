#pragma once

struct AABB_t
{
    double x_min;
    double y_min;
    double z_min;
    double x_max;
    double y_max;
    double z_max;
};

struct ns3_configuration_t
{
    int seed;
    uint32_t num_nodes;
    std::string wifi_phy_type;
    std::string wifi_phy_mode;
    std::string error_model;
    std::string propagation_loss_model;
    double frequency;
    std::string routing_algorithm;
    bool short_guard_interval_supported;
    bool use_real_routing_algorithm;
    std::vector<AABB_t> buildings;
    bool use_localization_noise;
    double localization_noise_stddev;

    // Network applications
    bool enable_broadcast_flow;
    double start_broadcast_time;
    double stop_broadcast_time;
    uint32_t broadcast_packet_size;
    uint32_t broadcast_interval;
    uint16_t broadcast_port;
    uint32_t broadcast_timeout;

    bool enable_mission_flow;
    std::vector<uint32_t> source_robots_ids;
    uint32_t sink_robot_id;
    double start_mission_time;
    double stop_mission_time;
    uint32_t mission_packet_size;
    uint32_t mission_interval;
    uint16_t mission_port;
    uint8_t mission_flow_id;
    uint32_t mission_timeout;

    bool enable_stats_module;

};
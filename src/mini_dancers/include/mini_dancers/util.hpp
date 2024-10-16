
#ifndef UTIL_HPP
#define UTIL_HPP

#include <uav_system.hpp>

#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

struct obstacle_t
{
    int id;
    Eigen::Vector3d center;
    double size_x, size_y, size_z;
};

struct triangle
{
    Eigen::Vector3d p1, p2, p3;
};

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
 * @brief Function used for flocking computation, see curve in Vásárhelyi 2018 Fig.6.
 */
double SigmoidLin(const double r, const double a, const double p)
{
    if (r <= 0)
    {
        return 0;
    }
    else if (r * p > 0 && r * p < a / p)
    {
        return r * p;
    }
    else
    {
        return std::sqrt(2 * a * r - std::pow(a, 2) / std::pow(p, 2));
    }
}

struct agent_t
{
    mrs_multirotor_simulator::UavSystem uav_system;
    int id;
    std::vector<int> neighbors;
    std::vector<int> neighbors_mission;
    std::vector<int> neighbors_potential;
    std::vector<double> link_qualities;
    std::optional<Eigen::Vector3d> secondary_objective;
};

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

/**
 * \brief Compresses a string with the zip protocol.
 *
 * \param data The string to compress.
 * \return The compressed string.
 */
static std::string gzip_compress(const std::string &data)
{
    std::stringstream compressed;
    std::stringstream origin(data);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_compressor());
    in.push(origin);
    boost::iostreams::copy(in, compressed);

    return compressed.str();
}

/**
 * \brief Decompress a string with the zip protocol.
 *
 * \param data The compressed string to decompress.
 * \return The decompressed string.
 */
static std::string gzip_decompress(const std::string &data)
{
    std::stringstream compressed(data);
    std::stringstream decompressed;

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(compressed);
    boost::iostreams::copy(in, decompressed);

    return decompressed.str();
}

/**
 * \brief Receives a message from a socket.
 *
 * This function will block until the next message is received, read its header (first 4 bytes)
 * and then read the content of the message and return it as a string.
 *
 * \param sock The socket on which to listen for the next message.
 * \return The received message as a std::string.
 */
std::string receive_one_message(boost::asio::local::stream_protocol::socket &sock)
{
    // Read Preamble
    uint32_t data_preamble[4];
    size_t length = sock.receive(boost::asio::buffer(data_preamble, 4));
    uint32_t receive_length = ntohl(*data_preamble);
    // Read Message
    char *data = new char[receive_length];
    length = sock.receive(boost::asio::buffer(data, receive_length));
    std::string data_string(data, length);

    return data_string;
}

/**
 * \brief Sends a message from a socket.
 *
 * \param sock The socket used to send the message.
 * \param str The string message to send.
 */
void send_one_message(boost::asio::local::stream_protocol::socket &sock, std::string str)
{
    // Send Preamble
    std::size_t response_size = str.size();
    uint32_t send_length = htonl(static_cast<uint32_t>(response_size));
    sock.send(boost::asio::buffer(&send_length, 4));
    // Send Message
    sock.send(boost::asio::buffer(str.data(), str.size()));
}

#endif // UTIL_HPP
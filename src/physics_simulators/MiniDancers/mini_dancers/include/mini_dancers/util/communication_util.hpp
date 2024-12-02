#pragma once


#include <boost/asio.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

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

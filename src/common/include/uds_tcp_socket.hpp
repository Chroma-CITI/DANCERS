#include <boost/asio.hpp>
#include <iostream>
#include <thread>


/**
 * @brief A virtual class of a Socket
 */
class CustomSocket
{
public:
    virtual void accept(const std::string &host, unsigned short port) = 0;
    virtual void connect(const std::string &host) = 0;
    virtual void send_one_message(const std::string &message) = 0;
    virtual void send_one_double(const double &value) = 0;
    virtual void send_one_int(const int &value) = 0;
    virtual void send_double_buffer(const std::vector<double> &values) = 0;
    virtual void send_double_array(const double *values, const int size) = 0;
    virtual std::string receive_one_message() = 0;
    virtual void close() = 0;
};

/**
 * @brief A Unix Domain Socket (UDS) implementation of the CustomSocket class using the Boost library
 */
class UDSSocket : public CustomSocket
{
public:
    UDSSocket(boost::asio::io_context &io_context) : socket_(io_context) {}
    void accept(const std::string &host, unsigned short) override
    {
        ::unlink(host.c_str());
        boost::asio::local::stream_protocol::acceptor acceptor(socket_.get_executor(), boost::asio::local::stream_protocol::endpoint(host));
        acceptor.accept(socket_);
    }
    void connect(const std::string &host) override
    {
        boost::asio::local::stream_protocol::endpoint endpoint(host.c_str());
        for (int i = 0; i < 20; i++)
        {
            try
            {
                socket_.connect(endpoint);
                return;
            }
            catch (std::exception &e)
            {
                std::cerr << "Error connecting to socket on try " << i << " : " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        std::cerr << "Couldn't connecting to socket after 20 tries." << std::endl;
        exit(EXIT_FAILURE);
    }
    void send_one_message(const std::string &message) override
    {
        // Send Preamble
        std::size_t response_size = message.size();
        uint32_t send_length = htonl(static_cast<uint32_t>(response_size));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(message.data(), message.size()));
    }
    void send_one_double(const double &value) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(sizeof(double)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(&value, sizeof(double)));
    }
    void send_one_int(const int &value) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(sizeof(int)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(&value, sizeof(int)));
    }
    void send_double_buffer(const std::vector<double> &values) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(values.size() * sizeof(double)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(values.data(), values.size() * sizeof(double)));
    }
    void send_double_array(const double *values, const int size) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(size * sizeof(double)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(values, size * sizeof(double)));
    }
    std::string receive_one_message() override
    {
        // Read Preamble
        uint32_t data_preamble;
        boost::asio::read(socket_, boost::asio::buffer(&data_preamble, 4));
        uint32_t receive_length = ntohl(data_preamble);

        // Read Message
        std::string message(receive_length, 0); // Allocate string with correct size
        boost::asio::read(socket_, boost::asio::buffer(&message[0], receive_length));

        return message;
    }
    void close() override
    {
        socket_.close();
    }

private:
    boost::asio::local::stream_protocol::socket socket_;
};

/**
 * @brief A TCP implementation of the CustomSocket class using the Boost library
 */
class TCPSocket : public CustomSocket
{
public:
    TCPSocket(boost::asio::io_context &io_context) : socket_(io_context) {}
    void accept(const std::string &host, unsigned short port) override
    {
        boost::asio::ip::tcp::acceptor acceptor(socket_.get_executor(), boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address(host), port));
        acceptor.accept(socket_);   
    }
    void connect(const std::string &host) override
    {
        boost::asio::ip::tcp::resolver resolver(socket_.get_executor());
        boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(host, "80");
        boost::asio::connect(socket_, endpoints);
    }
    void send_one_message(const std::string &message) override
    {
        // Send Preamble
        std::size_t response_size = message.size();
        uint32_t send_length = htonl(static_cast<uint32_t>(response_size));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(message.data(), message.size()));
    }
    void send_one_double(const double &value) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(sizeof(double)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(&value, sizeof(double)));
    }
    void send_one_int(const int &value) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(sizeof(int)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(&value, sizeof(int)));
    }
    void send_double_buffer(const std::vector<double> &values) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(values.size() * sizeof(double)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(values.data(), values.size() * sizeof(double)));
    }
    void send_double_array(const double *values, const int size) override
    {
        // Send Preamble
        uint32_t send_length = htonl(static_cast<uint32_t>(size * sizeof(double)));
        socket_.send(boost::asio::buffer(&send_length, 4));
        // Send Message
        socket_.send(boost::asio::buffer(values, size * sizeof(double)));
    }
    std::string receive_one_message() override
    {
        // Read Preamble
        uint32_t data_preamble[4];
        size_t length = socket_.receive(boost::asio::buffer(data_preamble, 4));
        uint32_t receive_length = ntohl(*data_preamble);
        // Read Message
        char *data = new char[receive_length];
        length = socket_.receive(boost::asio::buffer(data, receive_length));
        std::string data_string(data, length);
        return data_string;
    }
    void close() override
    {
        socket_.close();
    }

private:
    boost::asio::ip::tcp::socket socket_;
};

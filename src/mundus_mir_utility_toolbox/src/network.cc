#include "utility_toolbox/network.h"

///////////////////////////////////////////////////// SIMPLE UDP SERVER CLASS //////////////////////////////////////////


utility_toolbox::Simple_UDP_Client::Simple_UDP_Client() :
    socket_(io_context_), resolver_(socket_.get_executor()) {
    
    // Opening socket for udp ipv4
    socket_.open(asio::ip::udp::v4());
    }

utility_toolbox::Simple_UDP_Client::Simple_UDP_Client(const int& port, const string& ip) :
    socket_(io_context_), resolver_(socket_.get_executor()) {

    // Opening socket for udp ipv4
    socket_.open(asio::ip::udp::v4());

    // Check that ip and port is valid
    if (!utility_toolbox::ipv4_valid(ip)) {
        // Error handeling        
    }

    if (port < 0) {
        // Error handeling
    }

    // Port and ip is OK, setting up endpoint
    endpoint_ = *resolver_.resolve(asio::ip::udp::v4(), ip, to_string(port)).begin();
}

utility_toolbox::Simple_UDP_Client::~Simple_UDP_Client() {
    socket_.close();
}

bool utility_toolbox::Simple_UDP_Client::send_message(const string& message) {

    // Send message, should have some error handeling here aswell
    socket_.send_to(asio::buffer(message), endpoint_);
    return true;
}

bool utility_toolbox::Simple_UDP_Client::set_endpoint(const int& port, const string& ip) {

    // Check validity
    if (port < 0) {
        return false;
    }

    if (!utility_toolbox::ipv4_valid(ip)) {
        return false;
    }

    // Port and ip is OK, setting up endpoint
    endpoint_ = *resolver_.resolve(asio::ip::udp::v4(), ip, to_string(port)).begin();

    return true;
}






/////////////////////////////////////////////////////// Simple_UDP_Server Class ////////////////////////////////////////

utility_toolbox::Simple_UDP_Server::Simple_UDP_Server() :
socket_(io_context_), resolver_(socket_.get_executor()) {

}

utility_toolbox::Simple_UDP_Server::Simple_UDP_Server(const int& port, const string& ip) : 
    socket_(io_context_), resolver_(socket_.get_executor())
{
   endpoint_ = asio::ip::udp::endpoint(asio::ip::udp::v4(), port);
   socket_ = asio::ip::udp::socket(io_context_, endpoint_);
}

utility_toolbox::Simple_UDP_Server::~Simple_UDP_Server() {
    socket_.close();
};

string utility_toolbox::Simple_UDP_Server::recv() {
    char data[1024];
    // Endpoint used to store sender information
    asio::ip::udp::endpoint sender_endpoint;
    size_t length = socket_.receive_from(asio::buffer(data), sender_endpoint); 
    return string(data, length);
};

string utility_toolbox::Simple_UDP_Server::asy_recv() {
    auto data = make_shared<array<char, 1024>>();
    // Endpoint used to store sender information
    asio::ip::udp::endpoint sender_endpoint;
    asio::error_code error;
    size_t length = 0;

    // Timer for timeout
    asio::steady_timer timer(io_context_);
    timer.expires_after(chrono::milliseconds(100));

    // Asynch socket call
    socket_.async_receive_from(asio::buffer(*data), sender_endpoint,
    [&](const asio::error_code& ec, size_t len) {
        error = ec;
        length = len;
        timer.cancel();
    });

    // run I/O context to block
    io_context_.restart();
    io_context_.run();

    // If socket times out
    if (error == asio::error::operation_aborted) {
        return "";
    }

    // If not send data 
    return string(data->data(), length);
}

bool utility_toolbox::Simple_UDP_Server::set_endpoint(const int& port, const string& ip) {

    // Checks for valid IP
    if (!utility_toolbox::ipv4_valid(ip)) {
        return false;
    }

    // Close socket and open new 
    socket_.close();
    endpoint_ = asio::ip::udp::endpoint(asio::ip::udp::v4(), port);
    socket_ = asio::ip::udp::socket(io_context_, endpoint_);
    return true;
};

    





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns true if given string is a valid ipv4 else false
bool utility_toolbox::ipv4_valid(const string& ip) {

    // Initializing variables
    vector<string> parts;
    istringstream ip_stream(ip);
    string part;

    // Split the string into parts
    while(getline(ip_stream, part, '.')) {
        parts.push_back(part);
    }
    // More than 4 parts?
    if (parts.size() > 4) {
        return false;
    }

    // Check every part
    for (const auto& p : parts) {

        // Checks that parts are not empty and no leading zeros
        if (p.empty() || (p.size() > 1 && p[0] == '0')) {
            return false;
        }

        // Check that all numbers are within valid range
        for (char c : p) {
            if (!isdigit(c)) {
                return false;
            }
        }

        int num = stoi(p);
        if (num < 0 || num > 255) {
            return false;
        }
    }
    // All tests are passed, it is a valid IP address
    return true;
}

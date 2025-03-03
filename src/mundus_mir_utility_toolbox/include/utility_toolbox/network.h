#ifndef NETWORK_H_
#define NETWORK_H_

#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <asio.hpp>

using namespace std;

namespace utility_toolbox {

// Returns true if given string is a valid ipv4 else false
bool ipv4_valid(const string& ip);

// A simple UDP client that can be utilized to send UDP messages to spesific IP and port
class Simple_UDP_Client
{
    public:
        // Constructors and destructors
        Simple_UDP_Client(); 
        Simple_UDP_Client(const int& port, const string& ip);
        ~Simple_UDP_Client();

        // Send encoded message
        bool send_message(const string& message);
        bool set_endpoint(const int& port, const string& ip);

    private:
        asio::io_context io_context_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::resolver resolver_;
        asio::ip::udp::endpoint endpoint_;
};



class Simple_UDP_Server
{
    public:
        // Constructors and destructors
        Simple_UDP_Server();
        Simple_UDP_Server(const int& port, const string& ip);
        ~Simple_UDP_Server();

        // Recv message
        string recv();
        string asy_recv();
        bool set_endpoint(const int&port, const string& ip);

    private:
        asio::io_context io_context_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::resolver resolver_;
        asio::ip::udp::endpoint endpoint_;
};

};

#endif
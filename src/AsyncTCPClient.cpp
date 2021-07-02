#include "../include/boost_tcp/AsyncTCPClient.h"

namespace boost_tcp
{

AsyncTCPClient::AsyncTCPClient(io_service& io_service, std::string ip, int port, double dt, 
                               MessageProcessBufferPtr rbuffer, MessageProcessBufferPtr wbuffer)
    : io_service_(io_service), endpoint_(address_t::from_string(ip), port), dt_(dt), 
    read_timer_(io_service_), write_timer_(io_service_), deadline_(io_service_),
    recv_process_buffer_(rbuffer), send_process_buffer_(wbuffer), stopped_(false)
{
    std::cout << "<AsyncTCPClient>: Start client" << std::endl;
    connect();
}

}; // ns

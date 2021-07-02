/*
 * Filename: AsyncTCPServer.cpp
 * Path: tcp_ros_bridge
 * Created Date: Tuesday, June 29th 2021, 15:14:39
 * Author: zhao wang
 *
 * Copyright (c) 2021 hust-arms
 */

#include "../include/boost_tcp/AsyncTCPServer.h"

namespace boost_tcp
{
/////////////////////////////////////////////////////
AsyncTCPServer::AsyncTCPServer(io_service& io_service, int port, double dt, MessageProcessBufferPtr recv_buffer, MessageProcessBufferPtr send_buffer):
    io_service_(io_service), acceptor_(io_service_, endpoint_t(ip::tcp::v4(), port)), recv_buffer_(recv_buffer), send_buffer_(send_buffer), 
    read_timer_(io_service_), write_timer_(io_service_), dt_(dt), stopped_(false)
{
    acceptor_.set_option(ip::tcp::acceptor::reuse_address(true));

    std::cout << "<AsyncTCPServer>: Start async TCP server" << std::endl;
    accept();
}
}; // ns

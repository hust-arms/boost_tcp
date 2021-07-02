/*                                                                                                                          
 * Filename: AsyncTCPClient.h
 * Path: boost_tcp
 * Created Date: Tuesday, June 29th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once

#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

#include <iostream>

#include "MessageProcessBuffer.h"

// #define DEBUG

using namespace boost::asio;

const int BUFFER_MAX_LEN = 1024;

namespace boost_tcp
{
/**
 * @brief Async TCP client with boost lib
 */
class AsyncTCPClient
{
typedef AsyncTCPClient this_t;
typedef boost::asio::ip::tcp::endpoint endpoint_t;
typedef boost::asio::ip::address address_t;
typedef boost::asio::ip::tcp::socket socket_t;
typedef boost::shared_ptr<socket_t> pSocket_t; 
public:
    /**
     * @brief Constructor
     * @param ip Server Ip address
     * @param port Server port number
     * @rbuffer Message receive process buffer
     * @wbuffer Message send process buffer
     */
    AsyncTCPClient(io_service& io_service, std::string ip, int port, double dt, MessageProcessBufferPtr rbuffer, MessageProcessBufferPtr wbuffer);

    ~AsyncTCPClient() {}

    void run()
    {
        io_service_.run();
    }

    bool isStopped()
    {
        return stopped_;
    }

private:
    /**
     * @brief Client socket connect
     */
    void connect()
    {
        pSocket_t sock = boost::make_shared<socket_t>(io_service_);
        sock->async_connect(endpoint_, boost::bind(&this_t::connectHandle, this, placeholders::error, sock)); 
    }

    void connectHandle(const boost::system::error_code& ec, pSocket_t sock)
    {
        if(!ec)
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPClient>: Connect to " << sock->remote_endpoint().address() << std::endl;
#endif
            startRead(sock);
            startWrite(sock);
            // if(stopped_)
            //     return;
            connect();
        }
        else
        {
#ifdef DEBUG
            std::cerr << "<AsyncTCPClient>: Error in connect handle " << ec.message() << std::endl;
#endif
            stop(sock);
        }
    }
    
    void startRead(pSocket_t sock)
    {
        // read_timer_.expires_from_now(boost::posix_time::seconds(dt_));
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt_ * 1000));
#ifdef DEBUG
        std::cout << "<AsyncTCPClient>: Recv message" << std::endl;
#endif
        sock->async_read_some(buffer(buffer_, BUFFER_MAX_LEN), boost::bind(&this_t::readHandle, this, 
                                                                           placeholders::error, placeholders::bytes_transferred, sock));
    }

    /**
     * @brief Receive message processing 
     */
    void readHandle(const boost::system::error_code& ec, size_t bytes, pSocket_t sock)
    {
        if(ec)
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPClient>: Stop client in reading: " << ec.message() << std::endl;
#endif
            sock->close();
            stopped_ = true;
        }
        else
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPClient>: Parse recv message" << std::endl;
#endif
            recv_process_buffer_->parse(buffer_, bytes);
        }
    }

    void startWrite(pSocket_t sock)
    {
        // write_timer_.expires_from_now(boost::posix_time::seconds(dt_));
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt_ * 1000));
#ifdef DEBUG
        std::cout << "<AsyncTCPClient>: Writing" << std::endl;
        std::cout << "<AsyncTCPClient>: Get send message" << std::endl;
#endif

        uint8_t temp_buffer[BUFFER_MAX_LEN];
        size_t len = send_process_buffer_->getMessage(temp_buffer);

        sock->async_write_some(buffer(temp_buffer, len), boost::bind(&this_t::writeHandle, this, 
                                                           placeholders::error, sock));
    }

    /**
     * @brief Send message processing 
     */
    void writeHandle(const boost::system::error_code& ec, pSocket_t sock)
    {
        if(ec)
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPClient>: stop client in writing: " << ec.message() << std::endl;
#endif
            sock->close();
            stopped_ = true;
        }
    }

    void stop(pSocket_t sock)
    {
        read_timer_.cancel();
        write_timer_.cancel();
        sock->close();
        stopped_ = true;
    }

    io_service& io_service_;
    endpoint_t endpoint_;

    uint8_t buffer_[BUFFER_MAX_LEN];

    deadline_timer read_timer_;
    deadline_timer write_timer_;
    deadline_timer deadline_;

    MessageProcessBufferPtr recv_process_buffer_;
    MessageProcessBufferPtr send_process_buffer_;

    double dt_;
    bool stopped_;

}; // AsyncTCPClient
}; // ns

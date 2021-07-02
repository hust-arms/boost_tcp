/*                                                                                                                          
 * Filename: AsyncTCPServer.h
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
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>

#include <iostream>

#include "MessageProcessBuffer.h"

using namespace boost::asio;

#define DEBUG

const int BUFFER_MAX_LEN = 1024;

namespace boost_tcp
{

/**
 * @brief Async TCP server with boost lib
 */
class AsyncTCPServer
{
typedef AsyncTCPServer this_t;
typedef boost::asio::ip::tcp::acceptor acceptor_t;
typedef boost::asio::ip::tcp::endpoint endpoint_t;
typedef boost::asio::ip::tcp::socket socket_t;
typedef boost::shared_ptr<socket_t> pSocket_t;


public:
    /**
     * @brief TCP Server construtor
     * @param port Port number of server
     * @param dt Data proccess time period
     * @param recv_buffer Receive message processing buffer
     * @param send_buffer Send message processing buffer
     */
    AsyncTCPServer(io_service& io_service, int port, double dt, MessageProcessBufferPtr recv_buffer, MessageProcessBufferPtr send_buffer);

    /**
     * @brief Deconstrutor
     */
    ~AsyncTCPServer() {}

    /**
     * @brief Run service 
     */
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
     * @brief Server socket accept
     */
    void accept()
    {
        pSocket_t sock = boost::make_shared<socket_t>(io_service_);
        acceptor_.async_accept(*sock, boost::bind(&this_t::acceptHandle, this, 
                                                  placeholders::error, sock));
    }

    void acceptHandle(const boost::system::error_code& ec, pSocket_t sock)
    {
        if(!ec)
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPServer>: Accept client: " << sock->remote_endpoint().address() << std::endl;
#endif
            startWrite(sock);
            startRead(sock);
            accept();
        }
        else
        {
#ifdef DEBUG
            std::cerr << "<AsyncTCPServer>: Error in accept handle: " << ec.message() << std::endl;
#endif
        }
    }

    void startWrite(pSocket_t sock)
    {
        // write_timer_.expires_from_now(boost::posix_time::seconds(dt_));
        // write_timer_.async_wait(boost::bind(&this_t::write, this, sock));
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt_ * 1000));
        write(sock);
        // std::cout << "<AsyncTCPServer>: Write: " << std::endl;
        // sock->async_write_some(buffer("hello"), boost::bind(&this_t::writeHandle, this, 
        //                                                     placeholders::error, sock));
    }

    /**
     * @brief Send message
     */
    void write(pSocket_t sock)
    {
#ifdef DEBUG
        std::cout << "<AsyncTCPServer>: Write" << std::endl;
        std::cout << "<AsyncTCPServer>: Get send message" << std::endl;
#endif
        uint8_t temp_buffer[BUFFER_MAX_LEN];
        size_t len = send_buffer_->getMessage(temp_buffer);

        // std::cout << "<AsyncTCPServer>: Writein: ";
        // for(int i = 0; i < len; ++i)
        // {
        //     printf("0x%x ", temp_buffer[i]);
        // }
        // printf("\n");

        sock->async_write_some(buffer(temp_buffer, len), boost::bind(&this_t::writeHandle, this, 
                                                            placeholders::error, sock));
    }

    void startRead(pSocket_t sock)
    {
        // read_timer_.expires_from_now(boost::posix_time::seconds(dt_));
        // read_timer_.async_wait(boost::bind(&this_t::read, this, sock));
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt_ * 1000));
        read(sock);
        // std::cout << "<AsyncTCPServer>: Read: " << std::endl;
        // sock->async_read_some(buffer(buffer_, BUFFER_MAX_LEN), boost::bind(&this_t::readHandle, this, 
        //                                                                    placeholders::error, sock));
    }

    /**
     * @brief Receive message
     */
    void read(pSocket_t sock)
    {
#ifdef DEBUG
        std::cout << "<AsyncTCPServer>: Read" << std::endl;
#endif
        sock->async_read_some(buffer(buffer_, BUFFER_MAX_LEN), boost::bind(&this_t::readHandle, this, 
                                                                           placeholders::error, placeholders::bytes_transferred, sock));
    }

    /**
     * @brief Process error in message receive
     */
    void readHandle(const boost::system::error_code& ec, size_t bytes_transferred, pSocket_t sock)
    {
        if(ec)
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPServer>: stop server in reading: " << ec.message() << std::endl;
#endif
            stop(sock);
        }
        else
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPServer>: Process received message" << std::endl;
#endif
            recv_buffer_->parse(buffer_, bytes_transferred);
        }
    }

    /**
     * @brief Process error in message send
     */
    void writeHandle(const boost::system::error_code& ec, pSocket_t sock)
    {
        if(ec)
        {
#ifdef DEBUG
            std::cout << "<AsyncTCPServer>: stop server in writing: " << ec.message() << std::endl;
#endif
            stop(sock);
            return;
        }
    }

    void stop(pSocket_t sock)
    {
        read_timer_.cancel();
        write_timer_.cancel();
        sock->close();
        stopped_ = true;
    }

    uint8_t buffer_[BUFFER_MAX_LEN];

    io_service& io_service_;
    acceptor_t acceptor_;

    deadline_timer read_timer_;
    deadline_timer write_timer_;

    MessageProcessBufferPtr recv_buffer_;
    MessageProcessBufferPtr send_buffer_;

    double dt_;

    bool stopped_;

}; // AsyncTCPServer
}; // ns

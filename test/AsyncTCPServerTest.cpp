/*                                                                                                                          
 * Filename: AsyncTCPServerTest.cpp
 * Path: boost_tcp
 * Created Date: Tuesday, June 29th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <iostream>
#include "../include/boost_tcp/AsyncTCPServer.h"
#include "../include/boost_tcp/MessageProcessBuffer.h"

int main()
{
    boost_tcp::MessageProcessBufferPtr recv_buffer = boost::make_shared<boost_tcp::MessageProcessBufferTest>();
    boost_tcp::MessageProcessBufferPtr send_buffer = boost::make_shared<boost_tcp::MessageProcessBufferTest>();

    boost_tcp::AsyncTCPServer srv(10001, 2.0, recv_buffer, send_buffer);
    srv.run();

    return 0;
}


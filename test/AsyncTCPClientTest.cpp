/*                                                                                                                          
 * Filename: AsyncTCPClientTest.cpp
 * Path: boost_tcp
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <iostream>
#include "../include/boost_tcp/AsyncTCPClient.h"
#include "../include/boost_tcp/MessageProcessBuffer.h"

int main()
{
    boost_tcp::MessageProcessBufferPtr p_recv_buffer = boost::make_shared<boost_tcp::MessageProcessBufferTest>();
    boost_tcp::MessageProcessBufferPtr p_send_buffer = boost::make_shared<boost_tcp::MessageProcessBufferTest>();

    boost_tcp::AsyncTCPClient cl("127.0.0.1", 10001, 0.5, p_recv_buffer, p_send_buffer);
    cl.run();

    return 0;
}


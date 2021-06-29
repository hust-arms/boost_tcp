#pragma once

#include <boost/shared_ptr.hpp>

#include <stdio.h>
#include <iostream>

namespace boost_tcp
{

class MessageProcessBuffer
{
public:
    virtual ~MessageProcessBuffer() {}

    virtual void parse(uint8_t* buffer, size_t len) = 0;
    virtual size_t getMessage(uint8_t* buffer) = 0;
}; // MessageProecessBuffer


typedef boost::shared_ptr<MessageProcessBuffer> MessageProcessBufferPtr;

//////////////////////////////////////////////////////////////

class MessageProcessBufferTest : public MessageProcessBuffer
{
public:
    void parse(uint8_t* buffer, size_t len) override
    {
        std::cout << "<MessageProcessBufferTest>: parse" << std::endl;
        for(int i = 0; i < len; ++i)
        {
            std::cout << static_cast<char>(*buffer);
            ++buffer;
        }
        std::cout << std::endl;
    }

    size_t getMessage(uint8_t* buffer)override
    {
        std::cout << "<MessageProcessBufferTest>: get message" << std::endl;
        std::string str = "test";
        uint8_t temp[str.size()];
        for(int i = 0; i < str.size(); ++i)
        {
            temp[i] = static_cast<uint8_t>(str[i]);
        }
        buffer = temp;
        return static_cast<size_t>(sizeof(buffer) / sizeof(uint8_t));
    }
}; // MessageProcessBufferTest

}; //ns

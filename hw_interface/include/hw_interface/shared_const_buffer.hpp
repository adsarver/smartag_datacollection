#ifndef SHARED_CONST_BUFFER_HPP__
#define SHARED_CONST_BUFFER_HPP__

//
// reference_counted.hpp
// ~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio.hpp>
#include <vector>

// A reference-counted non-modifiable buffer class.

namespace hw_interface_support_types
{
    class shared_const_buffer
    {
        public:
            // Construct from a std::string.
            shared_const_buffer(const std::string& data)
                : data_(new std::vector<char>(data.begin(), data.end())),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {
            }

            shared_const_buffer(const char* data, const int length)
                : data_(new std::vector<char>(data, data+length)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {
            }

            shared_const_buffer(const char* data, const int length, bool LEtoBE)
                : data_(new std::vector<char>(length)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {
                if(LEtoBE)
                {
                    for(int i = 0; i < length; i++)
                    {
                        data_->data()[i] = data[length-1-i];
                    }
                }
            }

            shared_const_buffer(const std::vector<char> &data)
                : data_(new std::vector<char>(data)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {

            }

            template<typename T>
            shared_const_buffer(const std::vector<T> &data)
                : data_(new std::vector<char>(data)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {

            }

            template<typename S>
            shared_const_buffer(const S &data)
                :   data_(new std::vector<char>(reinterpret_cast<char*>(&data), sizeof(S)+reinterpret_cast<char*>(&data))),
                    buffer_(boost::asio::buffer(*data_))
            {

            }


            // Implement the ConstBufferSequence requirements.
            typedef boost::asio::const_buffer value_type;
            typedef const boost::asio::const_buffer* const_iterator;
            const boost::asio::const_buffer* begin() const { return buffer_.get(); }
            const boost::asio::const_buffer* end() const { return buffer_.get() + 1; }

        protected:
            //protected empty constructor for implementation overlods if needed
            shared_const_buffer() {}
            std::shared_ptr<std::vector<char> > data_;
            std::shared_ptr<boost::asio::const_buffer> buffer_;

    };
}

#endif //SHARED_CONST_BUFFER_HPP__


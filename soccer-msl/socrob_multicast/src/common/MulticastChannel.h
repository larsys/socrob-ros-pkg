/*
 * Copyright 2012, 2013 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of SocRob Multicast.
 *
 * SocRob Multicast is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SocRob Multicast is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with SocRob Multicast.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SOCROB_MULTICAST_MULTICASTCHANNEL_H_
#define _SOCROB_MULTICAST_MULTICASTCHANNEL_H_

#include <cstddef>
#include <stdint.h>

#include <vector>
#include <string>

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

#include <socrob_multicast/definitions.h>



namespace socrob
{
  namespace multicast
  {
    class MulticastChannel :
      private boost::noncopyable
    {
      public:
        // buffer is not constant on purpose, so it can be swapped if needed.
        typedef boost::function < void (boost::posix_time::ptime const& time,
                                        std::vector<uint8_t> & buffer) > receive_callback_type;
                                        
        MulticastChannel (boost::asio::io_service& io_service,
                          std::string const& ADDRESS_STRING,
                          unsigned short PORT,
                          receive_callback_type const& receive_callback);
                          
        void
        send (std::vector<uint8_t> const& buffer);
        
      private:
        boost::asio::ip::udp::socket socket_;
        boost::asio::ip::udp::endpoint endpoint_;
        
        void receive_setup();
        void receive_handler (const boost::system::error_code& error, std::size_t bytes_recvd);
        
        boost::asio::ip::udp::endpoint receive_endpoint_;
        std::vector<uint8_t> receive_buffer_;
        receive_callback_type receive_callback_;
    };
  }
}

#endif

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

#include "MulticastChannel.h"

#include <boost/bind.hpp>

#include <ros/console.h>



using namespace std;
using namespace boost::posix_time;
using namespace boost::asio::ip;
using namespace socrob::multicast;



MulticastChannel::
MulticastChannel (boost::asio::io_service& io_service,
                  string const& ADDRESS_STRING,
                  unsigned short PORT,
                  receive_callback_type const& receive_callback) :
  socket_ (io_service),
  receive_callback_ (receive_callback)
{
  const address_v4 ADDRESS = address_v4::from_string (ADDRESS_STRING);
  socket_.open (udp::v4());
  socket_.set_option (udp::socket::reuse_address (true));
  socket_.bind (udp::endpoint (udp::v4(), PORT));
#ifdef SOCROB_MULTICAST_USE_LOOPBACK_INTERFACE
  socket_.set_option (boost::asio::ip::multicast::outbound_interface (address_v4::loopback()));
  socket_.set_option (boost::asio::ip::multicast::join_group (ADDRESS, address_v4::loopback()));
#else
  socket_.set_option (boost::asio::ip::multicast::join_group (ADDRESS));
#endif
  socket_.set_option (boost::asio::ip::multicast::enable_loopback (true));
  endpoint_ = udp::endpoint (ADDRESS, PORT);
  
  receive_setup();
}



void
MulticastChannel::
send (std::vector<uint8_t> const& buffer)
{
  size_t sent = socket_.send_to (boost::asio::buffer (buffer), endpoint_);
  if (sent != buffer.size()) {
    ROS_ERROR ("Error sending UDP datagram in MulticastChannel::send");
  }
}



void
MulticastChannel::
receive_setup()
{
  receive_buffer_.clear();
  receive_buffer_.resize (MAX_MESSAGE_SIZE);
  
  socket_.async_receive_from (boost::asio::buffer (receive_buffer_),
                              receive_endpoint_,
                              boost::bind (&MulticastChannel::receive_handler,
                                  this,
                                  boost::asio::placeholders::error,
                                  boost::asio::placeholders::bytes_transferred));
}



void
MulticastChannel::
receive_handler (const boost::system::error_code& error,
                 size_t bytes_recvd)
{
  ptime rcv_time = microsec_clock::universal_time();
  
  if (error) {
    ROS_FATAL_STREAM ("Error in MulticastChannel::receive_handler : " << error.message());
    abort();
  }
  
  receive_buffer_.resize (bytes_recvd);
  receive_callback_ (rcv_time, receive_buffer_);
  
  receive_setup();
}

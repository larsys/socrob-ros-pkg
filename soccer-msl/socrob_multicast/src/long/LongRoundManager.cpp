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

#include "LongRoundManager.h"

#include <boost/bind.hpp>

#include <ros/console.h>

#include <socrob_multicast/serialization.h>

#include <socrob_multicast/Long.h>



using namespace std;
using namespace boost::posix_time;
using namespace socrob::multicast;
using boost::shared_ptr;



LongRoundManager::
LongRoundManager (shared_ptr<Protocol> const& protocol,
                  boost::asio::io_service& io_service,
                  std::string const& ADDRESS_STRING,
                  unsigned short PORT,
                  buffer_function_type const& marshall,
                  id_buffer_function_type const& unmarshall,
                  unsigned tup_us,
                  boost::shared_ptr<Compressor> const& compressor) :
  marshall_ (marshall),
  unmarshall_ (unmarshall),
  compressor_ (compressor),
  protocol_ (protocol),
  time_control_ (protocol->SID(), protocol->NUM_SIDS(), tup_us),
  timer_ (io_service),
  short_active_last_time_ (false),
  channel_ (io_service, ADDRESS_STRING, PORT, boost::bind (&LongRoundManager::receive_callback, this, _1, _2))
{
  timer_.expires_from_now (microseconds (0));
  control (boost::system::error_code());
}



void
LongRoundManager::
control (const boost::system::error_code& error)
{
  if (error) {
    ROS_FATAL_STREAM_NAMED ("multicast.long", "Error in LongRoundManager::control : " << error.message());
    abort();
  }
  
  ptime send_time = time_control_.control();
  time_duration prepare_estimation = prepare_estimator_.get();
  // Timer for prepare should never be set to before this control timer.
  timer_.expires_at (max (timer_.expires_at(), send_time - prepare_estimation));
  ROS_DEBUG_STREAM_NAMED ("multicast.long", "Timer (prepare) set to: " << ros::Time::fromBoost (timer_.expires_at())
                          << " Send time: " << ros::Time::fromBoost (send_time)
                          << " Prepare estimation: " << ros::Time::fromBoost (prepare_estimation));
  timer_.async_wait (boost::bind (&LongRoundManager::prepare,
                                  this,
                                  boost::asio::placeholders::error));
}



void
LongRoundManager::
prepare (const boost::system::error_code& error)
{
  if (error) {
    ROS_FATAL_STREAM_NAMED ("multicast.long", "Error in LongRoundManager::prepare : " << error.message());
    abort();
  }
  
  ROS_DEBUG_NAMED ("multicast.long", " ---- Preparing to send");
  
  socrob_multicast::Long info;
  info.static_id = protocol_->SID();
  for (uint8_t i = 0; i < protocol_->NUM_SIDS(); i++) {
    if (protocol_->table().active (i)) {
      info.active_agents.push_back (i);
    }
  }
  marshall_ (info.data);
  serialize_overwrite (send_buffer_, info);
  compressor_->compress (send_buffer_);
  
  prepare_estimator_.add (microsec_clock::universal_time() - timer_.expires_at());
  
  timer_.expires_at (time_control_.prepare());
  ROS_DEBUG_STREAM_NAMED ("multicast.long", "Timer (send) set to: " << ros::Time::fromBoost (timer_.expires_at()));
  timer_.async_wait (boost::bind (&LongRoundManager::send,
                                  this,
                                  boost::asio::placeholders::error));
}



void
LongRoundManager::
send (const boost::system::error_code& error)
{
  if (error) {
    ROS_FATAL_STREAM_NAMED ("multicast.long", "Error in LongRoundManager::send : " << error.message());
    abort();
  }
  
  bool should_send = true;
  
  if (protocol_->shortActive().is_active()) {
    if (! short_active_last_time_) {
      should_send = false;
      ROS_DEBUG_NAMED ("multicast.long", "Not sending in long round because short round is active.");
    }
    short_active_last_time_ = true;
  }
  else {
    short_active_last_time_ = false;
  }
  
  // If should_send is already false, no need to consume bandwidth.
  if (should_send) {
    if (! protocol_->bandwidthManager().authorize (send_buffer_.size(), 1)) {
      should_send = false;
      ROS_WARN_NAMED ("multicast.long", "Size of long round packages is too big for bandwidth restrictions, not transmitting this time.");
    }
  }
  
  if (should_send) {
    channel_.send (send_buffer_);
    ROS_DEBUG_NAMED ("multicast.long", "Sent");
  }
  
  // Here is the place to add computation intensive stuff.
  prepare_estimator_.update();
  protocol_->table().update();
  
  timer_.expires_at (time_control_.send (protocol_->table().did (), protocol_->table().num_running()));
  ROS_DEBUG_STREAM_NAMED ("multicast.long", "Timer (control) set to: " << ros::Time::fromBoost (timer_.expires_at()));
  timer_.async_wait (boost::bind (&LongRoundManager::control,
                                  this,
                                  boost::asio::placeholders::error));
}



void
LongRoundManager::
receive_callback (boost::posix_time::ptime const& time,
                  std::vector<uint8_t> & buffer)
{
  compressor_->decompress (buffer);
  
  socrob_multicast::Long info;
  deserialize (info, buffer);
  
  if (0 != info.data.size()) {
    unmarshall_ (info.static_id, info.data);
  }
  
  protocol_->table().notify_reception (info.static_id, info.active_agents);
  
  if (RUNNING == protocol_->table().state (info.static_id)) {
    time_control_.reception (info.static_id, protocol_->table().did (info.static_id), protocol_->table().num_running(), time);
  }
}

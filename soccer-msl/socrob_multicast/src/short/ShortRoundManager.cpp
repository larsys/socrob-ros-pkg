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

#include "ShortRoundManager.h"

#include <list>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/console.h>

#include <socrob_multicast/serialization.h>

#include <socrob_multicast/MultipleShort.h>



using namespace std;
using namespace boost::posix_time;
using namespace socrob::multicast;
using boost::shared_ptr;



ShortRoundManager::
ShortRoundManager (shared_ptr<Protocol> const& protocol,
                   boost::asio::io_service& io_service,
                   string const& ADDRESS_STRING,
                   unsigned short PORT,
                   handler_function_type const& handler,
                   unsigned short_slot_us,
                   unsigned short_wait_us,
                   std::size_t max_mine_simultaneous,
                   boost::shared_ptr<Compressor> const& compressor) :
  short_slot_time_ (microseconds (short_slot_us)),
  short_wait_time_ (microseconds (short_wait_us)),
  compressor_ (compressor),
  protocol_ (protocol),
  active_ (false),
  active_mine_ (protocol, handler, max_mine_simultaneous),
  active_others_ (protocol, handler),
  timer_ (io_service),
  channel_ (io_service, ADDRESS_STRING, PORT, boost::bind (&ShortRoundManager::receive_callback, this, _1, _2))
{
}



void
ShortRoundManager::
activate()
{
  // Start business right away
  activate (microsec_clock::universal_time());
}



void
ShortRoundManager::
activate (ptime const& new_time,
          bool shorten_only)
{
  active_ = true;
  
  if (shorten_only) {
    if (timer_.expires_at() < new_time) {
      return;
    }
  }
  
  timer_.expires_at (new_time);
  timer_.async_wait (boost::bind (&ShortRoundManager::send,
                                  this,
                                  boost::asio::placeholders::error));
}



void
ShortRoundManager::
deactivate()
{
  // Delete whatever does not concearn me
  active_others_.delete_not_concearning();
  // Check is some terminated round concearning me was left to be deleted
  active_others_.check_running_all();
  
  active_ = false;
  
  timer_.cancel();
}



bool
ShortRoundManager::
startShortRound (vector<uint8_t> const& question,
                 set<id_type> const& required_sids,
                 short_callback_type const& callback)
{
  boost::lock_guard<boost::mutex> _ (mutex_);
  
  ROS_DEBUG_NAMED ("multicast.short.add", "--- Adding new short round");
  bool added = active_mine_.add (question, required_sids, callback);
  
  if (! active_) {
    ROS_DEBUG_NAMED ("multicast.short.add", "Activating short rounds send timer");
    active_mine_.check_running_all();
    active_others_.check_running_all();
    activate();
  }
  else {
    ROS_DEBUG_NAMED ("multicast.short.add", "Timer already active, not changing");
  }
  
  return added;
}



void
ShortRoundManager::
send (const boost::system::error_code& error)
{
  if (error == boost::asio::error::operation_aborted) {
    ROS_DEBUG_NAMED ("multicast.short.snd", "... Send callback called with operation_aborted error. Ignored, this means it was rescheduled.");
    return;
  }
  
  if (error) {
    ROS_FATAL_STREAM_NAMED ("multicast.short.snd", "Error in LongRoundManager::control : " << error.message());
    abort();
  }
  
  socrob_multicast::MultipleShort info;
  info.sender_sid = protocol_->SID();
  
  {
    boost::lock_guard<boost::mutex> _ (mutex_);
    
    ROS_DEBUG_NAMED ("multicast.short.snd", "^^^ Preparing short round message...");
    
    active_mine_.check_running_all();
    active_others_.check_running_all();
    
    if (active_mine_.empty() && active_others_.none_concearns_me()) {
      ROS_DEBUG_NAMED ("multicast.short.snd", "Canceling short rounds send timer after check_running because nothing that concearns me is active.");
      deactivate();
      return;
    }
    
    active_mine_.send (info);
    active_others_.send (info);
    
    if (active_mine_.empty() && active_others_.none_concearns_me()) {
      ROS_DEBUG_NAMED ("multicast.short.snd", "Canceling short rounds send timer, going to send last package with terminators and things that don't concearn me");
      deactivate();
    }
    else {
      ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "Rescheduling short rounds send timer to " << ros::Time::fromBoost (short_slot_time_) << " * " << protocol_->shortActive().num_active() << " = " << ros::Time::fromBoost (short_slot_time_ * protocol_->shortActive().num_active()) << " from now");
      activate (timer_.expires_at() + (short_slot_time_ * protocol_->shortActive().num_active()));
    }
    
    // This needs to be synchronized so that messages are published in the same order they are processed
    publish_if ("~debug_socrob_multicast", "debug_socrob_multicast/short_send", send_debug_publisher_, info);
  }
  
  vector<uint8_t> buffer;
  serialize_overwrite (buffer, info);
  compressor_->compress (buffer);
  if (protocol_->bandwidthManager().authorize (buffer.size())) {
    ROS_DEBUG_NAMED ("multicast.short.snd", "... Sending short round message");
    channel_.send (buffer);
  }
  else {
    ROS_DEBUG_NAMED ("multicast.short.snd", "... Not sent this time because of bandwidth restrictions.");
  }
}



void
ShortRoundManager::
receive_callback (boost::posix_time::ptime const& time,
                  std::vector<uint8_t> & buffer)
{
  compressor_->decompress (buffer);
  
  socrob_multicast::MultipleShort info;
  deserialize (info, buffer);
  
  {
    boost::lock_guard<boost::mutex> _ (mutex_);
    
    ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "vvv Receiving short round message from " << static_cast<int> (info.sender_sid));
    
    if (0 == info.questions.size()) {
      ROS_DEBUG_NAMED ("multicast.short.rcv", "Received package with empty questions, ignoring.");
      return;
    }
    
    // This needs to be synchronized so that messages are published in the same order they are processed
    publish_if ("~debug_socrob_multicast", "debug_socrob_multicast/short_receive", receive_debug_publisher_, info);
    
    foreach (socrob_multicast::Short & src, info.questions) {
      if (src.starter_id == protocol_->SID()) {
        active_mine_.receive (src);
      }
      else {
        active_others_.receive (src);
      }
    }
    
    // If not active and received something that does not concearn me, it will be deleted right away in the deactivate() call
    
    if (active_mine_.empty() && active_others_.none_concearns_me()) {
      // Nothing to do
      ROS_DEBUG_NAMED ("multicast.short.rcv", "Canceling short rounds send timer");
      deactivate();
    }
    else {
      if (! active_) {
        active_mine_.check_running_all();
        active_others_.check_running_all();
      }
      int distance = protocol_->shortActive().distance (info.sender_sid);
      if (1 == distance) {
        // I'm the next one to transmit
        ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "Making sure send timer is active at most the short wait time from now, since I'm the next one to transmit.");
        activate (time + short_wait_time_, active_);
      }
      else {
        // Reschedule with delays
        // If this is my own package, distance should be NUM_SIDS. But activate is in shorten_only mode.
        ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "Making sure send timer is active at most " << distance << " times the short slot time.");
        activate (time + (short_slot_time_ * distance), active_);
      }
    }
  }
}

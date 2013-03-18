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

#include <socrob_multicast/manager.h>

#include <boost/thread/thread.hpp>

#include <ros/console.h>

#include "common/Protocol.h"
#include "long/LongRoundManager.h"
#include "short/ShortRoundManager.h"



using namespace std;
using namespace socrob::multicast;
using boost::shared_ptr;



namespace
{
#include <errno.h>
#include <sched.h>



  void try_real_time()
  {
    struct sched_param proc_sched;
    proc_sched.sched_priority = 60;
    if (0 == sched_setscheduler (getpid(), SCHED_FIFO, &proc_sched)) {
      ROS_INFO_NAMED ("multicast", "Using real-time scheduler.");
    }
    else {
      ROS_INFO_NAMED ("multicast", "Failed to set real-time scheduler: %s", strerror (errno));
    }
  }
}



struct Manager::implementation {
  implementation (ManagerOptions const& options) :
    io_service_(),
    protocol_ (new Protocol (options.sid_,
                             options.num_sids_,
                             options.max_lost_,
                             options.online_handler_,
                             options.offline_handler_,
                             options.bytes_per_second_)),
    long_round_manager_ (protocol_,
                         io_service_,
                         options.multicast_address_,
                         options.multicast_long_port_,
                         options.long_marshall_,
                         options.long_unmarshall_,
                         options.tup_us_,
                         options.compressor_),
    short_round_manager_ (protocol_,
                          io_service_,
                          options.multicast_address_,
                          options.multicast_short_port_,
                          options.short_handler_,
                          options.short_slot_us_,
                          options.short_wait_us_,
                          options.short_max_mine_simultaneous_,
                          options.compressor_) {
  }
  
  
  boost::asio::io_service io_service_;
  shared_ptr<Protocol> protocol_;
  LongRoundManager long_round_manager_;
  ShortRoundManager short_round_manager_;
  
  // A thread pool with four threads. A sender and a receiver for short and long rounds. Enough.
  boost::thread t0;
  boost::thread t1;
  boost::thread t2;
  boost::thread t3;
};



Manager::
Manager (ManagerOptions const& options) :
  impl_ (new implementation (options))
{
  if (options.multicast_long_port_ == options.multicast_short_port_) {
    ROS_FATAL ("Multicast ports for long and short rounds have to be different.");
    abort();
  }
  
  try_real_time();
  
  impl_->t0 = boost::thread (boost::bind (&boost::asio::io_service::run, boost::ref (impl_->io_service_)));
  impl_->t1 = boost::thread (boost::bind (&boost::asio::io_service::run, boost::ref (impl_->io_service_)));
  impl_->t2 = boost::thread (boost::bind (&boost::asio::io_service::run, boost::ref (impl_->io_service_)));
  impl_->t3 = boost::thread (boost::bind (&boost::asio::io_service::run, boost::ref (impl_->io_service_)));
}



Manager::
~Manager ()
{
  impl_->io_service_.stop();
  impl_->t0.join();
  impl_->t1.join();
  impl_->t2.join();
  impl_->t3.join();
}



bool
Manager::
startShortRound (std::vector<uint8_t> const& question,
                 std::set<id_type> const& required_sids,
                 short_callback_type const& callback)
{
  return impl_->short_round_manager_.startShortRound (question, required_sids, callback);
}

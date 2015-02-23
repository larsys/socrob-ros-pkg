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

#ifndef _SOCROB_MULTICAST_TIMECONTROL_H_
#define _SOCROB_MULTICAST_TIMECONTROL_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <ros/console.h>

#include <socrob_multicast/definitions.h>



namespace socrob
{
  namespace multicast
  {
    class TimeControl
    {
      public:
        TimeControl (id_type sid,
                     unsigned num_sids,
                     unsigned tup_us) :
          SID_ (sid),
          NUM_SIDS_ (num_sids),
          TUP_ (boost::posix_time::microseconds (tup_us)),
          IGNORE_DELTA_ (boost::posix_time::milliseconds (4)),
          BETA_ (TUP_ / num_sids / 2 * sid / num_sids),
          MAX_DELTA_TXWIN_PERCENT_ (67),
          max_delta_ (TUP_ / 1 * MAX_DELTA_TXWIN_PERCENT_ / 100),
          round_time_ (boost::posix_time::microsec_clock::universal_time()),
#ifdef SOCROB_MULTICAST_LONG_SYNC_WORST
          send_time_ (ros::Time (static_cast<double> (1 + static_cast<long> (ros::Time::now().toSec()))).toBoost()),
#else
          send_time_ (boost::posix_time::microsec_clock::universal_time()),
#endif
          round_received_any_ (false),
          round_max_delta_ (boost::posix_time::microseconds (0)) {}
          
          
          
        /// To be run in the control handler. Returns the next send time.
        boost::posix_time::ptime
        control() {
          boost::lock_guard<boost::mutex> _ (mutex_);
          
#ifdef SOCROB_MULTICAST_LONG_SYNC_WORST
          send_time_ += TUP_;
          ROS_DEBUG_STREAM_NAMED ("multicast.long", "SOCROB_MULTICAST_LONG_SYNC_WORST: Send time updated with TUP_:" << ros::Time::fromBoost (TUP_));
#else
          if (round_received_any_) {
            send_time_ += TUP_ + round_max_delta_;
            ROS_DEBUG_STREAM_NAMED ("multicast.long", "Send time updated with TUP_:" << ros::Time::fromBoost (TUP_) << " + round_max_delta_:" << ros::Time::fromBoost (round_max_delta_));
          }
          else {
            send_time_ += TUP_ + max_delta_ + BETA_;
            ROS_DEBUG_STREAM_NAMED ("multicast.long", "Send time updated with TUP_:" << ros::Time::fromBoost (TUP_) << " + max_delta_:" << ros::Time::fromBoost (max_delta_) << " + BETA_:" << ros::Time::fromBoost (BETA_));
          }
#endif
          
          return send_time_;
        }
        
        
        
        /// To be run in the prepare handler. Returns the next send time.
        boost::posix_time::ptime
        prepare() {
          boost::lock_guard<boost::mutex> _ (mutex_);
          boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
          while (now > (send_time_ + TUP_)) {
#ifdef SOCROB_MULTICAST_LONG_SYNC_WORST
            send_time_ += TUP_;
#else
            send_time_ += TUP_ + max_delta_ + BETA_;
#endif
          }
          return send_time_;
        }
        
        
        
        /// To be run in the send handler. Returns the next control time.
        boost::posix_time::ptime
        send (id_type did, unsigned num_running) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          
          max_delta_ = TUP_ / num_running * MAX_DELTA_TXWIN_PERCENT_ / 100;
          round_time_ = send_time_ - (TUP_ * did / num_running);
          round_received_any_ = false;
          round_max_delta_ = boost::posix_time::microseconds (0);
          
          return send_time_ + (TUP_ * (NUM_SIDS_ - 1) / NUM_SIDS_) + max_delta_;
        }
        
        
        
        /// To be run in the reception handler.
        void
        reception (id_type sid, id_type did, unsigned num_running, boost::posix_time::ptime time) {
          if (SID_ == sid) {
            return;
          }
          
          boost::lock_guard<boost::mutex> _ (mutex_);
          
          boost::posix_time::ptime rcv_expected_time = round_time_ + (TUP_ * did / num_running);
          if (sid < SID_) {
            rcv_expected_time += TUP_;
          }
          
          if ( (time >= (rcv_expected_time - IGNORE_DELTA_))
               && (time <= (rcv_expected_time + max_delta_))) {
            round_received_any_ = true;
            boost::posix_time::time_duration delta = time - rcv_expected_time;
            
            if (delta > IGNORE_DELTA_) {
              round_max_delta_ = std::max (round_max_delta_, delta);
              ROS_DEBUG_STREAM_NAMED ("multicast.long", "Received. SID:" << static_cast<int> (sid)
                                      << " Round:" << ros::Time::fromBoost (round_time_)
                                      << " Expected:" << ros::Time::fromBoost (rcv_expected_time)
                                      << " Time:" << ros::Time::fromBoost (time)
                                      << " DELAY " << ros::Time::fromBoost (delta));
            }
            else {
              ROS_DEBUG_STREAM_NAMED ("multicast.long", "Received. SID:" << static_cast<int> (sid)
                                      << " Round:" << ros::Time::fromBoost (round_time_)
                                      << " Expected:" << ros::Time::fromBoost (rcv_expected_time)
                                      << " Time:" << ros::Time::fromBoost (time)
                                      << " Ignored:" << ros::Time::fromBoost (delta));
            }
          }
          else {
            ROS_DEBUG_STREAM_NAMED ("multicast.long", "Received. SID " << static_cast<int> (sid)
                                    << " Round:" << ros::Time::fromBoost (round_time_)
                                    << " Expected:" << ros::Time::fromBoost (rcv_expected_time)
                                    << " Time:" << ros::Time::fromBoost (time)
                                    << " OUT OF TIME " << ros::Time::fromBoost (time - rcv_expected_time));
          }
        }
        
        
        
      private:
        const id_type SID_;
        const unsigned NUM_SIDS_;
        const boost::posix_time::time_duration TUP_;
        const boost::posix_time::time_duration IGNORE_DELTA_;
        const boost::posix_time::time_duration BETA_;
        const int MAX_DELTA_TXWIN_PERCENT_; // Only integer arithmetics in time_duration
        
        boost::mutex mutex_;
        
        boost::posix_time::time_duration max_delta_;
        boost::posix_time::ptime round_time_;
        boost::posix_time::ptime send_time_;
        bool round_received_any_;
        boost::posix_time::time_duration round_max_delta_;
    };
  }
}

#endif

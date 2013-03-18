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

#ifndef _SOCROB_MULTICAST_SHORTACTIVE_H_
#define _SOCROB_MULTICAST_SHORTACTIVE_H_

#include <vector>
#include <sstream>

#include <boost/thread/mutex.hpp>

#include <ros/console.h>

#include <socrob_multicast/definitions.h>

#include "StateTable.h"



namespace socrob
{
  namespace multicast
  {
    class ShortActive
    {
      public:
        ShortActive (id_type sid,
                     unsigned num_sids) :
          SID_ (sid),
          NUM_SIDS_ (num_sids),
          number_active_ (NUM_SIDS_, 0),
          total_active_ (0) {}
          
          
          
        /// TODO: When the terminator for a round which I'm not part of is lost
        void increment_active (id_type sid) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          // ROS_DEBUG_STREAM_NAMED ("multicast.protocol", "short_number_active for sid " << static_cast<int> (sid) << " is " << short_number_active_[sid] << ", incrementing.");
          number_active_[sid]++;
          total_active_++;
        }
        
        void decrement_active (id_type sid) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          // ROS_DEBUG_STREAM_NAMED ("multicast.protocol", "short_number_active for sid " << static_cast<int> (sid) << " is " << short_number_active_[sid] << ", decrementing.");
          if (0 == number_active_[sid]) {
            ROS_FATAL ("Internal error: decrementing active for agent already with 0");
            abort();
          }
          number_active_[sid]--;
          total_active_--;
        }
        
        bool is_active() {
          boost::lock_guard<boost::mutex> _ (mutex_);
          return 0 != total_active_;
        }
        
        /// Always at least 1
        int
        distance (id_type from,
                  id_type to) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          // "from" might already be inactive when this is called,
          // however we still want to consider it.
          int distance = 1;
          for (std::size_t i = (from + 1) % NUM_SIDS_;
               i != to;
               i = (i + 1) % NUM_SIDS_) {
            if (number_active_[i]) {
              distance++;
            }
          }
          return distance;
        }
        
        int
        distance (id_type from) {
          return distance (from, SID_);
        }
        
        int num_active() {
          return distance (SID_, SID_);
        }
        
      private:
        const id_type SID_;
        const unsigned NUM_SIDS_;
        
        boost::mutex mutex_;
        
        std::vector<unsigned> number_active_;
        unsigned total_active_;
    };
  }
}

#endif

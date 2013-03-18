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

#ifndef _SOCROB_MULTICAST_BANDWIDTHMANAGER_H_
#define _SOCROB_MULTICAST_BANDWIDTHMANAGER_H_

#include <cstddef>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>



namespace socrob
{
  namespace multicast
  {
    class BandwidthManager
    {
      public:
        BandwidthManager (std::size_t bytes_per_second) :
          bytes_per_second_ (bytes_per_second),
          last_send_stamp_ (boost::date_time::min_date_time),
          last_priority_ (0) {}
          
        bool authorize (std::size_t bytes,
                        unsigned priority = 0) {
          if (0 == bytes_per_second_) {
            return true;
          }
          
          boost::posix_time::time_duration duration = boost::posix_time::microseconds (1000000) * bytes / bytes_per_second_;
          boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
          
          boost::lock_guard<boost::mutex> _ (mutex_);
          
          if (priority <= last_priority_) {
            if (last_send_stamp_ > now) {
              return false;
            }
          }
          
          last_send_stamp_ = std::max (now, last_send_stamp_ + duration);
          last_priority_ = priority;
          return true;
        }
        
      private:
        const std::size_t bytes_per_second_;
        
        boost::mutex mutex_;
        
        boost::posix_time::ptime last_send_stamp_;
        unsigned last_priority_;
    };
  }
}

#endif

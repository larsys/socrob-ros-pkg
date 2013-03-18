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

#ifndef _SOCROB_MULTICAST_PROTOCOL_H_
#define _SOCROB_MULTICAST_PROTOCOL_H_

#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <socrob_multicast/definitions.h>

#include "DynamicTable.h"
#include "BandwidthManager.h"
#include "ShortActive.h"



namespace socrob
{
  namespace multicast
  {
    class Protocol :
      private boost::noncopyable
    {
      public:
        Protocol (id_type sid,
                  unsigned num_sids,
                  unsigned max_lost,
                  id_function_type const& online_handler,
                  id_function_type const& offline_handler,
                  std::size_t bytes_per_second) :
          SID_ (sid),
          NUM_SIDS_ (num_sids),
          table_ (sid, num_sids, max_lost, online_handler, offline_handler),
          bandwidthManager_ (bytes_per_second),
          shortActive_ (sid, num_sids) {}
          
          
        id_type SID() const {
          return SID_;
        }
        
        unsigned NUM_SIDS() const {
          return NUM_SIDS_;
        }
        
        
        DynamicTable&
        table() {
          return table_;
        }
        
        BandwidthManager&
        bandwidthManager() {
          return bandwidthManager_;
        }
        
        ShortActive&
        shortActive() {
          return shortActive_;
        }
        
        
      private:
        const id_type SID_;
        const unsigned NUM_SIDS_;
        
        DynamicTable table_;
        BandwidthManager bandwidthManager_;
        ShortActive shortActive_;
    };
  }
}

#endif

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

#ifndef _SOCROB_MULTICAST_MANAGER_H_
#define _SOCROB_MULTICAST_MANAGER_H_

#include <string>
#include <set>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <socrob_multicast/definitions.h>
#include <socrob_multicast/compressor.h>



namespace socrob
{
  namespace multicast
  {
    class ManagerOptions
    {
      public:
        ManagerOptions (uint8_t sid,
                        uint8_t num_sids,
                        std::string const& multicast_address) :
          sid_ (sid),
          num_sids_ (num_sids),
          multicast_address_ (multicast_address),
          bytes_per_second_ (0),
          multicast_long_port_ (2000),
          multicast_short_port_ (2001),
          online_handler_ (empty_id_function),
          offline_handler_ (empty_id_function),
          long_marshall_ (empty_buffer_function),
          long_unmarshall_ (empty_id_buffer_function),
          short_handler_ (empty_handler_function),
          tup_us_ (100000),
          max_lost_ (10),
          short_slot_us_ (5000),
          short_wait_us_ (1000),
          short_max_mine_simultaneous_ (4),
          compressor_ (new Compressor()) {}
          
        ManagerOptions& bandwidth_bps (std::size_t bytes_per_second) {
          bytes_per_second_ = bytes_per_second;
          return *this;
        }
        
        ManagerOptions& ports (unsigned short long_port, unsigned short short_port) {
          multicast_long_port_ = long_port;
          multicast_short_port_ = short_port;
          return *this;
        }
        
        ManagerOptions& tup_ms (unsigned tup_ms) {
          tup_us_ = tup_ms * 1000;
          return *this;
        }
        
        ManagerOptions& tup_us (unsigned tup_us) {
          tup_us_ = tup_us;
          return *this;
        }
        
        ManagerOptions& max_lost (unsigned max_lost) {
          max_lost_ = max_lost;
          return *this;
        }
        
        ManagerOptions& short_slot_us (unsigned short_slot_us) {
          short_slot_us_ = short_slot_us;
          return *this;
        }
        
        ManagerOptions& short_wait_us (unsigned short_wait_us) {
          short_wait_us_ = short_wait_us;
          return *this;
        }
        
        ManagerOptions& short_max_mine_simultaneous (std::size_t short_max_mine_simultaneous) {
          short_max_mine_simultaneous_ = short_max_mine_simultaneous;
          return *this;
        }
        
        ManagerOptions& compressor (boost::shared_ptr<Compressor> const& compressor) {
          compressor_ = compressor;
          return *this;
        }
        
        ManagerOptions& online_handler (id_function_type const& online_handler) {
          online_handler_ = online_handler;
          return *this;
        }
        
        ManagerOptions& offline_handler (id_function_type const& offline_handler) {
          offline_handler_ = offline_handler;
          return *this;
        }
        
        /// All functions might be called simultaneously from different threads. long_unmarshall and short_handler might be called many times simultaneously.
        ManagerOptions& long_marshall (buffer_function_type const& long_marshall) {
          long_marshall_ = long_marshall;
          return *this;
        }
        
        /// All functions might be called simultaneously from different threads. long_unmarshall and short_handler might be called many times simultaneously.
        ManagerOptions& long_unmarshall (id_buffer_function_type const& long_unmarshall) {
          long_unmarshall_ = long_unmarshall;
          return *this;
        }
        
        /// Handler for short rounds might be called multiple times for the same question. If needed, include your own mechanisms to avoid problems.
        /// All functions might be called simultaneously from different threads.
        /// In short rounds, handler and callback might be called from inside startShortRound, be careful to release mutexes before calling.
        ManagerOptions& short_handler (handler_function_type const& short_handler) {
          short_handler_ = short_handler;
          return *this;
        }
        
      private:
        friend class Manager;
        
        uint8_t sid_;
        uint8_t num_sids_;
        std::string const& multicast_address_;
        std::size_t bytes_per_second_;
        unsigned short multicast_long_port_;
        unsigned short multicast_short_port_;
        id_function_type online_handler_;
        id_function_type offline_handler_;
        buffer_function_type long_marshall_;
        id_buffer_function_type long_unmarshall_;
        handler_function_type short_handler_;
        unsigned tup_us_;
        unsigned max_lost_;
        unsigned short_slot_us_;
        unsigned short_wait_us_;
        std::size_t short_max_mine_simultaneous_;
        boost::shared_ptr<Compressor> compressor_;
    };
    
    
    
    class Manager :
      private boost::noncopyable
    {
      public:
        Manager (ManagerOptions const& options);
        
        ~Manager();
        
        bool
        startShortRound (std::vector<uint8_t> const& question,
                         std::set<id_type> const& required_sids,
                         short_callback_type const& callback = empty_short_callback);
                         
        bool
        startShortRound (std::vector<uint8_t> const& question,
                         id_type required_sid,
                         short_callback_type const& callback = empty_short_callback) {
          std::set<id_type> required_sids;
          required_sids.insert (required_sid);
          return startShortRound (question, required_sids, callback);
        }
        
      private:
        struct implementation;
        boost::shared_ptr<implementation> impl_;
    };
  }
}

#endif

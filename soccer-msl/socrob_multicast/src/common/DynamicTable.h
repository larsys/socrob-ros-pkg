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

#ifndef _SOCROB_MULTICAST_DYNAMICTABLE_H_
#define _SOCROB_MULTICAST_DYNAMICTABLE_H_

#include <vector>
#include <sstream>

#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/console.h>

#include <socrob_multicast/definitions.h>

#include "StateTable.h"



namespace socrob
{
  namespace multicast
  {
    class DynamicTable :
      public StateTable
    {
      public:
        DynamicTable (id_type sid,
                      unsigned num_sids,
                      unsigned max_lost,
                      id_function_type const& online_handler,
                      id_function_type const& offline_handler) :
          StateTable (sid, num_sids),
          MAX_LOST_ (max_lost),
          online_handler_ (online_handler),
          offline_handler_ (offline_handler),
          did_ (0),
          dids_ (std::vector<id_type> (num_sids, 0)),
          num_running_ (1),
          lost_ (num_sids, 0) {}
          
          
          
        /// Returns the number of running robots with assigned dynamic ID
        unsigned num_running() {
          boost::lock_guard<boost::mutex> _ (mutex_);
          return num_running_;
        }
        
        /// Returns my dynamic ID
        id_type did() {
          boost::lock_guard<boost::mutex> _ (mutex_);
          return did_;
        }
        
        /// Returns the dynamic ID of agent with given static ID.
        /// This is only valid if that agent is in state RUNNING.
        id_type did (id_type sid) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          return dids_[sid];
        }
        
        
        
        void
        update() {
          boost::lock_guard<boost::mutex> _ (mutex_);
          
          lost_[SID()] = 0;
          
          unsigned num_running = 0;
          for (uint8_t i = 0; i < NUM_SIDS(); i++) {
            switch (state (i)) {
              case NOT_RUNNING:
                // Becomes INSERT when a packages is received
                break;
              case INSERT:
                // State change: INSERT -> REMOVE
                if (lost_[i] > MAX_LOST_) {
                  state (i, REMOVE);
                  checkRemove (i);
                }
                else {
                  checkInsert (i);
                }
                break;
              case RUNNING:
                dids_[i] = num_running;
                num_running++;
                
                // State change: RUNNING -> REMOVE
                if (lost_[i] > MAX_LOST_) {
                  state (i, REMOVE);
                  checkRemove (i);
                }
                break;
              case REMOVE:
                dids_[i] = num_running;
                num_running++;
                
                checkRemove (i);
                break;
            }
            
            if (NOT_RUNNING != state (i)) {
              lost_[i]++;
            }
          }
          
          num_running_ = num_running;
          did_ = dids_[SID()];
          
#if 0
          for (id_type i = 0; i < NUM_SIDS(); i++) {
            std::ostringstream debug;
            debug << "Vector from agent " << static_cast<int> (i) << " :";
            for (id_type j = 0; j < NUM_SIDS(); j++) {
              if (active (i, j)) {
                debug << " A";
              }
              else {
                debug << " -";
              }
            }
            ROS_DEBUG_STREAM_NAMED ("multicast.protocol", debug.str());
          }
#endif
        }
        
        
        
        void
        notify_reception (id_type from,
                          std::vector<id_type> const& rcv_active) {
          if (SID() == from) {
            return;
          }
          if (from >= NUM_SIDS()) {
            ROS_FATAL ("Received message with Static ID greater than maximum");
            abort();
          }
          
          boost::lock_guard<boost::mutex> _ (mutex_);
          
          clear_active (from);
          foreach (id_type agent, rcv_active) {
            active (from, agent, true);
          }
          
          lost_[from] = 0;
          // State change: REMOVE -> INSERT
          // State change: NOT_RUNNING -> INSERT
          if (! active (from)) {
            state (from, INSERT);
            // Cannot checkInsert right here, num_running_ and dids_ would get inconsistent.
            online_handler_ (from);
          }
        }
        
        
        
      private:
        const unsigned MAX_LOST_;
        
        const id_function_type online_handler_;
        const id_function_type offline_handler_;
        
        boost::mutex mutex_;
        
        id_type did_;
        std::vector<id_type> dids_; // Make sure agent is in state RUNNING before accessing this
        unsigned num_running_;
        std::vector<unsigned> lost_;
        
        void
        checkInsert (uint8_t agent) {
          // State change: INSERT -> RUNNING
          bool upgrade_to_running = true;
          for (unsigned j = 0; j < NUM_SIDS(); j++) {
            if (state (j) != RUNNING) {
              continue;
            }
            if (! active (j, agent)) {
              upgrade_to_running = false;
              break;
            }
          }
          if (upgrade_to_running) {
            state (agent, RUNNING);
          }
        }
        
        void
        checkRemove (uint8_t agent) {
          // State change: REMOVE -> NOT_RUNNING
          bool upgrade_to_not_running = true;
          for (unsigned j = 0; j < NUM_SIDS(); j++) {
            if (state (j) != RUNNING) {
              continue;
            }
            if (active (j, agent)) {
              upgrade_to_not_running = false;
              break;
            }
          }
          if (upgrade_to_not_running) {
            state (agent, NOT_RUNNING);
            clear_active (agent);
            lost_[agent] = 0;
            offline_handler_ (agent);
          }
        }
    };
  }
}

#endif

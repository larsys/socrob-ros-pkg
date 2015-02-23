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

#ifndef _SOCROB_MULTICAST_STATETABLE_H_
#define _SOCROB_MULTICAST_STATETABLE_H_

#include <vector>

#include <boost/thread.hpp>

#include <ros/console.h>

#include <socrob_multicast/definitions.h>



namespace socrob
{
  namespace multicast
  {
    class StateTable
    {
      public:
        id_type SID() const {
          return SID_;
        }
        
        unsigned NUM_SIDS() const {
          return NUM_SIDS_;
        }
        
        bool active (id_type who) {
          return active (SID_, who);
        }
        
        bool active (id_type where, id_type who) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          return active_agents_[where][who];
        }
        
        agent_state_type state (id_type who) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          return state_vector_[who];
        }
        
      protected:
        StateTable (id_type sid,
                    unsigned num_sids) :
          SID_ (sid),
          NUM_SIDS_ (num_sids),
          active_agents_ (num_sids, std::vector<bool> (num_sids, false)),
          state_vector_ (num_sids, NOT_RUNNING) {
          if (sid >= num_sids) {
            ROS_FATAL ("Initializing Protocol: Static ID cannot be greater than Maximum Static ID");
            abort();
          }
          state (sid, RUNNING);
        }
        
        void clear_active (id_type agent) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          active_agents_[agent] = std::vector<bool> (NUM_SIDS(), false);
        }
        
        void active (id_type where, id_type who, bool state) {
          if (SID_ == where) {
            ROS_FATAL ("Use state() to change state on own vector");
            abort();
          }
          
          boost::lock_guard<boost::mutex> _ (mutex_);
          active_agents_[where][who] = state;
        }
        
        void state (id_type who, agent_state_type state) {
          boost::lock_guard<boost::mutex> _ (mutex_);
          
          state_vector_[who] = state;
          switch (state) {
            case INSERT:
            case RUNNING:
              active_agents_[SID_][who] = true;
              break;
            case REMOVE:
            case NOT_RUNNING:
              active_agents_[SID_][who] = false;
              break;
          }
        }
        
      private:
        const id_type SID_;
        const unsigned NUM_SIDS_;
        
        boost::mutex mutex_;
        
        std::vector<std::vector<bool> > active_agents_;
        std::vector<agent_state_type> state_vector_;
    };
  }
}

#endif

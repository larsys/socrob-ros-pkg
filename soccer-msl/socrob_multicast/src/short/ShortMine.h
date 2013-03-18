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

#ifndef _SOCROB_MULTICAST_SHORTMINE_H_
#define _SOCROB_MULTICAST_SHORTMINE_H_

#include <vector>
#include <set>
#include <map>
#include <list>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/console.h>

#include <socrob_multicast/definitions.h>

#include <socrob_multicast/MultipleShort.h>

#include "../common/Helpers.h"

#include "../common/Protocol.h"

#include "ShortDataMine.h"



namespace socrob
{
  namespace multicast
  {
    class ShortMine :
      private boost::noncopyable
    {
      public:
        ShortMine (boost::shared_ptr<Protocol> const& protocol,
                   handler_function_type const& handler,
                   std::size_t max_simultaneous) :
          protocol_ (protocol),
          handler_ (handler),
          max_simultaneous_ (max_simultaneous),
          next_qid_ (0) {}
          
          
          
        bool empty() const {
          return 0 == active_mine_.size();
        }
        
        
        
        bool
        add (std::vector<uint8_t> const& question,
             std::set<id_type> const& required_sids,
             short_callback_type const& callback) {
          if (active_mine_.size() >= max_simultaneous_) {
            //ROS_ERROR_NAMED ("multicast.short.add", "ShortMine::add: Too many active short rounds, ignoring new one, callback will not be called, returning false.");
            return false;
          }
          
          qid_type qid = next_qid_++;
          
          if (0 != active_mine_.count (qid)) {
            //ROS_ERROR_NAMED ("multicast.short.add", "ShortMine::add: Refusing to create short round with duplicate qid, ignoring new one, callback will not be called, returning false.");
            return false;
          }
          
          ShortDataMine local_data (protocol_, question, required_sids, callback);
          
          // Check if I am supposed to answer
          if (0 != required_sids.count (protocol_->SID())) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.add", "ShortMine::add: Calling handler to calculate answer for my own question.");
            std::vector<uint8_t> answer;
            std::vector<uint8_t> question_copy (question);
            handler_ (answer, protocol_->SID(), question_copy);
            local_data.add_answer_swap (protocol_->SID(), answer);
          }
          
          // Check if I am the only one supposed to answer
          // or required is not active
          if (local_data.complete()) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.add", "ShortMine::add: Calling callback for my question right away, since I was the only one required or required not active.");
            local_data.call_callback();
            return true;
          }
          
          active_mine_.insert (std::make_pair (qid, local_data));
          
          ROS_DEBUG_STREAM_NAMED ("multicast.short.add", "ShortMine::add: Added new short round with qid " << static_cast<int> (qid));
          
          return true;
        }
        
        
        
        void
        check_running_all() {
          foreach (active_mine_type::value_type & local_data_pair, active_mine_) {
            qid_type qid = local_data_pair.first;
            ShortDataMine& local_data = local_data_pair.second;
            
            // Check if completed before
            if (local_data.complete()) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortMine::check_running_all: Question " << static_cast<int> (qid) << " already complete, leaving in list so that terminator will be sent.");
              continue;
            }
            
            local_data.check_running();
            
            // Check if completed now
            if (local_data.complete()) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortMine::check_running_all: Some required agents no longer active, calling callback for question " << static_cast<int> (qid));
              local_data.call_callback();
            }
          }
        }
        
        
        
        void
        send (socrob_multicast::MultipleShort& info) {
          std::size_t pos = info.questions.size();
          info.questions.resize (info.questions.size() + active_mine_.size());
          
          std::list<qid_type> erase_mine;
          foreach (active_mine_type::value_type & local_data_pair, active_mine_) {
            qid_type qid = local_data_pair.first;
            ShortDataMine& local_data = local_data_pair.second;
            socrob_multicast::Short& package_data = info.questions[pos++];
            
            package_data.starter_id = protocol_->SID();
            package_data.qid = qid;
            
            if (! local_data.complete()) {
              // At this point, required should always contain something unless...
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortMine::send: Requiring answer for question " << static_cast<int> (qid) << " from agents " << toStr (local_data.required()));
              local_data.question (package_data.question);
              local_data.required (package_data.required);
            }
            else {
              // ...unless it is the last package signaling completion
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortMine::send: Sending terminator for question " << static_cast<int> (qid) << " and erasing from list");
              erase_mine.push_back (qid);
            }
          }
          foreach (qid_type id, erase_mine) {
            active_mine_.erase (id);
          }
        }
        
        
        
        void
        receive (socrob_multicast::Short& package_data) {
          if ( (0 == package_data.required.size()) && (0 == package_data.answers.size())) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortMine::receive: Received my own terminator package for question " << static_cast<int> (package_data.qid) << ", ignoring.");
            return;
          }
          
          // Check if question is still active
          active_mine_type::iterator q_it = active_mine_.find (package_data.qid);
          if (q_it == active_mine_.end()) {
            // Send another terminator
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortMine::receive: Received from inactive question " << static_cast<int> (package_data.qid) << ", scheduling resend of terminator.");
            ShortDataMine local_data (protocol_);
            active_mine_.insert (std::make_pair (package_data.qid, local_data));
            return;
          }
          // id_type qid = q_it->first;
          ShortDataMine& local_data = q_it->second;
          
          // Check if completed and terminated before
          if (local_data.complete()) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortMine::receive: Received duplicate for already completed question " << static_cast<int> (package_data.qid));
            return;
          }
          
          // Get new answers
          foreach (socrob_multicast::Answer & answer, package_data.answers) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortMine::receive: Saving answer from " << static_cast<int> (answer.sid) << " for question " << static_cast<int> (package_data.qid));
            local_data.add_answer_swap (answer.sid, answer.data);
          }
          
          // Check if completed now
          if (local_data.complete()) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortMine::receive: Calling callback for question " << static_cast<int> (package_data.qid));
            local_data.call_callback();
            local_data.terminate();
          }
          else {
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortMine::receive: Question " << static_cast<int> (package_data.qid) << " will remain active.");
          }
        }
        
        
        
      private:
        boost::shared_ptr<Protocol> protocol_;
        
        const handler_function_type handler_;
        
        const std::size_t max_simultaneous_;
        
        typedef std::map<qid_type, ShortDataMine> active_mine_type;
        active_mine_type active_mine_;
        
        qid_type next_qid_;
    };
  }
}

#endif

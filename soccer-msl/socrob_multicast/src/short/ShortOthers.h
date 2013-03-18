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

#ifndef _SOCROB_MULTICAST_SHORTOTHERS_H_
#define _SOCROB_MULTICAST_SHORTOTHERS_H_

#include <vector>
#include <list>
#include <set>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <socrob_multicast/MultipleShort.h>

#include <socrob_multicast/definitions.h>

#include "../common/Helpers.h"

#include "../common/Protocol.h"

#include "ShortDataOthers.h"



namespace socrob
{
  namespace multicast
  {
    class ShortOthers :
      private boost::noncopyable
    {
      public:
        ShortOthers (boost::shared_ptr<Protocol> const& protocol,
                     handler_function_type const& handler) :
          protocol_ (protocol),
          handler_ (handler) {}
          
          
          
        bool
        empty() const {
          return 0 == active_others_.size();
        }
        
        
        
        bool
        none_concearns_me() const {
          foreach (active_others_type::value_type const & local_data_pair, active_others_) {
            if (local_data_pair.second.concearns_me()) {
              return false;
            }
          }
          return true;
        }
        
        
        
        void
        check_running_all() {
          std::list<pair_id_type> erase;
          foreach (active_others_type::value_type & local_data_pair, active_others_) {
            std::pair<id_type, qid_type> pair_id = local_data_pair.first;
            id_type starter_id = pair_id.first;
            qid_type qid = pair_id.second;
            ShortDataOthers& local_data = local_data_pair.second;
            
            // If the starter_id is not running, it can be immediately deleted.
            if (NOT_RUNNING == protocol_->table().state (starter_id)) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortOthers::check_running_all: Starter of question " << static_cast<int> (qid) << " from sid " << static_cast<int> (starter_id) << " no longer running, deleting.");
              erase.push_back (pair_id);
              continue;
            }
            
            // If it was left to be deleted, delete.
            if (local_data.terminated()) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortOthers::check_running_all: Question " << static_cast<int> (qid) << " from sid " << static_cast<int> (starter_id) << " left to be deleted at send time, deleting.");
              erase.push_back (pair_id);
              continue;
            }
            
            // Else, leave it, will be deleted when the terminator arrives.
            local_data.check_running();
          }
          foreach (pair_id_type const & pair_id, erase) {
            active_others_.erase (pair_id);
          }
        }
        
        
        
        void
        send (socrob_multicast::MultipleShort& package_multiple) {
          std::size_t pos = package_multiple.questions.size();
          package_multiple.questions.resize (package_multiple.questions.size() + active_others_.size());
          
          foreach (active_others_type::value_type & local_data_pair, active_others_) {
            id_type starter_id = local_data_pair.first.first;
            qid_type qid = local_data_pair.first.second;
            ShortDataOthers& local_data = local_data_pair.second;
            socrob_multicast::Short& package_data = package_multiple.questions[pos++];
            
            // Even if this question does not concern me, if I have a slot in
            // short rounds I will transmit what I know.
            
            package_data.starter_id = starter_id;
            package_data.qid = qid;
            if (! local_data.complete()) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortOthers::send: Requiring answer for question " << static_cast<int> (qid) << " from sid " << static_cast<int> (starter_id) << " from agents " << toStr (local_data.required()));
              local_data.question (package_data.question);
              local_data.required (package_data.required);
            }
            
            package_data.answers.resize (local_data.answers().size());
            std::size_t a_pos = 0;
            foreach (ShortData::answers_type::value_type const & a_src, local_data.answers()) {
              socrob_multicast::Answer& a_dst = package_data.answers[a_pos++];
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortOthers::send: Added answer from " << static_cast<int> (a_src.first) << " for question " << static_cast<int> (qid) << " from sid " << static_cast<int> (starter_id));
              a_dst.sid = a_src.first;
              a_dst.data = a_src.second;
            }
          }
        }
        
        
        
        void
        delete_not_concearning() {
          std::list<pair_id_type> erase;
          foreach (active_others_type::value_type & local_data_pair, active_others_) {
            std::pair<id_type, qid_type> pair_id = local_data_pair.first;
            id_type starter_id = pair_id.first;
            qid_type qid = pair_id.second;
            ShortDataOthers& local_data = local_data_pair.second;
            
            if (! local_data.concearns_me()) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.snd", "ShortOthers::delete_not_concearning: Question " << static_cast<int> (qid) << " from sid " << static_cast<int> (starter_id) << " does not concearn me, deleting.");
              erase.push_back (pair_id);
            }
          }
          foreach (pair_id_type const & pair_id, erase) {
            active_others_.erase (pair_id);
          }
        }
        
        
        
        void
        receive (socrob_multicast::Short& src) {
          pair_id_type pair_id (src.starter_id, src.qid);
          
          // Ignore agents not yet active, online_handler must be called before evaluating questions.
          if (NOT_RUNNING == protocol_->table().state (src.starter_id)) {
            return;
          }
          
          active_others_type::iterator q_it = active_others_.find (pair_id);
          if (q_it == active_others_.end()) {
            // Check if it is a terminator package
            if ( (src.required.size() == 0) && (src.answers.size() == 0)) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Ignoring received terminator for question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id));
              return;
            }
            
            // New question
            // Note: that when a new question is received, either:
            // it is for me -> I am in required_sids, and since required_sids is not empty the question must be present
            // is is not for me -> might have missed the first package and this is already an answer, which might not have a question. No problem, not going to need it anyway.
            ShortDataOthers local_data (protocol_, src.starter_id, src.question, std::set<id_type> (src.required.begin(), src.required.end()));
            
            foreach (socrob_multicast::Answer & answer, src.answers) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Saving answer from " << static_cast<int> (answer.sid) << " for new question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id));
              local_data.add_answer_swap (answer.sid, answer.data);
            }
            
            if (0 != local_data.answers().count (protocol_->SID())) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Recevided new question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id) << " with my answer, therefore complete and not saved.");
              return;
            }
            
            if (0 != local_data.required().count (protocol_->SID())) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Calling handler to calculate answer for question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id) << ".");
              std::vector<uint8_t> answer;
              std::vector<uint8_t> question_copy (src.question);
              handler_ (answer, src.starter_id, question_copy);
              local_data.add_answer_swap (protocol_->SID(), answer);
            }
            else {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Saved question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id) << " not for me, not calculating answer.");
            }
            
            active_others_.insert (std::make_pair (pair_id, local_data));
          }
          else {
            // Already exists, update new answers
            ShortDataOthers& local_data = q_it->second;
            
            // Check if it is a terminator package
            if ( (src.required.size() == 0) && (src.answers.size() == 0)) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Received terminator for question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id) << ", leaving to be deleted before send.");
              local_data.terminate();
              return;
            }
            
            // Check if it is simply left to be deleted
            if (local_data.terminated()) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Received something for question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id) << " which is already terminated, ignoring.");
              return;
            }
            
            ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Received something for already existing question " << static_cast<int> (src.qid) << " from sid " << static_cast<int> (src.starter_id));
            
            foreach (socrob_multicast::Answer & answer, src.answers) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short.rcv", "ShortOthers::receive: Saving answer from " << static_cast<int> (answer.sid) << ".");
              local_data.add_answer_swap (answer.sid, answer.data);
            }
          }
        }
        
        
        
      private:
        boost::shared_ptr<Protocol> protocol_;
        
        const handler_function_type handler_;
        
        typedef std::pair<id_type, qid_type> pair_id_type;
        typedef std::map<pair_id_type, ShortDataOthers> active_others_type;
        active_others_type active_others_;
    };
  }
}

#endif

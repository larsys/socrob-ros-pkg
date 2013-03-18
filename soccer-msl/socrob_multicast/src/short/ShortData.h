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

#ifndef _SOCROB_MULTICAST_SHORTDATA_H_
#define _SOCROB_MULTICAST_SHORTDATA_H_

#include <vector>
#include <list>
#include <set>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/console.h>

#include <socrob_multicast/definitions.h>

#include "../common/Protocol.h"



namespace socrob
{
  namespace multicast
  {
    class ShortData
    {
      public:
        typedef std::map<id_type, std::vector<uint8_t> > answers_type;
        
        
        
        ShortData (boost::shared_ptr<Protocol> const& protocol,
                   id_type starter_id,
                   std::vector<uint8_t> const& question,
                   std::set<id_type> const& required_sids) :
          protocol_ (protocol),
          starter_id_ (starter_id),
          question_ (question),
          required_sids_ (required_sids),
          answers_() {
          protocol_->shortActive().increment_active (starter_id_);
          foreach (id_type id, required_sids_) {
            // ROS_DEBUG_STREAM_NAMED ("multicast.short", "ShortData::Constructor: incrementing for sid " << static_cast<int> (id));
            protocol_->shortActive().increment_active (id);
          }
          
          check_running();
        }
        
        
        
        ShortData (const ShortData& source) :
          protocol_ (source.protocol_),
          starter_id_ (source.starter_id_),
          question_ (source.question_),
          required_sids_ (source.required_sids_),
          answers_ (source.answers_) {
          protocol_->shortActive().increment_active (starter_id_);
          foreach (id_type id, required_sids_) {
            // ROS_DEBUG_STREAM_NAMED ("multicast.short", "ShortData::Copy constructor: incrementing for sid " << static_cast<int> (id));
            protocol_->shortActive().increment_active (id);
          }
          
          check_running();
        }
        
        
        
        ~ShortData() {
          protocol_->shortActive().decrement_active (starter_id_);
          foreach (id_type id, required_sids_) {
            // ROS_DEBUG_STREAM_NAMED ("multicast.short", "ShortData::Destructor: decrementing for sid " << static_cast<int> (id));
            protocol_->shortActive().decrement_active (id);
          }
        }
        
        
        
        void
        question (std::vector<uint8_t>& question) const {
          question = question_;
        }
        
        std::vector<uint8_t> const&
        question() const {
          return question_;
        }
        
        
        
        void
        required (std::vector<id_type>& required) const {
          required.clear();
          required.reserve (required_sids_.size());
          required.insert (required.end(), required_sids_.begin(), required_sids_.end());
        }
        
        std::set<id_type> const&
        required() const {
          return required_sids_;
        }
        
        
        
        answers_type const&
        answers() const {
          return answers_;
        }
        
        
        
        bool
        complete() const {
          return 0 == required_sids_.size();
        }
        
        bool
        terminated() const {
          return (0 == required_sids_.size())
                 && (0 == answers_.size());
        }
        
        
        
        void
        terminate() {
          required_sids_.clear();
          answers_.clear();
        }
        
        
        
        void
        add_answer_swap (id_type sid,
                         std::vector<uint8_t>& answer) {
          if (0 != answers_.count (sid)) {
            // Duplicate, ignoring.
            return;
          }
          
          answer.swap (answers_[sid]);
          
          if (0 != required_sids_.erase (sid)) {
            ROS_DEBUG_STREAM_NAMED ("multicast.short", "ShortData::add_answer_swap: decrementing for sid " << static_cast<int> (sid));
            protocol_->shortActive().decrement_active (sid);
          }
        }
        
        
        
        /// Does not check the starter_id, must be done by the owner since this instance might have to be erased
        void
        check_running() {
          std::list<id_type> erase;
          foreach (id_type sid, required_sids_) {
            if (NOT_RUNNING == protocol_->table().state (sid)) {
              erase.push_back (sid);
            }
          }
          foreach (id_type sid, erase) {
            if (0 != required_sids_.erase (sid)) {
              ROS_DEBUG_STREAM_NAMED ("multicast.short", "ShortData::check_running: decrementing for sid " << static_cast<int> (sid));
              protocol_->shortActive().decrement_active (sid);
            } // else something stange has happened
          }
        }
        
        
        
      protected:
        boost::shared_ptr<Protocol> protocol_;
        
        
        
        answers_type&
        mut_answers() {
          return answers_;
        }
        
        
        
      private:
        id_type starter_id_;
        
        // Attention: In case of ShortDataOthers:
        // The first received package might not contain a question, however,
        // in that case, it is not for us and we don't care about the question.
        // If the question should be saved anyway, this will stop being const.
        const std::vector<uint8_t> question_;
        
        std::set<id_type> required_sids_;
        
        answers_type answers_;
        
        // This class is copyable, but only by using the copy constructor.
        // operator= does not make sense because question_ is const.
        ShortData& operator= (const ShortData& source);
    };
  }
}

#endif

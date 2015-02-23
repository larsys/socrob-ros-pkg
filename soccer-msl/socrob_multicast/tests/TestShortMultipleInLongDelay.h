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

#ifndef TESTSHORTMULTIPLEINLONGDELAY_HPP
#define TESTSHORTMULTIPLEINLONGDELAY_HPP

#include <iostream>
#include <fstream>
#include <limits>

#include <boost/thread.hpp>
#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/ros.h>

#include <socrob/multicast.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Time.h>



using namespace std;
using namespace ros;
using namespace socrob::multicast;
using boost::lexical_cast;



class TestShortMultipleInLongDelay
{
    boost::mutex mutex_;
    
    int sid_;
    
    const uint8_t QUESTION_MAGIC_BYTE_;
    const uint8_t ANSWER_MAGIC_BYTE_;
    
    bool waiting_;
    unsigned round_number_;
    unsigned round_uid_;
    Time round_tx_;
    map<id_type, Time> round_answers_;
    
    bool answer_;
    unsigned answer_number_;
    unsigned answer_uid_;
    Time answer_time_;
    
    long double acc_max_arrival_delay_;
    long double acc_avg_arrival_delay_;
    long double acc_complete_delay_;
    unsigned long num_rounds_;
    
    boost::variate_generator<boost::mt19937, boost::uniform_int<unsigned> > random_uid_;
    
    Time output_time_base_;
    unsigned output_line_;
    ofstream output_;
    
  public:
    TestShortMultipleInLongDelay (int sid,
                                  string const& type) :
      sid_ (sid),
      QUESTION_MAGIC_BYTE_ (0),
      ANSWER_MAGIC_BYTE_ (1),
      waiting_ (false),
      round_number_ (0),
      round_uid_(),
      round_tx_ (Time::now()),
      answer_ (false),
      acc_max_arrival_delay_ (0),
      acc_avg_arrival_delay_ (0),
      acc_complete_delay_ (0),
      num_rounds_ (0),
      random_uid_ (boost::mt19937 (Time::now().toNSec()),
                   boost::uniform_int<unsigned> (numeric_limits<unsigned>::min(), numeric_limits<unsigned>::max())),
      output_time_base_ (Time::now()) {
      if (0 != sid_) {
        return;
      }
      
      waiting_ = true;
      round_number_++;
      round_uid_ = random_uid_();
      round_tx_ = Time::now();
      round_answers_.clear();
      
      output_.open ( ("/tmp/TestShortMultipleInLongDelay_" + type + "_" + lexical_cast<string> (output_time_base_.sec) + ".csv").c_str());
      output_ << "\"Round Number\""
              << ";\"Handler Delay 1\""
              << ";\"Handler Delay 2\""
              << ";\"Handler Delay 3\""
              << ";\"Handler Delay 4\""
              << ";\"Callback Delay\""
              << endl;
      output_line_ = 1;
    }
    
    
    
    ~TestShortMultipleInLongDelay() {
      unsigned l = output_line_;
      output_ << ";;;" << endl;
      output_ << "\"Average Handler Delay:\";=AVERAGE(B2:E" << l << ")" << endl;
      output_ << "\"Average Callback Delay:\";=AVERAGE(F2:F" << l << ")" << endl;
    }
    
    
    
    void
    timer () {
      if (0 != sid_) {
        return;
      }
      
      boost::lock_guard<boost::mutex> _ (mutex_);
      
      if (waiting_) {
        Duration age = Time::now() - round_tx_;
        if (age > Duration (1, 0)) {
          ROS_WARN_STREAM_THROTTLE (1, "Last round (" << round_number_ << ") was created " << age << " seconds ago, still waiting...");
        }
        return;
      }
    }
    
    
    
    void
    long_marshall (vector<uint8_t> & buffer) {
      boost::lock_guard<boost::mutex> _ (mutex_);
      
      if (answer_) {
        std_msgs::UInt8 magic_msg;
        magic_msg.data = ANSWER_MAGIC_BYTE_;
        serialize_append (buffer, magic_msg);
        
        std_msgs::UInt32 number_msg;
        number_msg.data = answer_number_;
        serialize_append (buffer, number_msg);
        
        std_msgs::UInt32 uid_msg;
        uid_msg.data = answer_uid_;
        serialize_append (buffer, uid_msg);
        
        std_msgs::UInt8 sid_msg;
        sid_msg.data = sid_;
        serialize_append (buffer, sid_msg);
        
        std_msgs::Time time_msg;
        time_msg.data = answer_time_;
        serialize_append (buffer, time_msg);
        
        answer_ = false;
        return;
      }
      
      if ( (0 == sid_)
           && (! waiting_)) {
        waiting_ = true;
        round_number_++;
        round_uid_ = random_uid_();
        round_tx_ = Time::now();
        round_answers_.clear();
        
        // To simulate worst case, not sending this time.
        return;
      }
      
      if (! waiting_) {
        return;
      }
      
      std_msgs::UInt8 magic_msg;
      magic_msg.data = QUESTION_MAGIC_BYTE_;
      serialize_append (buffer, magic_msg);
      
      std_msgs::UInt32 number_msg;
      number_msg.data = round_number_;
      serialize_append (buffer, number_msg);
      
      std_msgs::UInt32 uid_msg;
      uid_msg.data = round_uid_;
      serialize_append (buffer, uid_msg);
    }
    
    
    
    void
    long_unmarshall (socrob::multicast::id_type,
                     vector<uint8_t> & buffer) {
      size_t offset = 0;
      
      std_msgs::UInt8 magic_msg;
      offset += deserialize (magic_msg, buffer, offset);
      
      if (magic_msg.data == 0) {
        std_msgs::UInt32 number_msg;
        offset += deserialize (number_msg, buffer, offset);
        std_msgs::UInt32 uid_msg;
        offset += deserialize (uid_msg, buffer, offset);
        
        boost::lock_guard<boost::mutex> _ (mutex_);
        
        answer_ = true;
        answer_number_ = number_msg.data;
        answer_uid_ = uid_msg.data;
        answer_time_ = Time::now();
      }
      else if (magic_msg.data == 1) {
        if (0 != sid_) {
          return;
        }
        
        boost::lock_guard<boost::mutex> _ (mutex_);
        
        if (! waiting_) {
          ROS_WARN_STREAM ("Received package while short rounds not active");
          return;
        }
        
        std_msgs::UInt32 number_msg;
        offset += deserialize (number_msg, buffer, offset);
        std_msgs::UInt32 uid_msg;
        offset += deserialize (uid_msg, buffer, offset);
        std_msgs::UInt8 sid_msg;
        offset += deserialize (sid_msg, buffer, offset);
        std_msgs::Time arrival_time_msg;
        offset += deserialize (arrival_time_msg, buffer, offset);
        
        unsigned sent_number = number_msg.data;
        unsigned sent_uid = uid_msg.data;
        id_type sid = sid_msg.data;
        
        if (sid != 1 && sid != 2 && sid != 3 && sid != 4) {
          ROS_FATAL_STREAM ("Received answer from wrong agent while testing short rounds multiple.");
          abort();
        }
        if (sent_number != round_number_) {
          ROS_WARN ("Received answer wrong round number while testing short rounds single.");
          return;
        }
        if (sent_uid != round_uid_) {
          ROS_WARN ("Received answer with right number but wrong UID while testing short rounds single.");
          return;
        }
        
        if (0 == round_answers_.count (sid)) {
          round_answers_[sid] = arrival_time_msg.data;
        }
        
        if (round_answers_.size() == 4) {
          all_answers ();
          waiting_ = false;
        }
      }
      else {
        ROS_FATAL_STREAM ("Received package with wrong magic byte.");
        abort();
      }
    }
    
    
    
  private:
    void
    all_answers () {
      Time now = Time::now();
      
      long double avg_arrival_delay = 0;
      long double max_arrival_delay = 0;
      
      /*unsigned l =*/
      ++ (output_line_);
      output_ << round_number_;
      
      typedef map<id_type, Time>::value_type tmp_t;
      foreach (tmp_t const & answer_pair, round_answers_) {
        Time const& ans_arrival_time = answer_pair.second;
        
        Duration arrival_delay = ans_arrival_time - round_tx_;
        
        output_ << ";" << arrival_delay;
        
        long double arrival_delay_ld = (long double) arrival_delay.sec + 1e-9 * (long double) arrival_delay.nsec;
        avg_arrival_delay += arrival_delay_ld / 4;
        max_arrival_delay = max (max_arrival_delay, arrival_delay_ld);
      }
      
      Duration complete_delay = now - round_tx_;
      
      output_ << ";" << complete_delay << endl;
      
      ROS_INFO_STREAM ("Short round " << round_number_ << " to {1,2,3,4}: q. took " << avg_arrival_delay << " avg s and " << max_arrival_delay << " max s to arrive at dest, and " << complete_delay << " seconds from start to callback.");
      
      acc_avg_arrival_delay_ += avg_arrival_delay;
      acc_max_arrival_delay_ += max_arrival_delay;
      acc_complete_delay_ += (long double) complete_delay.sec + 1e-9 * (long double) complete_delay.nsec;
      num_rounds_++;
    }
    
    
    
  public:
    void
    statistics() {
      if (0 != sid_) {
        return;
      }
      
      acc_avg_arrival_delay_ /= num_rounds_;
      acc_max_arrival_delay_ /= num_rounds_;
      acc_complete_delay_ /= num_rounds_;
      
      cout << "Final results at " << sid_ << " for short multiple: handler avg delay: " << lexical_cast<string> (acc_avg_arrival_delay_) << ", handler avg of max delay: " << lexical_cast<string> (acc_max_arrival_delay_) << ", callback delay: " << lexical_cast<string> (acc_complete_delay_) << "." << endl;
    }
};

#endif

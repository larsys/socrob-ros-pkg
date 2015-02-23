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

#ifndef TESTSHORTSINGLEINLONGDELAY_HPP
#define TESTSHORTSINGLEINLONGDELAY_HPP

#include <iostream>
#include <fstream>
#include <limits>

#include <boost/thread.hpp>
#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <socrob/multicast.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Time.h>



using namespace std;
using namespace ros;
using namespace socrob::multicast;
using boost::lexical_cast;



class TestShortSingleInLongDelay
{
    boost::mutex mutex_;
    
    int sid_;
    
    const uint8_t QUESTION_MAGIC_BYTE_;
    const uint8_t ANSWER_MAGIC_BYTE_;
    
    bool waiting_;
    unsigned round_number_;
    unsigned round_uid_;
    Time round_tx_;
    
    bool answer_;
    unsigned answer_number_;
    unsigned answer_uid_;
    Time answer_time_;
    
    long double acc_arrival_delay_;
    long double acc_complete_delay_;
    unsigned long num_rounds_;
    
    boost::variate_generator<boost::mt19937, boost::uniform_int<unsigned> > random_uid_;
    
    Time output_time_base_;
    unsigned output_line_;
    ofstream output_;
    
  public:
    TestShortSingleInLongDelay (int sid,
                                string const& type) :
      sid_ (sid),
      QUESTION_MAGIC_BYTE_ (0),
      ANSWER_MAGIC_BYTE_ (1),
      waiting_ (false),
      round_number_ (0),
      round_uid_(),
      round_tx_ (Time::now()),
      answer_ (false),
      acc_arrival_delay_ (0),
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
      
      output_.open ( ("/tmp/TestShortSingleInLongDelay_" + type + "_" + lexical_cast<string> (output_time_base_.sec) + ".csv").c_str());
      output_ << "\"Round Number\""
              << ";\"Destination\""
              << ";\"Handler Delay\""
              << ";\"Callback Delay\""
              << endl;
      output_line_ = 1;
    }
    
    
    
    ~TestShortSingleInLongDelay() {
      unsigned l = output_line_;
      output_ << ";;;" << endl;
      output_ << "\"Average Handler Delay:\";=AVERAGE(C2:C" << l << ")" << endl;
      output_ << "\"Average Callback Delay:\";=AVERAGE(D2:D" << l << ")" << endl;
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
        
        // To simulate worst case, not sending this time.
        return;
      }
      
      if (! waiting_) {
        return;
      }
      
      std_msgs::UInt8 magic_msg;
      magic_msg.data = QUESTION_MAGIC_BYTE_;
      serialize_append (buffer, magic_msg);
      
      std_msgs::UInt8 dest_msg;
      dest_msg.data = (round_number_ % 4) + 1;
      serialize_append (buffer, dest_msg);
      
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
        std_msgs::UInt8 dest_msg;
        offset += deserialize (dest_msg, buffer, offset);
        
        if (dest_msg.data != sid_) {
          return;
        }
        
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
        
        Time now = Time::now();
        
        boost::lock_guard<boost::mutex> _ (mutex_);
        
        if (! waiting_) {
          ROS_WARN_STREAM ("Received package while short rounds not active");
          return;
        }
        waiting_ = false;
        
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
        Time ans_arrival_time = arrival_time_msg.data;
        
        if (sid == 0) {
          ROS_FATAL_STREAM ("Received from 0 while testing short rounds single.");
          abort();
        }
        if (sid != (round_number_ % 4) + 1) {
          ROS_WARN ("Received answer from wrong agent while testing short rounds single.");
          return;
        }
        if (sent_number != round_number_) {
          ROS_WARN ("Received answer wrong round number while testing short rounds single.");
          return;
        }
        if (sent_uid != round_uid_) {
          ROS_WARN ("Received answer with right number but wrong UID while testing short rounds single.");
          return;
        }
        
        Duration arrival_delay = ans_arrival_time - round_tx_;
        Duration complete_delay = now - round_tx_;
        
        ROS_INFO_STREAM ("Short round " << round_number_ << " to " << ( (round_number_ % 4) + 1) << ": question took " << arrival_delay << " seconds to arrive at destination, and " << complete_delay << " seconds from start to callback.");
        
        /*unsigned l =*/
        ++ (output_line_);
        output_ << round_number_
                << ";" << ( (round_number_ % 4) + 1)
                << ";" << arrival_delay
                << ";" << complete_delay
                << endl;
                
        acc_arrival_delay_ += (long double) arrival_delay.sec + 1e-9 * (long double) arrival_delay.nsec;
        acc_complete_delay_ += (long double) complete_delay.sec + 1e-9 * (long double) complete_delay.nsec;
        num_rounds_++;
      }
      else {
        ROS_FATAL_STREAM ("Received package with wrong magic byte.");
        abort();
      }
    }
    
    
    
    void
    statistics() {
      if (0 != sid_) {
        return;
      }
      
      acc_arrival_delay_ /= num_rounds_;
      acc_complete_delay_ /= num_rounds_;
      
      cout << "Final results at " << sid_ << " for short single: handler delay: " << lexical_cast<string> (acc_arrival_delay_) << ", callback delay: " << lexical_cast<string> (acc_complete_delay_) << "." << endl;
    }
};

#endif

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

#ifndef TESTSHORTMULTIPLEDELAY_HPP
#define TESTSHORTMULTIPLEDELAY_HPP

#include <iostream>
#include <fstream>
#include <limits>

#include <boost/thread/mutex.hpp>
#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Time.h>

#include <socrob/multicast.h>



using namespace std;
using namespace ros;
using namespace socrob::multicast;
using boost::lexical_cast;



class TestShortMultipleDelay
{
    boost::mutex mutex_;
    
    int sid_;
    
    const uint8_t MAGIC_BYTE_;
    
    unsigned round_number_;
    unsigned round_uid_;
    Time round_tx_;
    
    bool waiting_;
    
    long double acc_max_arrival_delay_;
    long double acc_avg_arrival_delay_;
    long double acc_complete_delay_;
    unsigned long num_rounds_;
    
    unsigned long failed_;
    
    boost::variate_generator<boost::mt19937, boost::uniform_int<unsigned> > random_uid_;
    
    Time output_time_base_;
    unsigned output_line_;
    ofstream output_;
    
  public:
    TestShortMultipleDelay (int sid,
                            string const& type) :
      sid_ (sid),
      MAGIC_BYTE_ (2),
      round_number_ (0),
      round_uid_(),
      round_tx_ (Time::now()),
      waiting_ (false),
      acc_max_arrival_delay_ (0),
      acc_avg_arrival_delay_ (0),
      acc_complete_delay_ (0),
      num_rounds_ (0),
      failed_ (0),
      random_uid_ (boost::mt19937 (Time::now().toNSec()),
                   boost::uniform_int<unsigned> (numeric_limits<unsigned>::min(), numeric_limits<unsigned>::max())),
      output_time_base_ (Time::now()) {
      if (0 != sid_) {
        return;
      }
      
      output_.open ( ("/tmp/TestShortMultipleDelay_" + type + "_" + lexical_cast<string> (output_time_base_.sec) + ".csv").c_str());
      output_ << "\"Round Number\""
              << ";\"Handler Delay 1\""
              << ";\"Handler Delay 2\""
              << ";\"Handler Delay 3\""
              << ";\"Handler Delay 4\""
              << ";\"Callback Delay\""
              << endl;
      output_line_ = 1;
    }
    
    
    
    ~TestShortMultipleDelay() {
      unsigned l = output_line_;
      output_ << ";;;" << endl;
      output_ << "\"Average Handler Delay:\";=AVERAGE(B2:E" << l << ")" << endl;
      output_ << "\"Average Callback Delay:\";=AVERAGE(F2:F" << l << ")" << endl;
    }
    
    
    
    bool
    is_question (vector<uint8_t> & question) {
      std_msgs::UInt8 magic_msg;
      deserialize (magic_msg, question);
      
      if (magic_msg.data == MAGIC_BYTE_) {
        return true;
      }
      
      return false;
    }
    
    
    
    void
    timer (socrob::multicast::Manager& manager) {
      if (0 != sid_) {
        return;
      }
      
      vector<uint8_t> question;
      std_msgs::UInt8 magic_msg;
      magic_msg.data = MAGIC_BYTE_;
      serialize_append (question, magic_msg);
      
      {
        boost::lock_guard<boost::mutex> _ (mutex_);
        
        if (waiting_) {
          Duration age = Time::now() - round_tx_;
          if (age > Duration (1, 0)) {
            ROS_WARN_STREAM_THROTTLE (1, "Last round (" << round_number_ << ") was created " << age << " seconds ago, still waiting...");
          }
          return;
        }
        
        round_number_++;
        round_uid_ = random_uid_();
        round_tx_ = Time::now();
        waiting_ = true;
        
        std_msgs::UInt32 round_number_msg;
        round_number_msg.data = round_number_;
        serialize_append (question, round_number_msg);
        
        std_msgs::UInt32 round_uid_msg;
        round_uid_msg.data = round_uid_;
        serialize_append (question, round_uid_msg);
      }
      
      set<id_type> required_sids;
      required_sids.insert (1);
      required_sids.insert (2);
      required_sids.insert (3);
      required_sids.insert (4);
      
      // This cannot be in the critical region because it may call the callback right away.
      Rate rate (1000);
      while (! manager.startShortRound (question, required_sids, boost::bind (&TestShortMultipleDelay::short_callback, this, _1))) {
        boost::lock_guard<boost::mutex> _ (mutex_);
        failed_++;
        rate.sleep();
      }
    }
    
    
    
    void
    short_handler (vector<uint8_t>& answer,
                   id_type from,
                   vector<uint8_t> & question) {
      Time now = Time::now();
      
      size_t offset = 0;
      std_msgs::UInt8 magic_msg;
      offset += deserialize (magic_msg, question, offset);
      std_msgs::UInt32 number_msg;
      offset += deserialize (number_msg, question, offset);
      std_msgs::UInt32 uid_msg;
      offset += deserialize (uid_msg, question, offset);
      
      if (magic_msg.data != MAGIC_BYTE_) {
        ROS_FATAL_STREAM ("TestShortMultipleDelay::short_handler received a question with wrong magic byte.");
        abort();
      }
      
      serialize_append (answer, number_msg);
      
      serialize_append (answer, uid_msg);
      
      std_msgs::UInt8 sid_msg;
      sid_msg.data = sid_;
      serialize_append (answer, sid_msg);
      
      std_msgs::Time time_msg;
      time_msg.data = now;
      serialize_append (answer, time_msg);
    }
    
    
    
    void
    short_callback (map<id_type, vector<uint8_t> > & answers) {
      Time now = Time::now();
      
      boost::lock_guard<boost::mutex> _ (mutex_);
      
      if (! waiting_) {
        ROS_FATAL_STREAM ("Callback called unexpectedly while short rounds not active");
        abort();
      }
      waiting_ = false;
      
      if (0 == answers.size()) {
        ROS_WARN_STREAM ("Did not receive any answer in this round while testing short rounds multiple.");
        return;
      }
      
      if (4 > answers.size()) {
        ROS_WARN_STREAM ("Did not receive enough answers in this round while testing short rounds multiple. Maybe the delay to one or more robots was too big and therefore considered NOT_RUNNING.");
        return;
      }
      
      if (4 != answers.size()) {
        ROS_FATAL_STREAM ("Received too many answers in this round while testing short rounds multiple.");
        abort();
      }
      
      long double avg_arrival_delay = 0;
      long double max_arrival_delay = 0;
      
      /*unsigned l =*/
      ++ (output_line_);
      output_ << round_number_;
      
      typedef map<id_type, vector<uint8_t> >::value_type tmp_t;
      foreach (tmp_t & answer_pair, answers) {
        vector<uint8_t> & answer = answer_pair.second;
        
        size_t offset = 0;
        std_msgs::UInt32 number_msg;
        offset += deserialize (number_msg, answer, offset);
        std_msgs::UInt32 uid_msg;
        offset += deserialize (uid_msg, answer, offset);
        std_msgs::UInt8 sid_msg;
        offset += deserialize (sid_msg, answer, offset);
        std_msgs::Time arrival_time_msg;
        offset += deserialize (arrival_time_msg, answer, offset);
        
        unsigned sent_number = number_msg.data;
        unsigned sent_uid = uid_msg.data;
        id_type sid = sid_msg.data;
        Time ans_arrival_time = arrival_time_msg.data;
        
        if (sid != answer_pair.first) {
          ROS_FATAL_STREAM ("Received answer with agent number mismatch while testing short rounds single.");
          abort();
        }
        if (sid != 1 && sid != 2 && sid != 3 && sid != 4) {
          ROS_FATAL_STREAM ("Received answer from wrong agent while testing short rounds multiple.");
          abort();
        }
        if (sent_number != round_number_) {
          ROS_FATAL_STREAM ("Received answer wrong round number while testing short rounds multiple.");
          abort();
        }
        if (sent_uid != round_uid_) {
          ROS_FATAL_STREAM ("Received answer with right number but wrong UID while testing short rounds multiple.");
          abort();
        }
        
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
    
    
    
    void
    statistics() {
      if (0 != sid_) {
        return;
      }
      
      acc_avg_arrival_delay_ /= num_rounds_;
      acc_max_arrival_delay_ /= num_rounds_;
      acc_complete_delay_ /= num_rounds_;
      
      cout << "Final results at " << sid_ << " for short multiple: handler avg delay: " << lexical_cast<string> (acc_avg_arrival_delay_) << ", handler avg of max delay: " << lexical_cast<string> (acc_max_arrival_delay_) << ", callback delay: " << lexical_cast<string> (acc_complete_delay_) << "." << endl;
      cout << "Failed rounds at " << sid_ << ": " << failed_ << endl;
    }
};

#endif

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

#ifndef TESTLONGDELAY_HPP
#define TESTLONGDELAY_HPP

#include <iostream>
#include <fstream>

#include <boost/thread/mutex.hpp>
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



struct rcv_data {
  Time time;
  size_t serial;
  Time production_time;
  Time send_time;
};



class TestLongDelay
{
    int sid_;
    
    Time production_time_;
    size_t production_serial_;
    
    boost::mutex mutex_;
    vector<list<rcv_data> > rcv_;
    
    Time output_time_base_;
    unsigned output_line_[5];
    ofstream output_[5];
    
    
  public:
    TestLongDelay (int sid,
                   string const& type) :
      sid_ (sid),
      production_time_ (Time::now()),
      production_serial_ (0),
      rcv_ (5, list<rcv_data>()),
      output_time_base_ (Time::now()) {
      for (int i = 0; i < 5; i++) {
#ifdef SOCROB_MULTICAST_LONG_SYNC_WORST
        output_[i].open ( ("/tmp/TestLongDelay_" + type + "_WorstCase_" + lexical_cast<string> (output_time_base_.sec) + "_at_" + lexical_cast<string> (sid_) + "_from_" + lexical_cast<string> (i) + ".csv").c_str());
#else
        output_[i].open ( ("/tmp/TestLongDelay_" + type + "_Normal_" + lexical_cast<string> (output_time_base_.sec) + "_at_" + lexical_cast<string> (sid_) + "_from_" + lexical_cast<string> (i) + ".csv").c_str());
#endif
        output_[i] << "\"Serial Number\""
                   << ";\"Production Time\""
                   << ";\"Send Time\""
                   << ";\"Reception Time\""
                   << ";\"Use Duration\""
                   << ";\"ACC Use Duration\""
                   << ";\"(Without) Age Rcv\""
                   << ";\"(Without) Age End\""
                   << ";\"(Without) ACC Average\""
                   << ";\"(With) Age Rcv\""
                   << ";\"(With) Age End\""
                   << ";\"(With) ACC Average\""
                   << endl;
        output_line_[i] = 1;
      }
    }
    
    
    
    ~TestLongDelay() {
      for (int i = 0; i < 5; i++) {
        output_[i] << ";;;;;;;;;;;" << endl;
        unsigned l = output_line_[i];
        output_[i] << "\"Received packages:\";=" << l << "-4" << endl;
        output_[i] << "\"Lost packages in between:\";=1+A" << l << "-A5-(" << l << "-4)" << endl;
        output_[i] << "\"Average without transmission:\";=I" << l << "/F" << l << "" << endl;
        output_[i] << "\"Average with transmission:\";=L" << l << "/F" << l << "" << endl;
      }
    }
    
    
    
    void
    long_marshall (vector<uint8_t> & buffer) {
      Time send_time = Time::now();
      
      std_msgs::UInt8 sid_msg;
      sid_msg.data = sid_;
      serialize_append (buffer, sid_msg);
      
      std_msgs::UInt32 prod_serial_msg;
      prod_serial_msg.data = production_serial_;
      serialize_append (buffer, prod_serial_msg);
      
      std_msgs::Time prod_time_msg;
      prod_time_msg.data = production_time_;
      serialize_append (buffer, prod_time_msg);
      
      std_msgs::Time send_time_msg;
      send_time_msg.data = send_time;
      serialize_append (buffer, send_time_msg);
      
      production_time_ = send_time;
      production_serial_++;
    }
    
    
    
    void
    long_unmarshall (socrob::multicast::id_type,
                     vector<uint8_t> & buffer) {
      Time now = Time::now();
      
      size_t offset = 0;
      std_msgs::UInt8 sid_msg;
      offset += deserialize (sid_msg, buffer, offset);
      std_msgs::UInt32 prod_serial_msg;
      offset += deserialize (prod_serial_msg, buffer, offset);
      std_msgs::Time prod_time_msg;
      offset += deserialize (prod_time_msg, buffer, offset);
      std_msgs::Time send_time_msg;
      offset += deserialize (send_time_msg, buffer, offset);
      
      int rcv_sid = sid_msg.data;
      
      rcv_data rcv;
      rcv.time = now;
      rcv.serial = prod_serial_msg.data;
      rcv.production_time = prod_time_msg.data;
      rcv.send_time = send_time_msg.data;
      
      boost::lock_guard<boost::mutex> _ (mutex_);
      
      rcv_[rcv_sid].push_back (rcv);
      
      unsigned l = ++ (output_line_[rcv_sid]);
      if (l >= 3 + 3/*line 3 + 3 ignored in statistics*/) {
        output_[rcv_sid] << rcv.serial
                         << ";" << (rcv.production_time - output_time_base_)
                         << ";" << (rcv.send_time - output_time_base_)
                         << ";" << (rcv.time - output_time_base_)
                         << ";" << "=D" << l << "-D" << (l - 1)
                         << ";" << "=E" << l << "+F" << (l - 1)
                         << ";" << "=C" << (l - 1) << "-B" << (l - 1)
                         << ";" << "=G" << l << "+E" << l
                         << ";" << "=I" << (l - 1) << "+(((G" << l << "+H" << l << ")/2)*E" << l << ")"
                         << ";" << "=D" << (l - 1) << "-B" << (l - 1)
                         << ";" << "=J" << l << "+E" << l
                         << ";" << "=L" << (l - 1) << "+(((J" << l << "+K" << l << ")/2)*E" << l << ")"
                         << endl;
      }
      else {
        output_[rcv_sid] << rcv.serial
                         << ";" << (rcv.production_time - output_time_base_)
                         << ";" << (rcv.send_time - output_time_base_)
                         << ";" << (rcv.time - output_time_base_)
                         << ";" << "\"vvv\""
                         << ";" << "0"
                         << ";" << "\"vvv\""
                         << ";" << "\"vvv\""
                         << ";" << "0"
                         << ";" << "\"vvv\""
                         << ";" << "\"vvv\""
                         << ";" << "0"
                         << endl;
      }
    }
    
    
    
    void
    statistics() {
      long double final_with = 0;
      long double final_without = 0;
      int active = 0;
      
      for (int sid = 0; sid < 5; sid++) {
        list<rcv_data>& rcv_list = rcv_[sid];
        
        rcv_list.pop_front();
        rcv_list.pop_front();
        rcv_list.pop_front();
        
        if (rcv_list.size() <= 1) {
          cout << sid_ << " Nothing or not enough received from sid " << sid << endl;
          continue;
        }
        
        // cout << sid_ << " Processing data from sid " << sid << endl;
        
        size_t lost_packages = (1 + rcv_list.back().serial - rcv_list.front().serial) - rcv_list.size();
        size_t received_packages = rcv_list.size();
        // cout << sid_ << "   Received " << received_packages << " packages, lost " << lost_packages << " packages." << endl;
        
        long double acc_use_time = 0;
        long double acc_without_transmission = 0;
        long double acc_with_transmission = 0;
        
        list<rcv_data>::iterator last_i = rcv_list.begin();
        for (list<rcv_data>::iterator i = ++ rcv_list.begin();
             i != rcv_list.end();
             i++) {
          rcv_data& last_data = *last_i;
          rcv_data& curr_data = *i;
          
          if (curr_data.serial <= last_data.serial) {
            cout << sid_ << "   Detected duplicate or reorder: Package with serial " << last_data.serial << " received before package with serial " << curr_data.serial << "." << endl;
          }
          
          long double use_time = (curr_data.time - last_data.time).toSec();
          acc_use_time += use_time;
          
          // Without transmission times
          {
            long double age_at_reception = (last_data.send_time - last_data.production_time).toSec();
            long double age_at_end = age_at_reception + use_time;
            cout << age_at_reception << " " << age_at_end << " " << use_time << endl;
            acc_without_transmission += ( (age_at_reception + age_at_end) / 2) * use_time;
          }
          
          // With transmission times
          {
            long double age_at_reception = (last_data.time - last_data.production_time).toSec();
            long double age_at_end = age_at_reception + use_time;
            acc_with_transmission += ( (age_at_reception + age_at_end) / 2) * use_time;
          }
          
          last_i = i;
        }
        
        long double without_transmission = acc_without_transmission / acc_use_time;
        long double with_transmission = acc_with_transmission / acc_use_time;
        long double transmission_time = with_transmission - without_transmission;
        
        if (sid != sid_) {
          final_with += with_transmission;
          final_without += without_transmission;
          active++;
        }
        
        // cout << sid_ << "   Without considering transmission time, the average outdatedness of data was " << without_transmission << endl;
        // cout << sid_ << "   Considering synchronized clocks and transmission time, the average outdatedness of data was " << with_transmission << endl;
        // cout << sid_ << "   Mean transmission time: "<<transmission_time<<endl;
        
        cout << sid_
             << " From: " << sid
             << ", received: " << received_packages
             << ", lost: " << lost_packages
             << ", without: " << lexical_cast<string> (without_transmission)
             << ", with: " << lexical_cast<string> (with_transmission)
             << ", transmission_time: " << transmission_time << "." << endl;
      }
      
      final_with /= active;
      final_without /= active;
      
      cout << "Final results at " << sid_ << ": without: " << lexical_cast<string> (final_without) << ", with: " << lexical_cast<string> (final_with) << "." << endl;
    }
};

#endif

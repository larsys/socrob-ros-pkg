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

#ifndef SHORTHEAVYLOADER_HPP
#define SHORTHEAVYLOADER_HPP

#include <iostream>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <socrob/multicast.h>



using namespace std;
using namespace ros;
using namespace socrob::multicast;



class ShortHeavyLoader
{
    int sid_;
    
    const uint8_t MAGIC_BYTE_;
    
    unsigned long rounds_;
    
  public:
    ShortHeavyLoader (int sid) :
      sid_ (sid),
      MAGIC_BYTE_ (0),
      rounds_ (0) {}
      
      
      
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
    short_handler (vector<uint8_t>& answer,
                   id_type from,
                   vector<uint8_t> const& question) {
    }
    
    
    
    void
    short_callback (std::map<id_type, std::vector<uint8_t> > const& answers) {
    }
    
    
    
    void
    timer (socrob::multicast::Manager& manager) {
      vector<uint8_t> question = vector<uint8_t> (1);
      question[0] = 0;
      
      for (int i = 0; i < 1000; i++) {
        set<id_type> required_sids;
        required_sids.insert (0);
        required_sids.insert (1);
        required_sids.insert (2);
        required_sids.insert (3);
        required_sids.insert (4);
        
        if (manager.startShortRound (question, required_sids, boost::bind (&ShortHeavyLoader::short_callback, this, _1))) {
          rounds_++;
        }
      }
    }
    
    
    
    void
    statistics() {
      cout << "ShortHeavyLoader at " << sid_ << " successfully started " << rounds_ << " short rounds." << endl;
    }
};

#endif

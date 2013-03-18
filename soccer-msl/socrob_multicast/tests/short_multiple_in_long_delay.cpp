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

#include <iostream>

#include <ros/console.h>

#include <socrob/multicast.h>

#include "TestShortMultipleInLongDelay.h"



using namespace std;
using namespace ros;
using namespace socrob::multicast;



int main (int argc, char** argv)
{
  init (argc, argv, "short_multiple_in_long_delay");
  
  ros::NodeHandle nh;
  
  int sid;
  if (! ros::param::getCached ("robot_number", sid)) {
    ROS_FATAL ("Could not get robot_number param");
    abort();
  }
  sid -= 1;
  
  TestShortMultipleInLongDelay test_short (sid, "Normal");
  
  {
    Manager srmm (ManagerOptions (sid, 5, "239.255.1.1")
                  .bandwidth_bps (57671) // 2.2*1024*1024/8/5
                  .ports (2000, 2001)
                  .tup_ms (100)
                  .long_marshall (boost::bind (&TestShortMultipleInLongDelay::long_marshall, boost::ref (test_short), _1))
                  .long_unmarshall (boost::bind (&TestShortMultipleInLongDelay::long_unmarshall, boost::ref (test_short), _1, _2)));
                  
    ros::Rate loop_rate (10);
    while (ros::ok()) {
      test_short.timer ();
      loop_rate.sleep();
    }
  }
  
  test_short.statistics();
}

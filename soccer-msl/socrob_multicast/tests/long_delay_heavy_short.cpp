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

#include "TestLongDelay.h"
#include "ShortHeavyLoader.h"



using namespace std;
using namespace ros;
using namespace socrob::multicast;



int main (int argc, char** argv)
{
  init (argc, argv, "long_delay_heavy_short");
  
  ros::NodeHandle nh;
  
  int sid;
  if (! ros::param::getCached ("robot_number", sid)) {
    ROS_FATAL ("Could not get robot_number param");
    abort();
  }
  sid -= 1;
  
  TestLongDelay test_long (sid, "HeavyShort");
  ShortHeavyLoader short_heavy_loader (sid);
  
  {
    Manager srmm (ManagerOptions (sid, 5, "239.255.1.1")
                  .bandwidth_bps (57671) // 2.2*1024*1024/8/5
                  .ports (2000, 2001)
                  .tup_ms (100)
                  .long_marshall (boost::bind (&TestLongDelay::long_marshall, boost::ref (test_long), _1))
                  .long_unmarshall (boost::bind (&TestLongDelay::long_unmarshall, boost::ref (test_long), _1, _2))
                  .short_handler (boost::bind (&ShortHeavyLoader::short_handler, boost::ref (short_heavy_loader), _1, _2, _3)));
                  
    Rate loop_rate (10);
    while (ok()) {
      short_heavy_loader.timer (srmm);
      loop_rate.sleep();
    }
  }
  
  test_long.statistics();
  short_heavy_loader.statistics();
}

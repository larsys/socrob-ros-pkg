/**
 *  @file keyframe_mapper_node.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  @section LICENSE
 *
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ccny_rgbd/apps/navigator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Navigator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::Navigator km(nh, nh_private);
  ros::spin();
  return 0;
}


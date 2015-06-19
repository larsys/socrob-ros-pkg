/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010,
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Henrique Silva 
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of CCNY Robotics Lab nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "laser_height/laser_height.h"

namespace mav{

LaserHeightEstimation::LaserHeightEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  first_time_(true)
{
  ROS_INFO("Starting LaserHeight");

  initialized_  = false;
  prev_height_  = 0.0;

  world_to_base_.setIdentity();

  ros::NodeHandle nh_mav (nh_, "quad_height");

  // Parameters
  nh_private_.param<std::string>("global_frame", world_frame_, "map");
  nh_private_.param<std::string>("stabilized_frame", stabilized_frame_, "base_stabilized");
  nh_private_.param<std::string>("base_frame", base_frame_, "base_link");

  nh_private_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
  nh_private_.param<std::string>("scan_topic", scan_topic_, "scan");

  nh_private_.param("min_values", min_values_, 5);
  nh_private_.param("max_stdev", max_stdev_, 0.1);
  nh_private_.param("max_height_jump", max_height_jump_, 0.25);
  nh_private_.param("bin_size", bin_size_, 0.2);

  nh_private_.param<bool>("use_imu", use_imu_, true);
  nh_private_.param<bool>("use_segmentation", use_segmentation_, true);

  // Publishers
  height_to_base_publisher_ = nh_mav.advertise<mav_msgs::Height>("height_to_base", 5);
  height_to_footprint_publisher_ = nh_mav.advertise<mav_msgs::Height>("height_to_footprint", 5);

  // Subscribers
  scan_subscriber_ = nh_.subscribe(scan_topic_, 5, &LaserHeightEstimation::scanCallback, this);
  if (use_imu_){
    imu_subscriber_ = nh_.subscribe(imu_topic_, 5, &LaserHeightEstimation::imuCallback, this);
  }
}

LaserHeightEstimation::~LaserHeightEstimation(){
  ROS_INFO("Destroying LaserHeightEstimation");
}

void LaserHeightEstimation::imuCallback (const sensor_msgs::ImuPtr& imu_msg){

  mutex_.lock();

  tf::Quaternion q;
  tf::quaternionMsgToTF(imu_msg->orientation, q);   
  world_to_base_.setRotation(q);

  mutex_.unlock();
}

void LaserHeightEstimation::scanCallback (const sensor_msgs::LaserScanPtr& scan_msg){
  if (!initialized_){
    // if this is the first scan, lookup the static base to laser tf
    // if tf is not available yet, skip this scan
    if (!setBaseToLaserTf(scan_msg)) return;

    initialized_ = true;
    last_update_time_ = scan_msg->header.stamp;

    return;
  }

  // Get required transforms
  if(!use_imu_){
    tf::StampedTransform world_to_base_tf;

    try{
      tf_listener_.waitForTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, ros::Duration(0.5));
      tf_listener_.lookupTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, world_to_base_tf);
    }
    catch (tf::TransformException ex){
      // transform unavailable - skip scan
      ROS_WARN ("TF unavailable, skipping scan (%s)", ex.what());
      return;
    }
    world_to_base_.setRotation( world_to_base_tf.getRotation());
  }

  mutex_.lock();

  tf::Transform rotated_laser     = world_to_base_ * base_to_laser_;
  tf::Transform rotated_footprint = world_to_base_ * base_to_footprint_;

  mutex_.unlock();

  // Get vector of height values
  std::vector<double> values;
  for(unsigned int i = 0; i < scan_msg->ranges.size(); i++){
    if (scan_msg->ranges[i] > scan_msg->range_min && scan_msg->ranges[i] < scan_msg->range_max){
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      tf::Vector3 v(cos(angle)*scan_msg->ranges[i], sin(angle)*scan_msg->ranges[i], 0.0);
      tf::Vector3 p = rotated_laser * v;
      values.push_back(p.getZ());
     }
  }

  // Segment if required
  if (!values.empty() && use_segmentation_){
    std::vector<double> values_seg;
    histogramSegmentation(values, values_seg);
    values = values_seg;
  }

  // Check if we have enough values present
  if ((int)values.size() < min_values_){
    ROS_WARN("Not enough valid values to determine height, skipping (%d collected, %d needed)",values.size(), min_values_);
    return;
  }

  // Get mean and standard dev
  double mean_value, stdev_value;
  getStats(values, mean_value, stdev_value);

  if (stdev_value > max_stdev_){
    ROS_WARN("Stdev of height readings too big to determine height, skipping (stdev is %f, max is %f)",stdev_value, max_stdev_);
    return;
  }

  // Estimate height (to base and to footprint)
  double height_to_base = 0.0 - 0.13 - 0.375 - mean_value;
  double height_to_footprint = rotated_footprint.getOrigin().getZ() - mean_value;
  //ROS_INFO("2Base:%f, 2Footprint:%f",height_to_base,height_to_footprint);
  ROS_INFO("Height:%f",height_to_base);

  // Check for discontinuity
  // Prevent first-time discontinuities
  if (first_time_){
    first_time_ = false;
    prev_height_ = height_to_base;
  }

  double height_jump = prev_height_ - height_to_base;

  if (fabs(height_jump) > max_height_jump_){
    ROS_WARN("Laser height estimation: floor discontinuity detected");
    height_to_base += height_jump;
    height_to_footprint += height_jump;
  }

  double dt = (scan_msg->header.stamp - last_update_time_).toSec();

  double climb = (height_to_base - prev_height_)/dt;
  prev_height_ = height_to_base;
  last_update_time_ = scan_msg->header.stamp;

  // Publish height message
  mav_msgs::HeightPtr height_to_base_msg;
  height_to_base_msg = boost::make_shared<mav_msgs::Height>();
  height_to_base_msg->height = height_to_base;
  height_to_base_msg->height_variance = stdev_value;
  height_to_base_msg->climb = climb;
  height_to_base_msg->header.stamp = scan_msg->header.stamp;
  height_to_base_msg->header.frame_id = scan_msg->header.frame_id;  //TODO:wrong frame
  height_to_base_publisher_.publish(height_to_base_msg);

  mav_msgs::HeightPtr height_to_footprint_msg;
  height_to_footprint_msg = boost::make_shared<mav_msgs::Height>();
  height_to_footprint_msg->height = height_to_footprint;
  height_to_footprint_msg->height_variance = stdev_value;
  height_to_footprint_msg->climb = climb;
  height_to_footprint_msg->header.stamp = scan_msg->header.stamp;
  height_to_footprint_msg->header.frame_id = scan_msg->header.frame_id; //TODO:wrong frame
  height_to_footprint_publisher_.publish(height_to_footprint_msg);

}

bool LaserHeightEstimation::setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg){
  ros::Time time = scan_msg->header.stamp;

  // Get transform base to laser
  tf::StampedTransform base_to_laser_tf;
  try{
    tf_listener_.waitForTransform(
      base_frame_, scan_msg->header.frame_id, time, ros::Duration(3.0));

    tf_listener_.lookupTransform (
      base_frame_, scan_msg->header.frame_id, time, base_to_laser_tf);
  }
  catch (tf::TransformException ex){
    // transform unavailable - skip scan
    ROS_WARN("%s: Transform unavailable, skipping scan (%s)", ros::this_node::getName().c_str(), ex.what());
    return false;
  }

  base_to_laser_ = base_to_laser_tf;

  // Get transform base to base_footprint
  tf::StampedTransform base_to_footprint_tf;
  try{
    tf_listener_.waitForTransform(
      base_frame_, stabilized_frame_, time, ros::Duration(3.0));

    tf_listener_.lookupTransform (
      base_frame_, stabilized_frame_, time, base_to_footprint_tf);
  }
  catch (tf::TransformException ex){
    // transform unavailable - skip scan
    ROS_WARN("%s: Transform unavailable, skipping scan (%s)", ros::this_node::getName().c_str(), ex.what());
    return false;
  }

  base_to_footprint_ = base_to_footprint_tf;

  return true;
}

void LaserHeightEstimation::getStats(const std::vector<double> values, double& ave, double& stdev){
  double sum   = 0.0;
  double sumsq = 0.0;

  for (size_t i = 0; i < values.size(); ++i)
    sum += values[i];

  ave = sum/values.size();

  for (size_t i = 0; i < values.size(); ++i)
    sumsq += (values[i] - ave) * (values[i] - ave);

  stdev = sqrt(sumsq/values.size());
}

void LaserHeightEstimation::histogramSegmentation(const std::vector<double>& input,std::vector<double>& output){
  // Get min and max bounds
  double min_value = input[0];
  double max_value = input[0];
  for (unsigned int i = 0; i < input.size(); ++i){
    if (input[i] < min_value) min_value = input[i];
    if (input[i] > max_value) max_value = input[i];
  }  

  // Create histogram
  int n_bins = (max_value - min_value) / bin_size_ + 1;
  std::vector<int> bins;
  bins.resize(n_bins);

  for (unsigned int i = 0; i < bins.size(); ++i)
    bins[i] = 0; 

  for (unsigned int i = 0; i < input.size(); ++i)
  { 
    unsigned int bin_idx = (input[i] - min_value) / bin_size_;
    bins[bin_idx]++;
  } 

  // Get max of histogram
  unsigned int best_bin_idx = 0;
  int best_bin_count = 0;
  for (unsigned int i = 0; i < bins.size(); ++i){
    if (bins[i] > best_bin_count){
      best_bin_count = bins[i];
      best_bin_idx = i;
    }
  }

  // Find inliers
  output.clear();
  for (unsigned int i = 0; i < input.size(); ++i){
    unsigned int bin_idx = (input[i] - min_value) / bin_size_;
    if (bin_idx == best_bin_idx)
      output.push_back(input[i]);
  }
}

};// namespace mav

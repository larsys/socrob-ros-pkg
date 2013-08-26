/**
 *  @file keyframe_mapper.cpp
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
#include "aruco/aruco.h"
namespace ccny_rgbd {

Navigator::Navigator(
        const ros::NodeHandle& nh,
        const ros::NodeHandle& nh_private):
    nh_(nh),
    nh_private_(nh_private),
    rgbd_frame_index_(0)
{
    ROS_INFO("Starting Navigator node");

    // **** params

    initParams();

    // **** publishers

    keyframes_pub_ = nh_.advertise<PointCloudT>(
                "keyframes", queue_size_);
    poses_pub_ = nh_.advertise<visualization_msgs::Marker>(
                "keyframe_poses", queue_size_);
    kf_assoc_pub_ = nh_.advertise<visualization_msgs::Marker>(
                "keyframe_associations", queue_size_);
    path_pub_ = nh_.advertise<PathMsg>(
                "mapper_path", queue_size_);
    aruco_pub_ = nh_.advertise<PathMsg>(
                "aRuco_path", queue_size_);

    // **** services

    pub_keyframe_service_ = nh_.advertiseService(
                "publish_keyframe", &Navigator::publishKeyframeSrvCallback, this);
    pub_keyframes_service_ = nh_.advertiseService(
                "publish_keyframes", &Navigator::publishKeyframesSrvCallback, this);
    save_kf_service_ = nh_.advertiseService(
                "save_keyframes", &Navigator::saveKeyframesSrvCallback, this);
    load_kf_service_ = nh_.advertiseService(
                "load_keyframes", &Navigator::loadKeyframesSrvCallback, this);
    save_pcd_map_service_ = nh_.advertiseService(
                "save_pcd_map", &Navigator::savePcdMapSrvCallback, this);
    save_octomap_service_ = nh_.advertiseService(
                "save_octomap", &Navigator::saveOctomapSrvCallback, this);
    add_manual_keyframe_service_ = nh_.advertiseService(
                "add_manual_keyframe", &Navigator::addManualKeyframeSrvCallback, this);
    generate_graph_service_ = nh_.advertiseService(
                "generate_graph", &Navigator::generateGraphSrvCallback, this);
    solve_graph_service_ = nh_.advertiseService(
                "solve_graph", &Navigator::solveGraphSrvCallback, this);

    // **** subscribers

    ImageTransport rgb_it(nh_);
    ImageTransport depth_it(nh_);

    sub_rgb_.subscribe(rgb_it,     "/camera/rgb/image_rect_color",   queue_size_);
    sub_depth_.subscribe(depth_it, "/camera/depth_registered/image_rect_raw", queue_size_);
    sub_info_.subscribe(nh_,       "/camera/rgb/camera_info",  queue_size_);

    // Synchronize inputs.
    sync_.reset(new RGBDSynchronizer3(
                    RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));

    sync_->registerCallback(boost::bind(&Navigator::RGBDCallback, this, _1, _2, _3));
}

Navigator::~Navigator()
{

}
aruco::MarkerDetector MDetector;
rgbdtools::KeyframeGraphSolverISAM solver_;
void Navigator::initParams()
{
    bool verbose;

    if (!nh_private_.getParam ("verbose", verbose))
        verbose = true;
    if (!nh_private_.getParam ("queue_size", queue_size_))
        queue_size_ = 5;
    if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
        fixed_frame_ = "/odom";
    if (!nh_private_.getParam ("pcd_map_res", pcd_map_res_))
        pcd_map_res_ = 0.01;
    if (!nh_private_.getParam ("octomap_res", octomap_res_))
        octomap_res_ = 0.05;
    if (!nh_private_.getParam ("octomap_with_color", octomap_with_color_))
        octomap_with_color_ = true;
    if (!nh_private_.getParam ("kf_dist_eps", kf_dist_eps_))
        kf_dist_eps_  = 0.10;
    if (!nh_private_.getParam ("kf_angle_eps", kf_angle_eps_))
        kf_angle_eps_  = 10.0 * M_PI / 180.0;
    if (!nh_private_.getParam ("max_range", max_range_))
        max_range_  = 05.0;
    if (!nh_private_.getParam ("max_stdev", max_stdev_))
        max_stdev_  = 555;
    if (!nh_private_.getParam ("max_map_z", max_map_z_))
        max_map_z_ = std::numeric_limits<double>::infinity();

    // configure graph detection
    MDetector.setThresholdParams( 10.0,7.0);
    MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);

    int graph_n_keypoints;
    int graph_n_candidates;
    int graph_k_nearest_neighbors;
    bool graph_matcher_use_desc_ratio_test = true;

    if (!nh_private_.getParam ("graph/n_keypoints", graph_n_keypoints))
        graph_n_keypoints = 500;
    if (!nh_private_.getParam ("graph/n_candidates", graph_n_candidates))
        graph_n_candidates = 10;
    if (!nh_private_.getParam ("graph/k_nearest_neighbors", graph_k_nearest_neighbors))
        graph_k_nearest_neighbors = 4;
    int min_inliers = 25;

    graph_detector_.setMapping(false);
    graph_detector_.setNKeypoints(graph_n_keypoints);
    graph_detector_.setNCandidates(graph_n_candidates);
    graph_detector_.setKNearestNeighbors(graph_k_nearest_neighbors);
    graph_detector_.setMatcherUseDescRatioTest(graph_matcher_use_desc_ratio_test);
    graph_detector_.setSacMinInliers(min_inliers);
    graph_detector_.setSACReestimateTf(false);
    graph_detector_.setSACSaveResults(true);
    graph_detector_.setVerbose(verbose);
    graph_detector_.setMaxIterations(300);
}

void Navigator::filter_aruco()
{
    double max_dist_ = 0.2*0.2;
    std::vector<std::vector<double> > positions;
    std::vector<std::vector<int> > inds;
    for(u_int i = 0; i < aruco_pos.size(); i++)
    {
        std::vector<double> val;
        val.push_back(0);
        val.push_back(0);
        val.push_back(0);
        std::vector<int> sub_inds;
        for(u_int j = 0; j < aruco_pos.size() ; j++)
        {
            double man_dist = std::pow(aruco_pos[i][0]-aruco_pos[j][0],2)+ std::pow(aruco_pos[i][1]-aruco_pos[j][1],2)+ std::pow(aruco_pos[i][2]-aruco_pos[j][2],2);
            if (man_dist < max_dist_)
            {
                sub_inds.push_back(j);
                val[0] += aruco_pos[j][0];
                val[1] += aruco_pos[j][1];
                val[2] += aruco_pos[j][2];
            }

        }
        val[0] /= sub_inds.size();
        val[1] /= sub_inds.size();
        val[2] /= sub_inds.size();
        positions.push_back(val);
        inds.push_back(sub_inds);
    }
    PathMsg ar;
    ar.header.frame_id = fixed_frame_;
    ar.header.seq = 0;
    ar.header.stamp = ros::Time::now();
    double min_readings = 2;
    std::vector<std::vector<double> > aruc;
    //    aruco_pos.clear();
    for(u_int i = 0; i < positions.size(); i++)
    {
        std::vector<double> val;
        val.push_back(0);
        val.push_back(0);
        val.push_back(0);
        std::vector<int> sub_inds;
        for(u_int j = 0; j < positions.size() ; j++)
        {
            if(inds[j].size() <= min_readings) //we need min 2 readings to consider a position safe.
            {
                continue;
            }
            double dist = std::pow(positions[i][0]-positions[j][0],2)+ std::pow(positions[i][1]-positions[j][1],2)+ std::pow(positions[i][2]-positions[j][2],2);
            if (dist < max_dist_)
            {
                sub_inds.push_back(j);
                val[0] += positions[j][0];
                val[1] += positions[j][1];
                val[2] += positions[j][2];
            }

        }
        if(sub_inds.size() <= min_readings)
        {
            val[0] = -99999;
            aruc.push_back(val);
            continue;
        }
        if(!(val[0]==-99999))
        {
            val[0] /= sub_inds.size();
            val[1] /= sub_inds.size();
            val[2] /= sub_inds.size();
        }
        aruc.push_back(val);
    }
    aruco_pos.clear();
    for(u_int i = 0; i< aruc.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        if(aruc[i][0] == -99999)
            continue;
        rgbdtools::RGBDKeyframe keyframe=keyframes_[aruco_kfs[i]];
        pose.header.frame_id = fixed_frame_;
        pose.header.seq = keyframe.header.seq;
        pose.header.stamp.sec = keyframe.header.stamp.sec;
        pose.header.stamp.nsec = keyframe.header.stamp.nsec;
        pose.pose.position.x = aruc[i][0];
        pose.pose.position.y = aruc[i][1];
        pose.pose.position.z = aruc[i][2];
        pose.pose.orientation.x = 1;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        ar.poses.push_back(pose);
        std::vector<double> pos;
        pos.push_back(aruc[i][0]);
        pos.push_back(aruc[i][1]);
        pos.push_back(aruc[i][2]);
        aruco_pos.push_back(pos);
    }
    aruco_pub_.publish(ar);
    //    positions.clear();
    //    for(u_int i = 0; i< aruco_pos.size(); i++)
    //    {
    //        double x,y,z;
    //        x = aruco_pos[i][0];
    //        y = aruco_pos[i][1];
    //        z = aruco_pos[i][2];
    //        bool hit = false;
    //        for(u_int j = 0; j < positions.size(); j++)
    //        {
    //            if (x == positions[j][0] && y == positions[j][1] && z == positions[j][2])
    //            {
    //                hit = true;
    //                break;
    //            }
    //        }
    //        if(!hit)
    //        {
    //            positions.push_back(aruco_pos[i]);
    //        }
    //    }
    //    aruco_pos.clear();
    //    for(u_int i = 0; i < positions.size(); i++)
    //        aruco_pos.push_back(positions[i]);
    //    std::cout << "oh hai!" << std::endl;
}

bool first = true;
aruco::CameraParameters param;
cv::Mat intr, dist;
cv::Size CamSize;

void Navigator::RGBDCallback(
        const ImageMsg::ConstPtr& rgb_msg,
        const ImageMsg::ConstPtr& depth_msg,
        const CameraInfoMsg::ConstPtr& info_msg)
{
    tf::StampedTransform transform;

    const ros::Time& time = rgb_msg->header.stamp;
    if (first)
    {
        convertCameraInfoToMats(info_msg, intr, dist);
        CamSize.width = info_msg->width;
        CamSize.height = info_msg->height;
        param.CameraMatrix = intr;
        param.CamSize = CamSize;
        param.Distorsion = dist;

        first = false;
    }
    try{
        tf_listener_.waitForTransform(
                    fixed_frame_, rgb_msg->header.frame_id, time, ros::Duration(0.1));
        tf_listener_.lookupTransform(
                    fixed_frame_, rgb_msg->header.frame_id, time, transform);
    }
    catch(...)
    {
        return;
    }

    // create a new frame and increment the counter
    rgbdtools::RGBDFrame frame;
    createRGBDFrameFromROSMessages(rgb_msg, depth_msg, info_msg, frame);
    frame.index = rgbd_frame_index_;
    rgbd_frame_index_++;
    std::vector<aruco::Marker> markers;

    ros::WallTime start_detect = ros::WallTime::now();

    MDetector.detect(frame.rgb_img,markers,param,0.20); // magic number 0.20 = marker size
    if (markers.size()>0)
    {
        manual_add_ = true;
        solver_.giveMarkers(markers);
    }
    ros::WallTime end_detect = ros::WallTime::now();

    //    bool result = processFrame(frame, eigenAffineFromTf(transform));
    addKeyframe(frame,eigenAffineFromTf( transform));

    ros::WallTime prepare_start = ros::WallTime::now();
    graph_detector_.prepareFeaturesForRANSAC_Iterative(keyframes_.size()-1,keyframes_);
    ros::WallTime prepare_end = ros::WallTime::now();

    ros::WallTime matrix_start = ros::WallTime::now();
    graph_detector_.buildMatchMatrixSurfTree_Iterative(keyframes_.size()-1,keyframes_);
    ros::WallTime matrix_end = ros::WallTime::now();

    ros::WallTime candidate_start = ros::WallTime::now();
    graph_detector_.buildCandidateMatrixSurfTree_Iterative(keyframes_.size()-1);
    ros::WallTime candidate_end = ros::WallTime::now();

    ros::WallTime correspondence_start = ros::WallTime::now();
    graph_detector_.buildCorrespondenceMatrix_mine_Iterative(keyframes_.size()-1,keyframes_,associations_);
    ros::WallTime correspondence_end = ros::WallTime::now();

    ros::WallTime solver_start = ros::WallTime::now();
    solver_.add_edges(keyframes_.size()-1,keyframes_,associations_);
    solver_.solve_Iterative(keyframes_);
    solver_.getArucoPos(&aruco_pos,&aruco_kfs,keyframes_);
    ros::WallTime solver_end = ros::WallTime::now();

    ros::WallTime filter_start = ros::WallTime::now();
    filter_aruco();
    ros::WallTime filter_end = ros::WallTime::now();

//            ros::ServiceClient client = nh_.serviceClient<ccny_rgbd::Save>("reset_pos_");
//            ccny_rgbd::Save save;

//            if(keyframes_[keyframes_.size()-1].used == true)
//            {
//                Pose pose = keyframes_[keyframes_.size()-1].pose;
//                std::stringstream text;
//                text << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << '\n' <<
//                        pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << '\n' <<
//                        pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << '\n' <<
//                        pose(3,0) << " " << pose(3,1) << " " << pose(3,2) << " " << pose(3,3) << '\n';
//                save.request.filename = text.str();
//                //            std::cout << text.str() << std::endl;
//                client.call(save);
//            }



    ros::WallTime publish_start = ros::WallTime::now();
    publishKeyframePoses();
    publishPath();
    publishArucoPos();
    publishKeyframeAssociations();
    ros::WallTime publish_end = ros::WallTime::now();
    std::cout << "Statistics:" << std::endl;
    std::cout << "Prepare Features:        "    << 1000.0 * (prepare_end    - prepare_start   ).toSec() << std::endl;
    std::cout << "Building Matrix:         "    << 1000.0 * (matrix_end    - matrix_start   ).toSec() << std::endl;
    std::cout << "Finding Candidates:      "    << 1000.0 * (candidate_end    - candidate_start   ).toSec() << std::endl;
    std::cout << "Creating Correspondeces: "    << 1000.0 * (correspondence_end    - correspondence_start   ).toSec() << std::endl;
    std::cout << "Solving graph:           "    << 1000.0 * (solver_end    - solver_start   ).toSec() << std::endl;
    std::cout << "Filtering Aruco:         "    << 1000.0 * (filter_end    - filter_start   ).toSec() << std::endl;
    std::cout << "Publishing Results:      "    << 1000.0 * (publish_end    - publish_start   ).toSec() << std::endl;

}

int triple_add=0;


bool Navigator::processFrame(
        const rgbdtools::RGBDFrame& frame,
        const AffineTransform& pose)
{
    // add the frame pose to the path vector
    geometry_msgs::PoseStamped frame_pose;
    tf::Transform frame_tf = tfFromEigenAffine(pose);
    tf::poseTFToMsg(frame_tf, frame_pose.pose);

    // update the header of the pose for the path
    frame_pose.header.frame_id = fixed_frame_;
    frame_pose.header.seq = frame.header.seq;
    frame_pose.header.stamp.sec = frame.header.stamp.sec;
    frame_pose.header.stamp.nsec = frame.header.stamp.nsec;

    path_msg_.poses.push_back(frame_pose);

    // determine if a new keyframe is needed
    bool result;

    if(keyframes_.empty() || manual_add_)
    {
        result = true;
    }
    else
    {
        double dist, angle;
        getTfDifference(tfFromEigenAffine(pose),
                        tfFromEigenAffine(keyframes_.back().pose),
                        dist, angle);

        if (triple_add >0 || dist > kf_dist_eps_ || angle > kf_angle_eps_)
        {
            result = true;
            //        if (triple_add == 0)
            //        {
            //            triple_add = 2;
            //        }
            //        else
            //        {
            //            triple_add--;
            //        }
        }
        else
            result = false;
    }

    if (result)
    {
        addKeyframe(frame, pose);
    }
    return result;
}

void Navigator::addKeyframe(
        const rgbdtools::RGBDFrame& frame,
        const AffineTransform& pose)
{
    rgbdtools::RGBDKeyframe keyframe(frame);
    keyframe.pose = pose;

    if (manual_add_)
    {
        ROS_INFO("Adding frame manually");
        manual_add_ = false;
        keyframe.manually_added = true;
    }
    keyframes_.push_back(keyframe);
}

bool Navigator::publishKeyframeSrvCallback(
        PublishKeyframe::Request& request,
        PublishKeyframe::Response& response)
{
    int kf_idx = request.id;

    if (kf_idx >= 0 && kf_idx < (int)keyframes_.size())
    {
        ROS_INFO("Publishing keyframe %d", kf_idx);
        publishKeyframeData(kf_idx);
        Pose(kf_idx);
        return true;
    }
    else
    {
        ROS_ERROR("Index out of range");
        return false;
    }
}

bool Navigator::publishKeyframesSrvCallback(
        PublishKeyframes::Request& request,
        PublishKeyframes::Response& response)
{
    bool found_match = false;

    // regex matching - try match the request string against each
    // keyframe index
    boost::regex expression(request.re);

    for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
    {
        std::stringstream ss;
        ss << kf_idx;
        std::string kf_idx_string = ss.str();

        boost::smatch match;

        if(boost::regex_match(kf_idx_string, match, expression))
        {
            found_match = true;
            ROS_INFO("Publishing keyframe %d", kf_idx);
            publishKeyframeData(kf_idx);
            publishKeyframePose(kf_idx);
            usleep(25000);
        }
    }

    publishPath();

    return found_match;
}

void Navigator::publishKeyframeData(int i)
{
    //    //for testing iterative stuff
    //    graph_detector_.prepareFeaturesForRANSAC_Iterative(i,keyframes_);
    //    graph_detector_.buildMatchMatrixSurfTree_Iterative(i,keyframes_);
    //    graph_detector_.buildCandidateMatrixSurfTree_Iterative(i);
    rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

    // construct a cloud from the images
    PointCloudT cloud;
    keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);

    // cloud transformed to the fixed frame
    PointCloudT cloud_ff;
    pcl::transformPointCloud(cloud, cloud_ff, keyframe.pose);

    cloud_ff.header.frame_id = fixed_frame_;

    keyframes_pub_.publish(cloud_ff);
}

void Navigator::publishKeyframeAssociations()
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = fixed_frame_;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points.resize(associations_.size() * 2);

    marker.color.a = 1.0;

    for (unsigned int as_idx = 0; as_idx < associations_.size(); ++as_idx)
    {
        // set up shortcut references
        const rgbdtools::KeyframeAssociation& association = associations_[as_idx];
        int kf_idx_a = association.kf_idx_a;
        int kf_idx_b = association.kf_idx_b;
        rgbdtools::RGBDKeyframe& keyframe_a = keyframes_[kf_idx_a];
        rgbdtools::RGBDKeyframe& keyframe_b = keyframes_[kf_idx_b];

        int idx_start = as_idx*2;
        int idx_end   = as_idx*2 + 1;

        tf::Transform keyframe_a_pose = tfFromEigenAffine(keyframe_a.pose);
        tf::Transform keyframe_b_pose = tfFromEigenAffine(keyframe_b.pose);

        // start point for the edge
        marker.points[idx_start].x = keyframe_a_pose.getOrigin().getX();
        marker.points[idx_start].y = keyframe_a_pose.getOrigin().getY();
        marker.points[idx_start].z = keyframe_a_pose.getOrigin().getZ();

        // end point for the edge
        marker.points[idx_end].x = keyframe_b_pose.getOrigin().getX();
        marker.points[idx_end].y = keyframe_b_pose.getOrigin().getY();
        marker.points[idx_end].z = keyframe_b_pose.getOrigin().getZ();

        if (association.type == rgbdtools::KeyframeAssociation::VO)
        {
            marker.ns = "VO";
            marker.scale.x = 0.002;

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if (association.type == rgbdtools::KeyframeAssociation::RANSAC)
        {
            marker.ns = "RANSAC";
            marker.scale.x = 0.002;

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        kf_assoc_pub_.publish(marker);
    }
}

void Navigator::publishKeyframePoses()
{
    for(unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
    {
        publishKeyframePose(kf_idx);
    }
}

void Navigator::publishArucoPos()
{
    for(u_int i = 0; i < aruco_pos.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = fixed_frame_;
        marker.ns = "Aruco_pos";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = aruco_pos[i][0];
        marker.pose.position.y = aruco_pos[i][1];
        marker.pose.position.z = aruco_pos[i][2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        poses_pub_.publish(marker);
    }
}
void Navigator::publishPath()
{
    PathMsg kfs;
    kfs.header.frame_id = fixed_frame_;
    kfs.header.seq = 0;
    kfs.header.stamp = ros::Time::now();
    for(u_int i = solver_.n_original_poses; i < keyframes_.size(); i++)
    {
        const rgbdtools::RGBDKeyframe& keyframe=keyframes_[i];

        if(!keyframe.used)
        {
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = fixed_frame_;
        pose.header.seq = keyframe.header.seq;
        pose.header.stamp.sec = keyframe.header.stamp.sec;
        pose.header.stamp.nsec = keyframe.header.stamp.nsec;
        pose.pose.position.x = keyframe.pose(0,3);
        pose.pose.position.y = keyframe.pose(1,3);
        pose.pose.position.z = keyframe.pose(2,3);
        pose.pose.orientation.x = 1;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        kfs.poses.push_back(pose);
    }
    if(kfs.poses.size()!=0)
        path_pub_.publish(kfs);
}

void Navigator::publishKeyframePose(int i)
{
    rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

    // **** publish camera pose

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = fixed_frame_;
    marker.ns = "keyframe_poses";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);

    // start point for the arrow
    tf::Transform keyframe_pose = tfFromEigenAffine(keyframe.pose);
    marker.points[0].x = keyframe_pose.getOrigin().getX();
    marker.points[0].y = keyframe_pose.getOrigin().getY();
    marker.points[0].z = keyframe_pose.getOrigin().getZ();

    // end point for the arrow
    tf::Transform ep;
    ep.setIdentity();
    ep.setOrigin(tf::Vector3(0.00, 0.00, 0.12)); // z = arrow length
    ep = keyframe_pose * ep;

    marker.points[1].x = ep.getOrigin().getX();
    marker.points[1].y = ep.getOrigin().getY();
    marker.points[1].z = ep.getOrigin().getZ();

    marker.scale.x = 0.02; // shaft radius
    marker.scale.y = 0.05; // head radius
    if((u_int)i < solver_.n_original_poses)
    {
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else
    {
        if(!keyframe.used)
        {
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

    }
    poses_pub_.publish(marker);

    // **** publish frame index text

    visualization_msgs::Marker marker_text;
    marker_text.header.stamp = ros::Time::now();
    marker_text.header.frame_id = fixed_frame_;
    marker_text.ns = "keyframe_indexes";
    marker_text.id = i;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;

    tf::poseTFToMsg(keyframe_pose, marker_text.pose);

    marker_text.pose.position.z -= 0.05;

    char label[6];
    sprintf(label, "%d", i);
    marker_text.text = label;

    marker_text.color.a = 1.0;
    marker_text.color.r = 1.0;
    marker_text.color.g = 1.0;
    marker_text.color.b = 0.0;

    marker_text.scale.z = 0.05; // shaft radius

    poses_pub_.publish(marker_text);
}

bool Navigator::saveKeyframesSrvCallback(
        Save::Request& request,
        Save::Response& response)
{
    std::string filepath = request.filename;

    ROS_INFO("Saving keyframes...");
    std::string filepath_keyframes = filepath + "/keyframes/";
    bool result_kf = saveKeyframes(keyframes_, filepath_keyframes);
    if (result_kf) ROS_INFO("Keyframes saved to %s", filepath.c_str());
    else ROS_ERROR("Keyframe saving failed!");

    ROS_INFO("Saving path...");
    bool result_path = savePath(filepath);
    savePathTUMFormat(filepath);
    if (result_path ) ROS_INFO("Path saved to %s", filepath.c_str());
    else ROS_ERROR("Path saving failed!");
    solver_.save_graph(filepath);

    return result_kf && result_path;
}

bool Navigator::loadKeyframesSrvCallback(
        Load::Request& request,
        Load::Response& response)
{
    bool skip = false;
    ros::WallTime start_all = ros::WallTime::now();
    std::string filepath1 = request.filename;
    char buffer[200];
    std::string filepath;
    if(filepath1.at(filepath1.size()-1)=='!')
    {
        skip = true;
        int copied = filepath1.copy(buffer,filepath1.size()-1,0);
        buffer[copied] = '\0';
        filepath = buffer;
    }
    else
    {
        filepath = filepath1;
    }
    ROS_INFO("Loading keyframes...");
    std::string filepath_keyframes = filepath + "/keyframes/";
    bool result_kf = loadKeyframes(keyframes_, filepath_keyframes);
    if (result_kf) ROS_INFO("Keyframes loaded successfully");
    else ROS_ERROR("Keyframe loading failed!");

    ROS_INFO("Loading path...");
    bool result_path = loadPath(filepath);
    if (result_path) ROS_INFO("Path loaded successfully");
    else ROS_ERROR("Path loading failed!");
    publishKeyframePoses();
    solver_.parse((filepath).c_str(),keyframes_);
    publishKeyframePoses();
    if(!(result_kf && result_path))
        return false;
    if(skip)
        return result_kf && result_path;
    graph_detector_.generateKeyframeAssociations_Iterative(keyframes_,associations_);
    double dur = getMsDuration(start_all);

    ROS_INFO("Everything took %.1f ms", dur);
    return result_kf && result_path;
}

bool Navigator::savePcdMapSrvCallback(
        Save::Request& request,
        Save::Response& response)
{
    ROS_INFO("Saving map as pcd...");
    const std::string& path = request.filename;
    bool result = savePcdMap(path);

    if (result) ROS_INFO("Pcd map saved to %s", path.c_str());
    else ROS_ERROR("Pcd map saving failed");

    return result;
}

bool Navigator::saveOctomapSrvCallback(
        Save::Request& request,
        Save::Response& response)
{
    ROS_INFO("Saving map as Octomap...");
    const std::string& path = request.filename;
    bool result = saveOctomap(path);

    if (result) ROS_INFO("Octomap saved to %s", path.c_str());
    else ROS_ERROR("Octomap saving failed");

    return result;
}

//void Navigator::createmap(octomap::ColorOcTree* m_octree)
//{
//    nav_msgs::OccupancyGrid m_gridmap;
//    m_gridmap.data.clear();
//    m_gridmap.info.height = 0.0;
//    m_gridmap.info.width = 0.0;
//    m_gridmap.info.resolution = 0.0;
//    m_gridmap.info.origin.position.x = 0.0;
//    m_gridmap.info.origin.position.y = 0.0;
//    m_gridmap.header.frame_id = fixed_frame_;
//    m_gridmap.header.stamp = ros::Time::now();
//    // now, traverse all leafs in the tree:
//    for (octomap::ColorOcTree::iterator it = m_octree->begin(999),
//         end = m_octree->end(); it != end; ++it)
//    {
//        if (m_octree->isNodeOccupied(*it)){
//            double z = it.getZ();
//            if (z > min_map_z_ && z < max_map_z_)
//            {
//                double size = it.getSize();
//                double x = it.getX();
//                double y = it.getY();


//                if (publishCollisionMap){
//                    collObjBox.extents.x = collObjBox.extents.y = collObjBox.extents.z = size;

//                    collObjBox.center.x = x;
//                    collObjBox.center.y = y;
//                    collObjBox.center.z = z;
//                    collisionMap.boxes.push_back(collObjBox);

//                }

//                //create marker:
//                if (publishMarkerArray){
//                    unsigned idx = it.getDepth();
//                    assert(idx < occupiedNodesVis.markers.size());

//                    geometry_msgs::Point cubeCenter;
//                    cubeCenter.x = x;
//                    cubeCenter.y = y;
//                    cubeCenter.z = z;

//                    occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
//                    if (m_useHeightMap){
//                        double minX, minY, minZ, maxX, maxY, maxZ;
//                        m_octree->getMetricMin(minX, minY, minZ);
//                        m_octree->getMetricMax(maxX, maxY, maxZ);

//                        double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
//                        occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
//                    }
//                }

//                // insert into pointcloud:
//                if (publishPointCloud)
//                    pclCloud.push_back(pcl::PointXYZ(x, y, z));

//            }
//        } else{ // node not occupied => mark as free in 2D map if unknown so far
//            double z = it.getZ();
//            if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
//            {
//                handleFreeNode(it);
//                if (inUpdateBBX)
//                    handleFreeNodeInBBX(it);

//                if (m_publishFreeSpace){
//                    double x = it.getX();
//                    double y = it.getY();

//                    if (publishFreeMap){
//                        freeObjBox.extents.x = freeObjBox.extents.y = freeObjBox.extents.z = it.getSize();
//                        freeObjBox.center.x = x;
//                        freeObjBox.center.y = y;
//                        freeObjBox.center.z = z;
//                        freeMap.boxes.push_back(freeObjBox);
//                    }

//                    //create marker for free space:
//                    if (publishFreeMarkerArray){
//                        unsigned idx = it.getDepth();
//                        assert(idx < freeNodesVis.markers.size());

//                        geometry_msgs::Point cubeCenter;
//                        cubeCenter.x = x;
//                        cubeCenter.y = y;
//                        cubeCenter.z = z;

//                        freeNodesVis.markers[idx].points.push_back(cubeCenter);
//                    }
//                }

//            }
//        }
//    }
//}

bool Navigator::addManualKeyframeSrvCallback(
        AddManualKeyframe::Request& request,
        AddManualKeyframe::Response& response)
{

    manual_add_ = true;

    return true;
}

bool Navigator::generateGraphSrvCallback(
        GenerateGraph::Request& request,
        GenerateGraph::Response& response)
{
    associations_.clear();
    graph_detector_.generateKeyframeAssociations(keyframes_, associations_);

    ROS_INFO("%d associations detected", (int)associations_.size());
    publishKeyframePoses();
    publishKeyframeAssociations();

    return true;
}

bool Navigator::solveGraphSrvCallback(
        SolveGraph::Request& request,
        SolveGraph::Response& response)
{
    ros::WallTime start = ros::WallTime::now();

    // Graph solving: keyframe positions only, path is interpolated
    //  graph_solver_.solve(keyframes_, associations_);
    solver_.solve(keyframes_, associations_);
    //  updatePathFromKeyframePoses();
    max_map_z_ = getmaxheight(path_msg_)+0.2; //cut the ceiling
    min_map_z_= getminheight(path_msg_)-0.5; //cut the ground
    // Graph solving: keyframe positions and VO path
    /*
  AffineTransformVector path;
  pathROSToEigenAffine(path_msg_, path);
  graph_solver_.solve(keyframes_, associations_, path);
  pathEigenAffineToROS(path, path_msg_);
  */

    double dur = getMsDuration(start);

    ROS_INFO("Solving took %.1f ms", dur);

    //  publishPath();
    publishKeyframePoses();
    publishKeyframeAssociations();

    return true;
}


/** In the event that the keyframe poses change (from pose-graph solving)
 * this function will propagete teh changes in the path message
 */
void Navigator::updatePathFromKeyframePoses()
{
    int kf_size = keyframes_.size();
    int f_size = path_msg_.poses.size();

    // temporary store the new path
    AffineTransformVector path_new;
    path_new.resize(f_size);

    if (kf_size < 2) return;

    for (int kf_idx = 0; kf_idx < kf_size - 1; ++kf_idx)
    {
        // the indices of the current and next keyframes (a and b)
        const rgbdtools::RGBDKeyframe& keyframe_a = keyframes_[kf_idx];
        const rgbdtools::RGBDKeyframe& keyframe_b = keyframes_[kf_idx + 1];

        // the corresponding frame indices
        int f_idx_a = keyframe_a.index;
        int f_idx_b = keyframe_b.index;

        // the new poses of keyframes a and b (after graph solving)
        tf::Transform kf_pose_a = tfFromEigenAffine(keyframe_a.pose);
        tf::Transform kf_pose_b = tfFromEigenAffine(keyframe_b.pose);

        // the previous pose of keyframe a and b (before graph solving)
        tf::Transform kf_pose_a_prev, kf_pose_b_prev;
        tf::poseMsgToTF(path_msg_.poses[f_idx_a].pose, kf_pose_a_prev);
        tf::poseMsgToTF(path_msg_.poses[f_idx_b].pose, kf_pose_b_prev);

        // the motion, in the camera frame (after and before graph solving)
        tf::Transform kf_motion      = kf_pose_a.inverse() * kf_pose_b;
        tf::Transform kf_motion_prev = kf_pose_a_prev.inverse() * kf_pose_b_prev;

        // the correction from the graph solving
        tf::Transform correction = kf_motion_prev.inverse() * kf_motion;

        // update the poses in-between keyframes
        for (int f_idx = f_idx_a; f_idx < f_idx_b; ++f_idx)
        {
            // calculate interpolation scale
            double interp_scale = (double)(f_idx - f_idx_a) / (double)(f_idx_b - f_idx_a);

            // create interpolated correction translation and rotation
            tf::Vector3 v_interp = correction.getOrigin() * interp_scale;
            tf::Quaternion q_interp = tf::Quaternion::getIdentity();
            q_interp.slerp(correction.getRotation(), interp_scale);

            // create interpolated correction
            tf::Transform interpolated_correction;
            interpolated_correction.setOrigin(v_interp);
            interpolated_correction.setRotation(q_interp);

            // the previous frame pose
            tf::Transform frame_pose_prev;
            tf::poseMsgToTF(path_msg_.poses[f_idx].pose, frame_pose_prev);

            // the pevious frame motion
            tf::Transform frame_motion_prev = kf_pose_a_prev.inverse() * frame_pose_prev;

            // the interpolated motion
            tf::Transform interpolated_motion = frame_motion_prev * interpolated_correction;

            // calculate the interpolated pose
            path_new[f_idx] = keyframe_a.pose * eigenAffineFromTf(interpolated_motion);
        }
    }

    // update the last pose
    const rgbdtools::RGBDKeyframe& last_kf = keyframes_[kf_size - 1];

    tf::Transform last_kf_pose_prev;
    tf::poseMsgToTF(path_msg_.poses[last_kf.index].pose, last_kf_pose_prev);

    // update the poses in-between last keyframe and end of vo path
    for (int f_idx = last_kf.index; f_idx < f_size; ++f_idx)
    {
        // the previous frame pose
        tf::Transform frame_pose_prev;
        tf::poseMsgToTF(path_msg_.poses[f_idx].pose, frame_pose_prev);

        // the pevious frame motion
        tf::Transform frame_motion_prev = last_kf_pose_prev.inverse() * frame_pose_prev;

        // calculate the new pose
        path_new[f_idx] = last_kf.pose * eigenAffineFromTf(frame_motion_prev);
    }

    // copy over the interpolated path
    pathEigenAffineToROS(path_new, path_msg_);
}


bool Navigator::savePcdMap(const std::string& path)
{
    PointCloudT pcd_map;
    buildPcdMap(pcd_map);

    // write out
    pcl::PCDWriter writer;
    int result_pcd = writer.writeBinary<PointT>(path, pcd_map);

    if (result_pcd < 0) return false;
    else return true;
}

void Navigator::buildPcdMap(PointCloudT& map_cloud)
{
    PointCloudT::Ptr aggregate_cloud(new PointCloudT());
    aggregate_cloud->header.frame_id = fixed_frame_;

    // aggregate all frames into single cloud
    for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
    {
        const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];

        PointCloudT cloud;
        keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);

        PointCloudT cloud_tf;
        pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
        cloud_tf.header.frame_id = fixed_frame_;

        *aggregate_cloud += cloud_tf;
    }

    // filter cloud using voxel grid, and for max z
    pcl::VoxelGrid<PointT> vgf;
    vgf.setInputCloud(aggregate_cloud);
    vgf.setLeafSize(pcd_map_res_, pcd_map_res_, pcd_map_res_);
    vgf.setFilterFieldName("z");
    vgf.setFilterLimits (-std::numeric_limits<double>::infinity(), max_map_z_);

    vgf.filter(map_cloud);
}
void Navigator::buildFullCloud(PointCloudT& map_cloud)
{
    PointCloudT::Ptr aggregate_cloud(new PointCloudT());
    aggregate_cloud->header.frame_id = fixed_frame_;

    // aggregate all frames into single cloud
    for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
    {
        const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];

        PointCloudT cloud;
        keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);

        PointCloudT cloud_tf;
        pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
        cloud_tf.header.frame_id = fixed_frame_;

        *aggregate_cloud += cloud_tf;
    }
    map_cloud = *aggregate_cloud;
}

bool Navigator::saveOctomap(const std::string& path)
{
    bool result;

    if (octomap_with_color_)
    {
        octomap::ColorOcTree tree(octomap_res_);
        buildColorOctomap(tree);
        result = tree.write(path);
        //        std::cout << "Publishing octomap as pointcloud" << std::endl;
        //        octomap::point3d_list points_;
        //        tree.getOccupied(points_);
        //        PointCloudT cloud;
        //        for (octomap::point3d_list::iterator it=points_.begin(); it != points_.end(); ++it)
        //        {
        //            PointT point;
        //            point.x = it->x();
        //            point.y = it->y();
        //            point.z = it->z();
        //            point.rgb = 0;
        //            cloud.push_back(point);
        //        }
        //        cloud.header.frame_id = fixed_frame_;
        //        keyframes_pub_.publish(cloud);
    }
    else
        result = false;
    return result;
}


void Navigator::buildColorOctomap(octomap::ColorOcTree& tree)
{
    ROS_INFO("Building Octomap with color...");

    octomap::point3d sensor_origin(0.0, 0.0, 0.0);
    for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
    {
        ROS_INFO("Processing keyframe %u", kf_idx);
        const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];

        // construct the cloud
        PointCloudT::Ptr cloud_unf(new PointCloudT());
        keyframe.constructDensePointCloud(*cloud_unf, max_range_, max_stdev_);

        // perform filtering for max z
        pcl::transformPointCloud(*cloud_unf, *cloud_unf, keyframe.pose);
        PointCloudT cloud;
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud_unf);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-std::numeric_limits<double>::infinity(), max_map_z_);
        pass.filter(cloud);
        pcl::transformPointCloud(cloud, cloud, keyframe.pose.inverse());

        octomap::pose6d frame_origin = poseTfToOctomap(tfFromEigenAffine(keyframe.pose));

        // build octomap cloud from pcl cloud
        octomap::Pointcloud octomap_cloud;
        for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
        {
            const PointT& p = cloud.points[pt_idx];
            if (!std::isnan(p.z))
                octomap_cloud.push_back(p.x, p.y, p.z);
        }

        // insert scan (only xyz considered, no colors)
        tree.insertScan(octomap_cloud, sensor_origin, frame_origin);

        // insert colors
        PointCloudT cloud_tf;
        pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
        for (unsigned int pt_idx = 0; pt_idx < cloud_tf.points.size(); ++pt_idx)
        {
            const PointT& p = cloud_tf.points[pt_idx];
            if (!std::isnan(p.z))
            {
                octomap::point3d endpoint(p.x, p.y, p.z);
                octomap::ColorOcTreeNode* n = tree.search(endpoint);
                if (n) n->setColor(p.r, p.g, p.b);
            }
        }

        tree.updateInnerOccupancy();
    }
}

//void Navigator::publishPath()
//{
//    path_msg_.header.frame_id = fixed_frame_;
//    path_pub_.publish(path_msg_);
//}

bool Navigator::savePath(const std::string& filepath)
{
    // open file
    std::string filename = filepath + "/path.txt";
    std::ofstream file(filename.c_str());
    if (!file.is_open()) return false;

    file << "# index seq stamp.sec stamp.nsec x y z qx qy qz qw" << std::endl;

    for (unsigned int idx = 0; idx < path_msg_.poses.size(); ++idx)
    {
        const geometry_msgs::PoseStamped& pose = path_msg_.poses[idx];

        file << idx << " "
             << pose.header.seq << " "
             << pose.header.stamp.sec << " "
             << pose.header.stamp.nsec << " "
             << pose.pose.position.x << " "
             << pose.pose.position.y << " "
             << pose.pose.position.z << " "
             << pose.pose.orientation.x << " "
             << pose.pose.orientation.y << " "
             << pose.pose.orientation.z << " "
             << pose.pose.orientation.w << std::endl;
    }

    file.close();

    return true;
}

bool Navigator::savePathTUMFormat(const std::string& filepath)
{
    // open file
    std::string filename = filepath + "/path.tum.txt";
    std::ofstream file(filename.c_str());
    if (!file.is_open()) return false;

    file << "# stamp x y z qx qy qz qw" << std::endl;

    for (unsigned int idx = 0; idx < path_msg_.poses.size(); ++idx)
    {
        const geometry_msgs::PoseStamped& pose = path_msg_.poses[idx];

        file << pose.header.stamp.sec << "."
             << pose.header.stamp.nsec << " "
             << pose.pose.position.x << " "
             << pose.pose.position.y << " "
             << pose.pose.position.z << " "
             << pose.pose.orientation.x << " "
             << pose.pose.orientation.y << " "
             << pose.pose.orientation.z << " "
             << pose.pose.orientation.w << std::endl;
    }

    file.close();

    return true;
}

bool Navigator::loadPath(const std::string& filepath)
{
    path_msg_.poses.clear();

    // open file
    std::string filename = filepath + "/path.txt";
    std::ifstream file(filename.c_str());
    if (!file.is_open()) return false;

    std::string line;

    // get header
    getline(file, line);

    // read each line
    while(std::getline(file, line))
    {
        std::istringstream is(line);

        // fill out pose information
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = fixed_frame_;
        int idx;

        is >> idx
           >> pose.header.seq
           >> pose.header.stamp.sec
           >> pose.header.stamp.nsec
           >> pose.pose.position.x
           >> pose.pose.position.y
           >> pose.pose.position.z
           >> pose.pose.orientation.x
           >> pose.pose.orientation.y
           >> pose.pose.orientation.z
           >> pose.pose.orientation.w;

        // add to poses vector
        path_msg_.poses.push_back(pose);
    }

    file.close();
    return true;
}

} // namespace ccny_rgbd


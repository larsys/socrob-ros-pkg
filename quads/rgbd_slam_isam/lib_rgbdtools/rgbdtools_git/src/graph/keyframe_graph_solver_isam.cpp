/**
 *  @file keyframe_graph_solver_g2o.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  @note based on GraphOptimizer_G2O.cpp by Miguel Algaba Borrego
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

#include "isam/isam.h"
#include "rgbdtools/graph/keyframe_graph_solver_isam.h"
#include "rgbdtools/map_util.h"
#include "rgbdtools/rgbd_util.h"
#include "Eigen/Geometry"

namespace rgbdtools {

KeyframeGraphSolverISAM::KeyframeGraphSolverISAM()
{
//    isam::Properties prop;
//    prop.method = isam::DOG_LEG;
//    slam.set_properties(prop);
    n_original_poses = 0;

}

KeyframeGraphSolverISAM::~KeyframeGraphSolverISAM()
{

}
void KeyframeGraphSolverISAM::giveMarkers( std::vector<aruco::Marker> markers)
{
    markers_ = markers;
}

void KeyframeGraphSolverISAM::save_graph(std::string filepath)
{
    slam.save(filepath+"/out.graph");
    save_landmarks(filepath);
}

void KeyframeGraphSolverISAM::save_landmarks(std::string filepath)
{
    std::ofstream ldmrk_file;
    ldmrk_file.open ((filepath+"/out.lnd").c_str());
    if(!ldmrk_file.is_open())
    {
        std::cout << "Landmark file wasnt saved." << std::endl;
        return;
    }
    ldmrk_file << "# Keyframe Match Landmark" << std::endl;
    for(std::map<Observation,isam::Point3d_Node*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it)
    {
        ldmrk_file << it->first.first << " " << it->first.second << " " << it->second->unique_id() << std::endl;
    }
    ldmrk_file.close();
    return;
}
void KeyframeGraphSolverISAM::load_landmarks(std::string filepath)
{
    FILE* file = fopen(filepath.c_str(), "r");
    if(file == NULL)
    {
        std::cout << "Landmark file wasnt read." << std::endl;
        return;
    }

    while (!feof(file)) {
       char str[2000];
       if (fgets(str, 2000, file)) {
           if(str[0] == '#') //commnent
               continue;
           int kf,match,land;
           int res = 0;
           res = sscanf(str, "%i %i %i",&kf,&match,&land);
           if(res != 3)
           {
               std::cout << "Syntax error in landmark file" << std::endl;
               continue;
           }
           Observation o;
           o.first = kf;
           o.second = match;
           landmarks[o] = landmarks_explicit[land-poses.size()];
       }
     }
    return;
}

Pose tfFromMeans(const Eigen::Vector3f& t)
{
    Pose pose = Eigen::Affine3f(Eigen::Translation3f(t));
    return pose;
}
Eigen::Matrix<double,3,1> XYZfromPose(const Pose& t)
{
    Eigen::Matrix<double,3,1> pos;
    pos[0]=t(0,3);
    pos[1]=t(1,3);
    pos[2]=t(2,3);
    return pos;
}



void KeyframeGraphSolverISAM::add_edge(isam::Pose3d_Node* from, isam::Pose3d_Node* to, const AffineTransform& pose, isam::Noise noise)
{
    // TODO: use eigen quaternion, not manual conversion
    //Transform Eigen::Matrix4f into 3D traslation and rotation for g2o
    Eigen::Matrix4d pose_ = m4dfromPose(pose);
    isam::Pose3d transf(pose_);
    isam::Pose3d_Pose3d_Factor *edge = new isam::Pose3d_Pose3d_Factor(from,to,transf,noise);
    slam.add_factor(edge);
    vodom.push_back(edge);
}
void KeyframeGraphSolverISAM::add_edges(u_int kf_idx,KeyframeVector& keyframes,
                                        const KeyframeAssociationVector& associations)
{
    isam::Noise inf_pose = isam::SqrtInformation(1*isam::eye(6));
    isam::Noise inf_ransac = isam::SqrtInformation(60*isam::eye(6));

    bool hit = false;
    add_pose(keyframes[kf_idx].pose,inf_pose);
    valid_poses.resize(keyframes.size());
    for(int  i = associations.size()-1; i>=0; i--)
    {
        KeyframeAssociation association = associations[i];
        if(association.kf_idx_a != kf_idx && association.kf_idx_b != kf_idx)
        {
            break;
        }
        hit = true;
        add_edge(poses[association.kf_idx_a],poses[association.kf_idx_b],association.a2b,inf_ransac);
    }
    if(hit)
        valid_poses[kf_idx] = true;
    else
        valid_poses[kf_idx] = false;
    int old_size=valid_markers.size();
    valid_markers.resize(valid_markers.size()+markers_.size());
    for(u_int i = 0; i < markers_.size(); i++)
    {
        if(hit)
            valid_markers[old_size+i]=true;
        else
            valid_markers[old_size+i]=false;
        const aruco::Marker marker = markers_[i];
        isam::Noise info = isam::SqrtInformation(6*isam::eye(3));
        marker_kf.push_back(kf_idx);
        add_aruco(marker,poses[kf_idx],info);
//        isam::Point3d measure1(marker.Tvec.ptr<float>(0)[0],marker.Tvec.ptr<float>(0)[1],marker.Tvec.ptr<float>(0)[2]);
//        RGBDFrame keyframe = keyframes[kf_idx];
//        aruco::Marker m=marker;
//        std::vector<std::vector<float> > points;
//        std::vector<float> point;
//        point.push_back(m[0].x);
//        point.push_back(m[0].y);
//        points.push_back(point);
//        point.clear();
//        point.push_back(m[1].x);
//        point.push_back(m[1].y);
//        points.push_back(point);
//        point.clear();
//        point.push_back(m[2].x);
//        point.push_back(m[2].y);
//        points.push_back(point);
//        point.clear();
//        point.push_back(m[3].x);
//        point.push_back(m[3].y);
//        points.push_back(point);
//        std::vector<std::vector<double> > dists;
//        keyframe.getPointsDist(points,&dists);
//        std::cout << " " <<std::endl;
//        std::vector<double> mean;
//        mean.push_back(0.0);
//        mean.push_back(0.0);
//        mean.push_back(0.0);
//        for (u_int k = 0; k < dists.size(); k++)
//        {
//            mean[0]+=(dists[k][0]);
//            mean[1]+=(dists[k][1]);
//            mean[2]+=(dists[k][2]);
//        }
//        mean[0]/=dists.size();
//        mean[1]/=dists.size();
//        mean[2]/=dists.size();
//        std::cout << measure1 << std::endl;
//        std::cout << mean[0] << " " << mean[1] << " "<< mean[2] << std::endl;
    }
}
void KeyframeGraphSolverISAM::add_edge(isam::Pose3d_Node* from, isam::Pose3d_Node* to,double x, double y, double z, double roll, double pitch, double yaw, isam::Noise noise)
{

    isam::Rot3d r(yaw,pitch,roll);
    isam::Point3d p(x,y,z);
    isam::Pose3d transf(p,r);
    isam::Pose3d_Pose3d_Factor *edge = new isam::Pose3d_Pose3d_Factor(from,to,transf,noise);
    slam.add_factor(edge);
    vodom.push_back(edge);
}
std::vector<std::string> factors;
std::vector<std::string> nodes;
void KeyframeGraphSolverISAM::parse_nodes()
{
    for (u_int i = 0; i< nodes.size() ; i++)
    {
        char str[2000];
        strcpy(str,nodes[i].c_str());
        char keyword_c[2000];
        int key_length;
        sscanf(str, "%s%n", keyword_c, &key_length);
        const char* arguments = &str[key_length];
        std::string keyword(keyword_c);

        if (keyword == "Pose3d_Node"  )
        {
            int idx;
            double x,y,z,roll,pitch,yaw;
            int res = sscanf(arguments, "%i (%lg, %lg, %lg; %lg, %lg, %lg)",
                             &idx, &x, &y, &z, &yaw, &pitch,&roll);
            isam::Noise inf = isam::SqrtInformation(100*isam::eye(6));
            add_pose(x,y,z,roll,pitch,yaw,inf);
        }
        else
        {
            std::cout << "Error: I shouldnt find a " << keyword << " here." << std::endl;
        }

    }
}

void KeyframeGraphSolverISAM::parse_factors()
{
    for (u_int i = 0; i< factors.size() ; i++)
    {
        char str[2000];
        strcpy(str,factors[i].c_str());
        char keyword_c[2000];
        int key_length;
        sscanf(str, "%s%n", keyword_c, &key_length);
        const char* arguments = &str[key_length];
        std::string keyword(keyword_c);
//        if (keyword == "Pose3d_Factor"  )
//        {
//            continue;
//            int idx;
//            double x,y,z,roll,pitch,yaw,i11,i12,i13,i14,i15,i16,i22,i23,i24,i25,i26,i33, i34, i35, i36, i44, i45, i46, i55, i56, i66;
//            int res = sscanf(arguments, "%i (%lg, %lg, %lg; %lg, %lg, %lg) {%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg}",
//                             &idx, &x, &y, &z, &yaw, &pitch,&roll,
//                             &i11, &i12, &i13, &i14, &i15, &i16, &i22, &i23, &i24, &i25, &i26,
//                             &i33, &i34, &i35, &i36, &i44, &i45, &i46, &i55, &i56, &i66);
//            if(res != 28)
//            {
//                std::cout << "Error on input graph file: ->" << std::endl << str << std::endl;
//                continue;
//            }
//            Eigen::MatrixXd sqrtinf(6,6);
//            sqrtinf <<
//                       i11, i12, i13, i14, i15, i16,
//                    0., i22, i23, i24, i25, i26,
//                    0.,  0., i33, i34, i35, i36,
//                    0.,  0.,  0., i44, i45, i46,
//                    0.,  0.,  0.,  0., i55, i56,
//                    0.,  0.,  0.,  0.,  0., i66;

//            isam::Noise inf = isam::SqrtInformation (sqrtinf);
//            add_prior(poses[idx],x,y,z,roll,pitch,yaw,inf);
//            continue;
//        }
        if(keyword == "Pose3d_Pose3d_Factor" )
        {
            int idx_0,idx_1;
            double x,y,z,roll,pitch,yaw,i11,i12,i13,i14,i15,i16,i22,i23,i24,i25,i26,i33, i34, i35, i36, i44, i45, i46, i55, i56, i66;
            int res = sscanf(arguments, "%i %i (%lg, %lg, %lg; %lg, %lg, %lg) {%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg,%lg}",
                             &idx_0,&idx_1, &x, &y, &z ,&yaw, &pitch, &roll,
                             &i11, &i12, &i13, &i14, &i15, &i16, &i22, &i23, &i24, &i25, &i26,
                             &i33, &i34, &i35, &i36, &i44, &i45, &i46, &i55, &i56, &i66);
            if(res != 29)
            {
                std::cout << "Error on input graph file: ->" << std::endl << str << std::endl;
                continue;
            }
            Eigen::MatrixXd sqrtinf(6,6);
            sqrtinf <<
                       i11, i12, i13, i14, i15, i16,
                    0., i22, i23, i24, i25, i26,
                    0.,  0., i33, i34, i35, i36,
                    0.,  0.,  0., i44, i45, i46,
                    0.,  0.,  0.,  0., i55, i56,
                    0.,  0.,  0.,  0.,  0., i66;

            isam::Noise inf = isam::SqrtInformation (sqrtinf);
            add_edge(poses[idx_0],poses[idx_1],x,y,z,roll,pitch,yaw,inf);
            continue;
        }
//        if(keyword == "Pose3d_Point3d_Factor" )
//        {
//            int idx_0,idx_1;
//            double x,y,z,i11,i12,i13,i22,i23,i33;
//            int res = sscanf(arguments, "%i %i (%lg, %lg, %lg) {%lg,%lg,%lg,%lg,%lg,%lg}",
//                             &idx_0,&idx_1, &x, &y, &z,
//                             &i11, &i12, &i13,
//                             &i22, &i23,
//                             &i33);
//            if(res != 11)
//            {
//                std::cout << "Error on input graph file: ->" << std::endl << str << std::endl;
//                continue;
//            }
//            Eigen::MatrixXd sqrtinf(3,3);
//            sqrtinf <<
//                       i11, i12, i13,
//                    0., i22, i23,
//                    0.,  0., i33;


//            isam::Noise inf = isam::SqrtInformation (sqrtinf);

//            add_landmark_from_file(idx_0,idx_1-poses.size(),x,y,z,inf);
//            continue;
//        }
    }
}

int counter = 0;
void KeyframeGraphSolverISAM::parse_line(char* str)
{

    char keyword_c[2000];
    sscanf(str, "%s", keyword_c);
    std::string keyword(keyword_c);
    if (keyword == "Pose3d_Factor" || keyword == "Pose3d_Point3d_Factor" || keyword == "Pose3d_Pose3d_Factor" )
    {
        std::string k(str);
        factors.push_back(k);
        return;
    }
    if(keyword == "Pose3d_Node" )
    {
        std::string k(str);
        nodes.push_back(k);
        return;
    }
    if(keyword == "Point3d_Node")
    {
//        isam::Point3d_Node* node = new isam::Point3d_Node;
//        slam.add_node(node);
//        landmarks_explicit.push_back(node);
//        counter++;
        return;
    }
    std::cout << str << std::endl;
}

void KeyframeGraphSolverISAM::parse (std::string fname, KeyframeVector keyframes)
{
    FILE* file = fopen((fname+"/out.graph").c_str(), "r");
    if(file == NULL)
        return;
    while (!feof(file)) {
       char str[2000];
       if (fgets(str, 2000, file)) {
         parse_line(str);
       }
     }
     fclose(file);
     parse_nodes();
//     for (u_int i = 0; i< counter ; i++)
//     {
//         isam::Point3d_Node* node = new isam::Point3d_Node;
//         slam.add_node(node);
//         landmarks_explicit.push_back(node);
//     }
     parse_factors();
     //we still need to load landmark observations
//     load_landmarks(fname+"/out.lnd");
//     save_graph(fname+"/teste");
     std::cout << "Solving Graph" << std::endl;
     optimizeGraph();
     AffineTransformVector optimized_poses;
     optimized_poses.resize(keyframes.size());

     getOptimizedPoses(optimized_poses);
     valid_poses.resize(keyframes.size());
     for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
     {
         valid_poses[kf_idx] = true;
         RGBDKeyframe& keyframe = keyframes[kf_idx];
         keyframe.pose = optimized_poses[kf_idx];
         keyframe.used = true;
     }
     n_original_poses = keyframes.size();
}



void KeyframeGraphSolverISAM::add_pose(Pose pose,isam::Noise noise)
{

    isam::Pose3d_Node* pose_node = new isam::Pose3d_Node();
    slam.add_node(pose_node);
    poses.push_back(pose_node);

    Eigen::Matrix4d pose_ = m4dfromPose(pose);

    isam::Pose3d vo(pose_);
    isam::Factor* prior;

    prior = new isam::Pose3d_Factor(pose_node,vo,noise);
    slam.add_factor(prior);
    priors.push_back(prior);

}
void KeyframeGraphSolverISAM::add_pose(double x, double y, double z, double roll, double pitch, double yaw,isam::Noise noise)
{

    isam::Pose3d_Node* pose_node = new isam::Pose3d_Node();
    slam.add_node(pose_node);
    poses.push_back(pose_node);
    isam::Point3d p(x,y,z);
    isam::Rot3d r(yaw,pitch,roll);
    isam::Pose3d vo(p,r);
    isam::Factor* prior;

    prior = new isam::Pose3d_Factor(pose_node,vo,noise);

    slam.add_factor(prior);
    priors.push_back(prior);

}
void KeyframeGraphSolverISAM::add_prior(isam::Pose3d_Node* pose_node, double x, double y, double z, double roll, double pitch, double yaw,isam::Noise noise)
{

    isam::Point3d p(x,y,z);
    isam::Rot3d r(yaw,pitch,roll);
    isam::Pose3d vo(p,r);
    isam::Factor* prior;

    prior = new isam::Pose3d_Factor(pose_node,vo,noise);

    slam.add_factor(prior);
    priors.push_back(prior);

}
void KeyframeGraphSolverISAM::add_landmark_from_file(int from_idx,int to_idx,double x, double y, double z,isam::Noise noise3)
{
    // this is only from file because we dont load landmark_observations here
    isam::Point3d measure1(x,y,z);
    isam::Point3d c_landmark1(measure1);
    isam::Pose3d_Point3d_Factor* factor1 = new isam::Pose3d_Point3d_Factor(poses[from_idx],landmarks_explicit[to_idx],c_landmark1,noise3);
    landmark_observations.push_back(factor1);
    slam.add_factor(factor1);
}

void KeyframeGraphSolverISAM::add_landmark(KeyframeVector& keyframes,KeyframeAssociation association, int from_idx,int to_idx,int j, isam::Noise noise3)
{
    const RGBDKeyframe keyframe1 = keyframes[from_idx];
    const RGBDKeyframe keyframe2 = keyframes[to_idx];
    const cv::DMatch match = association.matches[j];
    Observation land1, land2;
    land1.first = from_idx;
    land2.first = to_idx;
    land1.second = match.trainIdx; // these ones are switched on purpose (because on graph detector it a = train, b = query)
    land2.second = match.queryIdx; // these ones are switched on purpose (because on graph detector it a = train, b = query)

    assert(keyframe1.kp_valid[land1.second] == true && keyframe2.kp_valid[land2.second] == true);

    //we have to check if it exists already
    if(landmarks.find(land1)==landmarks.end() && landmarks.find(land2)==landmarks.end())//not added yet
    {
        isam::Point3d_Node* landmark = new isam::Point3d_Node();
        slam.add_node(landmark);
        landmarks_explicit.push_back(landmark);
        landmarks[land1] = landmark;
        landmarks[land2] = landmark;

    }
    else if (landmarks.find(land1)!=landmarks.end() && landmarks.find(land2)==landmarks.end())//seen in keyframe1
    {
        landmarks[land2] = landmarks[land1];
    }
    else if(landmarks.find(land1)==landmarks.end() && landmarks.find(land2)!=landmarks.end()) //seen in keyframe2
    {
        landmarks[land1] = landmarks[land2];
    }
    isam::Point3d measure1(keyframe1.kp_means[land1.second][0],keyframe1.kp_means[land1.second][1],keyframe1.kp_means[land1.second][2]);
    isam::Point3d c_landmark1(measure1);
    isam::Pose3d_Point3d_Factor* factor1 = new isam::Pose3d_Point3d_Factor(poses[land1.first],landmarks[land1],c_landmark1,noise3);
    slam.add_factor(factor1);
    landmark_observations.push_back(factor1);
    isam::Point3d measure2(keyframe2.kp_means[land2.second][0],keyframe2.kp_means[land2.second][1],keyframe2.kp_means[land2.second][2]);
    isam::Point3d c_landmark2(measure2);
    isam::Pose3d_Point3d_Factor* factor2 = new isam::Pose3d_Point3d_Factor(poses[land2.first],landmarks[land2],c_landmark2,noise3);
    slam.add_factor(factor2);
    landmark_observations.push_back(factor2);
}

void KeyframeGraphSolverISAM::add_aruco(aruco::Marker marker, isam::Pose3d_Node* from, isam::Noise noise3)
{
    isam::Point3d_Node* aruco = new isam::Point3d_Node();
    slam.add_node(aruco);
    marker_poses.push_back(aruco);
    isam::Point3d measure1(marker.Tvec.ptr<float>(0)[0],marker.Tvec.ptr<float>(0)[1],marker.Tvec.ptr<float>(0)[2]);
    isam::Point3d aruco_measure(measure1);
    isam::Pose3d_Point3d_Factor* factor = new isam::Pose3d_Point3d_Factor(from,aruco,aruco_measure,noise3);
    slam.add_factor(factor);
    marker_observations.push_back(factor);
}

void KeyframeGraphSolverISAM::solve(
        KeyframeVector& keyframes,
        const KeyframeAssociationVector& associations)
{
    isam::Noise noise3 = isam::Information(1. * isam::eye(3));
    isam::Noise noise6 = isam::Information(10. * isam::eye(6));
    bool train = false;

    if(keyframes.size() > 0 && train == true)
    {
        train = false;
        std::cout << "Can't use training data because there are already keyframes loaded" << std::endl;
    }
    KeyframeAssociationVector local_associations = associations;

    //Sample data
    if(train)
    {
        RGBDKeyframe sample;
        sample.pose = AffineFromTRPY(0,0,0,0,0,0);

        sample.kp_valid.push_back(true);
        sample.kp_valid.push_back(true);
        sample.kp_valid.push_back(true);
        sample.kp_means.push_back(Eigen::Vector3f(1,0,0));
        sample.kp_means.push_back(Eigen::Vector3f(0,1,0));
        sample.kp_means.push_back(Eigen::Vector3f(0,0,1));
        RGBDKeyframe sample2;
        sample2.pose = AffineFromTRPY(1,1,1,0,0,M_PI/2);
        sample2.kp_valid.push_back(true);
        sample2.kp_valid.push_back(true);
        sample2.kp_valid.push_back(true);
        sample2.kp_means.push_back(sample.pose*sample2.pose.inverse()*sample.kp_means[0]);
        sample2.kp_means.push_back(sample.pose*sample2.pose.inverse()*sample.kp_means[1]);
        sample2.kp_means.push_back(sample.pose*sample2.pose.inverse()*sample.kp_means[2]);
        keyframes.push_back(sample);
        keyframes.push_back(sample2);
        KeyframeAssociation a;
        a.a2b = sample.pose.inverse()*sample2.pose;
        a.kf_idx_a=0;
        a.kf_idx_b=1;
        cv::DMatch b0;
        b0.distance =0;
        b0.imgIdx = 0;
        b0.queryIdx = 0;
        b0.trainIdx = 0;
        cv::DMatch c0;
        c0.distance =0;
        c0.imgIdx = 0;
        c0.queryIdx = 1;
        c0.trainIdx = 1;
        cv::DMatch d0;
        d0.distance =0;
        d0.imgIdx = 0;
        d0.queryIdx = 2;
        d0.trainIdx = 2;
        a.matches.push_back(b0);
        a.matches.push_back(c0);
        a.matches.push_back(d0);
        local_associations.push_back(a);
    }

    if (!train)
        local_associations = associations;
    // add vertices
    n_original_poses = keyframes.size();
    std::cout << "Adding vertices..." << std::flush;
    for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
    {
        Pose pose = keyframes[kf_idx].pose;
        if (kf_idx == 0)
        {
            isam::Noise noise_prior = isam::Information(99999. * isam::eye(6));
            add_pose(pose,noise_prior);

        }
        else
        {
            add_pose(pose,noise6);
        }
    }
    std::cout << " done." << std::endl;

    // add visual odometry
    std::cout << "Adding Visual Odometry..." << std::flush;
    for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
    {
        Pose pose;
        if (kf_idx != 0)
        {
            pose = keyframes[kf_idx-1].pose.inverse()*keyframes[kf_idx].pose;
            add_edge(poses[kf_idx-1],poses[kf_idx],pose,noise6);

        }
    }
    std::cout << " done." << std::endl;
    std::cout << "Adding RANSAC vertices..." << std::flush;

    for (unsigned int as_idx = 0; as_idx < local_associations.size(); ++as_idx)
    {
        const KeyframeAssociation& association = local_associations[as_idx];
        if (!(association.type == KeyframeAssociation::RANSAC))
            continue;
        int from_idx = association.kf_idx_a;
        int to_idx   = association.kf_idx_b;

        add_edge(poses[from_idx], poses[to_idx], association.a2b, noise6);
    }


    std::cout << " done." << std::endl;

    std::cout << "Adding Landmark Observations...     " << std::flush;

    for (size_t i = 0; i < local_associations.size(); ++i) {
        //        std::cout << '\b'<<'\b'<<'\b'<<'\b' << std::flush;
        //        std::cout << boost::format("%.2f") % ((float)(i)/local_associations.size()) << std::flush;
        const KeyframeAssociation& association = local_associations[i];
        bool cont = false;
        int from_idx = association.kf_idx_a;
        int to_idx   = association.kf_idx_b;
        if (association.type == KeyframeAssociation::RANSAC && association.matches.size() < 30) //we already have a good estimate with ransac
            cont = true;
        if (association.type == KeyframeAssociation::LANDMARKS || cont)
            for (u_int j = 0; j < association.matches.size(); j++)
            {
                add_landmark(keyframes,association,from_idx,to_idx,j,noise3);
            }
    }
    std::cout << " done." << std::endl;

//    std::cout << "Searching for ArUco markers..." << std::endl;
//    assert(markers_.size() == keyframes.size());
//    for(u_int i = 0; i< markers_.size(); i++)
//    {
//        const std::vector<aruco::Marker> marker_vec;
//        for (u_int j = 0; j < marker_vec.size(); j++)
//        {
//            add_aruco(marker_vec[j],i,noise3);
//        }
//    }

    std::cout << "Graph has " << slam.num_nodes() << " nodes and " << slam.num_factors() << " factors." << std::endl;

    slam.save("~/before.txt");
    // run the optimization
    printf("Optimizing...\n");
    optimizeGraph();

    // update the keyframe poses
    printf("Updating keyframe poses...\n");

    AffineTransformVector optimized_poses;
    optimized_poses.resize(keyframes.size());
    getOptimizedPoses(optimized_poses);
    slam.save("~/after.txt");
    for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
    {
        RGBDKeyframe& keyframe = keyframes[kf_idx];
        keyframe.pose = optimized_poses[kf_idx];
    }
    PointCloudT c;
    concatenate_clouds(c,keyframes);
}

void KeyframeGraphSolverISAM::solve(
        KeyframeVector& keyframes,
        const KeyframeAssociationVector& associations,
        AffineTransformVector& path)
{
    return;
}
void KeyframeGraphSolverISAM::solve_Iterative(
        KeyframeVector& keyframes)
{
    slam.update();
    AffineTransformVector optimized_poses;
    optimized_poses.resize(keyframes.size());

    getOptimizedPoses(optimized_poses);
    for (unsigned int kf_idx = n_original_poses; kf_idx < keyframes.size(); ++kf_idx)
    {
        RGBDKeyframe& keyframe = keyframes[kf_idx];
        keyframe.pose = optimized_poses[kf_idx];
        isam::Pose3d_Node* pose = poses[kf_idx];
        bool hit = false;

        for (std::list<isam::Factor*>::const_iterator it=pose->factors().begin(); it != pose->factors().end(); ++it)
        {
            if(dynamic_cast<isam::Pose3d_Pose3d_Factor*>(*it) == *it)
                hit = true;
        }
        if(hit) //robustness
            keyframe.used = true;
    }
//    for (u_int i = 0; i<marker_poses.size(); i++)
//    {
//        if(!valid_markers[i])
//            continue;
//        const isam::Point3d_Node* pose = marker_poses[i];
//        std::cout << pose->value().x() << " " <<pose->value().y() << " " << pose->value().z() << std::endl;
//    }

}
void KeyframeGraphSolverISAM::solve_rgb_Iterative(
        KeyframeVector& keyframes)
{
    slam.update();
    AffineTransformVector optimized_poses;
    optimized_poses.resize(keyframes.size());

    getOptimizedPoses(optimized_poses);
    for (unsigned int kf_idx = n_original_poses; kf_idx < keyframes.size(); ++kf_idx)
    {
        RGBDKeyframe& keyframe = keyframes[kf_idx];
        keyframe.pose = optimized_poses[kf_idx];
        isam::Pose3d_Node* pose = poses[kf_idx];
        if(pose->factors().size() > 1)
            keyframe.used = true;
    }

}
void KeyframeGraphSolverISAM::getArucoPos(std::vector<std::vector<double> >* positions,std::vector<int> *marker_kfs,KeyframeVector& keyframes)
{

    positions->clear();
    for (u_int i = 0; i<marker_poses.size(); i++)
    {
        if(!valid_markers[i])
            continue;
        isam::Point3d_Node* pose = marker_poses[i];
        std::vector<double> values;
        values.push_back(pose->value().x());
        values.push_back(pose->value().y());
        values.push_back(pose->value().z());
        positions->push_back(values);
    }
    for(u_int i = 0; i< marker_kf.size(); i++)
    {
        marker_kfs->push_back(marker_kf[i]);
    }
}

void KeyframeGraphSolverISAM::optimizeGraph()
{
    //  //Prepare and run the optimization
    slam.batch_optimization();
}

void KeyframeGraphSolverISAM::getOptimizedPoses(AffineTransformVector& poses_)
{
    for (unsigned int idx = 0; idx < poses_.size(); ++idx)
    {
        isam::Pose3d_Node* pose = poses[idx];
        Pose optimized_pose = AffineFromTRPY(pose->value().x(),pose->value().y(),pose->value().z(),pose->value().roll(),pose->value().pitch(),pose->value().yaw());
        //    //Set the optimized pose to the vector of poses

        poses_[idx] = optimized_pose;
    }
}

} // namespace ccny_rgbd


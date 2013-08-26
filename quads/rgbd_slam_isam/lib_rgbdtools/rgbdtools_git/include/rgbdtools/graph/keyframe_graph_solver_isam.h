/**
 *  @file keyframe_graph_solver.h
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

#ifndef RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_ISAM_H
#define RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_ISAM_H

#include "rgbdtools/rgbd_keyframe.h"
#include "rgbdtools/graph/keyframe_association.h"
#include "rgbdtools/graph/keyframe_graph_solver.h"
#include "isam/isam.h"
#include "aruco/aruco.h"

namespace rgbdtools {


class KeyframeGraphSolverISAM: public KeyframeGraphSolver
{

  public:

    /** @brief Constructor
     */
    KeyframeGraphSolverISAM();

    /** @brief Default destructor
     */
    ~KeyframeGraphSolverISAM();

     void solve(
      KeyframeVector& keyframes,
      const KeyframeAssociationVector& associations);
     void solve_Iterative(KeyframeVector& keyframes);
     void solve_rgb_Iterative(KeyframeVector& keyframes);

     void solve(
      KeyframeVector& keyframes,
      const KeyframeAssociationVector& associations,
      AffineTransformVector& path);
     void save_graph(std::string filepath);
     void giveMarkers(std::vector<aruco::Marker> markers);
     void save_landmarks(std::string filepath);

     void parse (std::string fname,rgbdtools::KeyframeVector keyframes);
     void parse_line(char* str);
     void load_landmarks(std::string filepath);
     u_int getNOriginalPoses(){ return n_original_poses;}

     typedef  std::pair<int,int> Observation;

     u_int n_original_poses;
     std::map<Observation,isam::Point3d_Node*> landmarks;
     std::vector<isam::Point3d_Node*> landmarks_explicit;
     std::vector<isam::Point3d_Node*> marker_poses;
     std::vector<int> marker_kf;
     std::vector<isam::Pose3d_Point3d_Factor*> marker_observations;
     std::vector<aruco::Marker> markers_;
     std::vector<isam::Pose3d_Node*> poses;
     std::vector<bool> valid_poses;
     std::vector<bool> valid_markers;
     std::vector<isam::Pose3d_Pose3d_Factor*> vodom;
     std::vector<isam::Factor*> priors;
     std::vector<isam::Pose3d_Point3d_Factor*> landmark_observations;
     isam::Slam slam ;
     void optimizeGraph();     
     void getOptimizedPoses(AffineTransformVector& poses_);
     void add_vo(isam::Pose3d_Node *from, isam::Pose3d_Node *to, AffineTransform pose, isam::Noise noise); 
     void add_prior(isam::Pose3d_Node* pose_node, double x, double y, double z, double roll, double pitch, double yaw,isam::Noise noise);
     void add_pose(Pose pose,isam::Noise noise);
     void add_pose(double x, double y, double z, double roll, double pitch, double yaw,isam::Noise noise);
     void add_edge(isam::Pose3d_Node* from, isam::Pose3d_Node* to,double x, double y, double z, double roll, double pitch, double yaw, isam::Noise noise);
     void add_edge(isam::Pose3d_Node* from, isam::Pose3d_Node* to, const AffineTransform& pose, isam::Noise noise);
     void add_edges(u_int kf_idx,KeyframeVector& keyframes,
                                             const KeyframeAssociationVector& associations);
     void add_landmark(KeyframeVector& keyframes,KeyframeAssociation association, int from_idx,int to_idx,int j, isam::Noise noise3);
     void add_landmark_from_file(int from_idx,int to_idx,double x, double y, double z,isam::Noise noise3);
     void add_aruco(aruco::Marker marker, isam::Pose3d_Node* from, isam::Noise noise3);
     void getArucoPos(std::vector<std::vector<double> >* positions,std::vector<int>* marker_kfs,KeyframeVector& keyframes);

private:
     void parse_factors();
     void parse_nodes();

};

} // namespace rgbdtools

#endif // RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_ISAM_H

/*
 *  UtilitiesBlock.h
 *  
 *  Created by Pedro Vieira on 10/27/11.
 *  
 */

#ifndef UTILITIESBLOCK_H
#define UTILITIESBLOCK_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <Eigen/Dense>


typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ> Cloudxyz;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCloudxyz;


//make txt backup
void SaveCloudToTXT(const char* filename,Cloud cloud);
void SaveCloudToTXT(const char* filename,PCloud pcloud);

//load txt backup
void LoadCloudFromTXT(const char* filename, Cloud &cloud);
void LoadCloudFromTXT(const char* filename, PCloud &pcloud);
void LoadCloudFromTXT(const char* filename, PCloudxyz &pcloud);
void loadCloudFromPLY(const char* filename, PCloud &pcloud);

//make ply file
void PrintCloudToPLY(const char* filename, Cloud cloud, bool allpoints=false);
void PrintCloudToPLY(const char* filename, PCloud pcloud, bool allpoints=false);
void PrintCloudToPLY(const char* filename, PCloud pcloud, std::vector<int> indexs);
void PrintCloudToPLY(const char* filename, PCloud pcloud, Eigen::Matrix4d T, bool allpoints=false);
void PrintCloudToPLY(const char* filename, PCloud pcloud, std::vector<Eigen::Vector3d> Normals );

void WriteCameraPLY(const char* filename, std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_position);

void SaveKinectPose(const char* filename, double x,double y,double z,double ax, double ay, double az,double q0=0.0, double q1=0.0, double q2=0.0, double q3=0.0);

void saveDepthMatrix(const char* filename,PCloud pcloud);
void saveDepthImagePNG(std::string filename, PCloud pcloud);

void saveRGBimageJPG(std::string filename, PCloud pcloud);
void saveRGBimagePNG(std::string filename, PCloud pcloud);

#endif

/*
 *  Dataset_analyser.h   
 *  Created by Pedro Vieira on 10/28/11.
 *  
 */

#include <iostream> 
#include <string>
#include <Eigen/Dense>

 
#ifndef DATASET_ANALYZER_H
#define DATASET_ANALYZER_H

class Dataset_analyzer{
	
	Eigen::Vector3d ts;
	Eigen::Quaterniond qs;
	
	Eigen::Vector3d td;
	Eigen::Quaterniond qd;
	
	Eigen::Quaterniond qp; //permutation matrix 
		
public:	

	Dataset_analyzer(std::string file_pose_s, std::string file_pose_d);
	
	Eigen::Matrix4d Computetransformation();
	
	Eigen::Quaterniond returnRotquat();
	
	void returnAngles_rad (double &ax,double &ay,double &az);
	void returnAngles_deg (double &ax,double &ay,double &az);
	Eigen::Vector3d returnTran();
	Eigen::Matrix4d returnTr();
 	
	//debug
	void testetransformation(std::string file_pose_s, std::string file_pose_d, std::string ply_filename_s,std::string ply_filename_d);													

	
private:
	void ReadFile(std::string filename, Eigen::Vector3d &t, double &qw, double &qx,double &qy,double &qz);	

};




#endif

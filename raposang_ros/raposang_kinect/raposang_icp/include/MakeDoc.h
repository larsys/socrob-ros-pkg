/*
 *  MakeDoc.h
 *  
 *  Created by Pedro Vieira on 1/4/12.
 *  
 */

#include <fstream> 
#include <string>
#include <Eigen/Dense>

class Doc{
	
	std::ofstream txt_file;
	
	std::string filepath; 
	std::string filename;
	
	Eigen::Matrix4d T_icp;
	Eigen::Matrix4d T_truth;
	Eigen::Matrix4d T_regua;
	
	Eigen::Matrix3d trans_cov;
	
	double icp_time;
	double icp_iter_time;
	
	int iter_number;	
		

public:
	Doc(std::string fph , std::string fn){
		filepath=fph; 
		filename=fn;
		
		T_icp.setIdentity();
		T_truth.setIdentity();
		T_regua.setIdentity();
		trans_cov.setIdentity();
		
		icp_time= 999.0;
		icp_iter_time = 999.0;
		iter_number = 999;	
	}
	
	void makeDoc();
	
	inline void set_filepath(std::string fph){filepath = fph;}	
	inline void set_filename(std::string fn){filename = fn;}
	inline void set_T_icp(Eigen::Matrix4d T){T_icp = T;}
	inline void set_T_truth(Eigen::Matrix4d T){T_truth = T;}			
	inline void set_T_regua(Eigen::Matrix4d T){T_regua = T;}
	inline void set_trans_cov(Eigen::Matrix3d T){trans_cov = T;}
	inline void set_icp_time(double it){icp_time = it;}
	inline void set_icp_iter_time(double it){icp_iter_time = it;}
	inline void set_iter_number(int in){iter_number = in;}
		
	double rotation_error(Eigen::Matrix3d R1, Eigen::Matrix3d R2);
	double translation_error(Eigen::Vector3d t1, Eigen::Vector3d t2);

};
 
 



#include "MakeDoc.h"
	

void Doc::makeDoc(){
	
	std::string filecname = filepath+filename; 
	
	txt_file.open (filecname.data());
	
	txt_file << "2 Frames Alignment\n";
	txt_file << "\n";
	
	txt_file << "Total time: " << icp_time << " seconds" << "\n";
	txt_file << "Time per iteration: " << icp_iter_time << " seconds" << "\n";
	txt_file << "Number of iterations " << iter_number << "\n";
	
	txt_file << "\n";
	
	txt_file << "Rotation error (icp vs gtruth) " << rotation_error(T_icp.block<3,3>(0,0),T_truth.block<3,3>(0,0)) << " degrees" <<"\n";
	txt_file << "Translation error (icp vs gtruth) " << translation_error(T_icp.block<3,1>(0,3),T_truth.block<3,1>(0,3)) << " m"<< "\n";
	
	txt_file << "\n";
	
	txt_file << "Rotation error (icp vs regua) " << rotation_error(T_icp.block<3,3>(0,0),T_regua.block<3,3>(0,0)) << " degrees" << "\n";
	txt_file << "Translation error (icp vs regua) " << translation_error(T_icp.block<3,1>(0,3),T_regua.block<3,1>(0,3)) << " m" << "\n";	
	
	txt_file << "\n";
	
	txt_file << "Rotation error (gtruth vs regua) " << rotation_error(T_truth.block<3,3>(0,0),T_regua.block<3,3>(0,0)) << " degrees" << "\n";
	txt_file << "Translation error (gtruth vs regua) " << translation_error(T_truth.block<3,1>(0,3),T_regua.block<3,1>(0,3)) << " m" << "\n";	
	
	txt_file << "\n";

	txt_file << "Transformation matrix (ICP):\n" << T_icp << "\n";
	txt_file << "Transformation matrix (gtruth):\n" << T_truth << "\n"; 
	txt_file << "Transformation matrix (regua):\n" << T_regua << "\n"; 
	
	txt_file << "\n";
	
	txt_file << "Transformation Covariance:\n" << trans_cov << "\n"; 
	
	txt_file.close();
	
}


double Doc::rotation_error(Eigen::Matrix3d R1, Eigen::Matrix3d R2){
	Eigen::Quaterniond q1 (R1);
	Eigen::Quaterniond q2 (R2);
	
	
	return fabs(2*acos(q2.w()) - 2*acos(q1.w()) )*180.0/3.1415926535;
}

double Doc::translation_error(Eigen::Vector3d t1, Eigen::Vector3d t2){
	return fabs(t1.norm() - t2.norm());
}

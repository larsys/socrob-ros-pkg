
#include <fstream>
#include "DatasetAnalyzer.h"
#include "UtilitiesBlock.h"


Dataset_analyzer::Dataset_analyzer(std::string file_pose_s, std::string file_pose_d){
	
	double qw,qx,qy,qz;
	Eigen::Matrix3d P;
	P << 1.0, 0.0, 0.0 ,
		 0.0, 0.0, 1.0 ,
		 0.0,-1.0, 0.0 ;
	
	ReadFile(file_pose_s,ts,qw,qx,qy,qz);
	qs=Eigen::Quaterniond(qw,qx,qy,qz);
	
	ReadFile(file_pose_d,td,qw,qx,qy,qz);
	qd=Eigen::Quaterniond(qw,qx,qy,qz);
	
	qp=Eigen::Quaterniond(P);
	
}

Eigen::Matrix4d Dataset_analyzer::Computetransformation(){
	Eigen::Quaterniond qs_n = qs*qp; //change axis
	Eigen::Quaterniond qd_n = qd*qp; //change axis
	
	/*
	Eigen::Matrix4d Ts;	Ts.setIdentity();
	Eigen::Matrix4d Td; Td.setIdentity();
	
	Ts.block<3,3>(0,0) = qs_n.toRotationMatrix ();
	Ts.block<3,1>(0,3) = ts;
	
	Td.block<3,3>(0,0) = qd_n.toRotationMatrix ();
	Td.block<3,1>(0,3) = td;
		
	Eigen::Matrix4d T = Ts*Td.inverse();	
	*/

	Eigen::Quaterniond q12 = qd_n*qs_n.inverse(); //compute rotation s --> d
	
	Eigen::Matrix4d T; T.setIdentity();
	T.block<3,3>(0,0) = q12.toRotationMatrix ();
	T.block<3,1>(0,3) = td-ts;
	
	
	return T;
}

Eigen::Quaterniond Dataset_analyzer::returnRotquat(){
	Eigen::Matrix4d T = Computetransformation();
	Eigen::Quaterniond q(T.block<3,3>(0,0));
	return q;
}

void Dataset_analyzer::returnAngles_rad (double &ax,double &ay,double &az){
	Eigen::Matrix4d T = Computetransformation();
		
	ax = atan2(T(2,1), T(2,2)); //rx
	ay = asin(-T(2,0)); 		//ry
	az = atan2(T(1,0), T(0,0));	//rz
	
}
void Dataset_analyzer::returnAngles_deg (double &ax,double &ay,double &az){
	Eigen::Matrix4d T = Computetransformation();
	double PI=3.1415926535;
		
	ax = atan2(T(2,1), T(2,2)); //rx
	ay = asin(-T(2,0)); 		//ry
	az = atan2(T(1,0), T(0,0));	//rz
	
	ax = ax*180/PI;
	ay = ay*180/PI;
	az = az*180/PI;
}
Eigen::Vector3d Dataset_analyzer::returnTran(){
	Eigen::Matrix4d T = Computetransformation();
	return T.block<3,1>(0,3);
}
Eigen::Matrix4d Dataset_analyzer::returnTr(){
	Eigen::Matrix4d T = Computetransformation();
	return T;
}

//debug
void Dataset_analyzer::testetransformation(std::string file_pose_s, std::string file_pose_d, std::string ply_filename_s,std::string ply_filename_d){
	PCloud cloud_s;	
	PCloud cloud_d;	
	
	LoadCloudFromTXT(file_pose_s.data(), cloud_s);
	LoadCloudFromTXT(file_pose_d.data(), cloud_d);
	
	Eigen::Matrix4d Ts; Ts.setIdentity();
	Ts.block<3,3>(0,0) = (qs*qp).toRotationMatrix ();
	Ts.block<3,1>(0,3) = ts;
	
	Eigen::Matrix4d T = Computetransformation();
	//T.setIdentity();
	//T.block<3,3>(0,0) = (qs*qp).toRotationMatrix()*(qd*qp).toRotationMatrix().transpose();
	//T.block<3,1>(0,3) = td-ts;
	
	Eigen::Matrix4d Taux; Taux.setIdentity(); 
	Taux.block<3,3>(0,0) = T.block<3,3>(0,0)*Ts.block<3,3>(0,0);
	Taux.block<3,1>(0,3) = Ts.block<3,1>(0,3) + T.block<3,1>(0,3);
	
	PrintCloudToPLY(ply_filename_s.data(), cloud_s, Ts);
	PrintCloudToPLY(ply_filename_d.data(), cloud_d, Taux);
	
	//std::cout << T<< "\n";
	//std::cout << atan2(T(2,1), T(2,2)) << " " << asin(-T(2,0)) << " " << atan2(T(1,0), T(0,0)) << "\n";	
}		


void ReadFile(std::string filename, Eigen::Vector3d &t, double &qw, double &qx,double &qy,double &qz){
	
	t<<0.0,0.0,0.0;
	double dummy;
	
	double qw_aux,qx_aux,qy_aux,qz_aux;
	Eigen::Vector3d t_aux(0.0,0.0,0.0);
	qw=qx=qy=qz=0.0;
	
	int npoint=0;
	
	std::ifstream file(filename.data());
	
	while(!file.eof()){
		file >> dummy;
		file >> t_aux(0);
		file >> t_aux(1);
		file >> t_aux(2);
		file >> dummy; file >> dummy; file >> dummy;
		file >> qw_aux;
		file >> qx_aux;
		file >> qy_aux;
		file >> qz_aux;
		
		t+=t_aux;
		qw+=qw_aux;
		qx+=qx_aux;
		qy+=qy_aux;
		qz+=qz_aux;
			
		npoint++;
	}		
	file.close();
	
	qw=qw/npoint;
	qx=qx/npoint;
	qy=qy/npoint;
	qz=qz/npoint;
	t=t/npoint;
	
}	




/*
void Dataset_analyzer::Print_error_from_quaternions(Eigen::MatrixXd T){}
void Dataset_analyzer::Print_error_from_angles(Eigen::MatrixXd T){
	Eigen::Matrix4d T_s2d = ComputeTransformation();
	Eigen::Vector3d anlges_T = getAngles(T);
	Eigen::Vector3d anlges_Ts2d = getAngles(T_s2d);
	
	std::cout<< "Tranlation Error: " << (T.block<3,1>(0,3) - T_s2d.block<3,1>(0,3)).norm() << "\n";
			 
	std::cout<< "Angles Error: "
			 << fabs(anlges_T(0) - anlges_Ts2d(0)) << " "
			 << fabs(anlges_T(1) - anlges_Ts2d(1)) << " "
			 << fabs(anlges_T(2) - anlges_Ts2d(2)) << "\n";
			 
	std::cout<< "Angles Error2: " << (anlges_T - anlges_Ts2d).norm() << "\n";
}

Eigen::Matrix4d Dataset_analyzer::ComputeTransformation(){
	Eigen::Matrix4d T_s;
	Eigen::Matrix4d T_d;
	
	Eigen::Vector3d t_s(x_s, y_s, z_s);
	Eigen::Vector3d t_d(x_d, y_d, z_d); 
	Eigen::Quaterniond q_s (qw_s,qx_s,qy_s,qz_s);
	Eigen::Quaterniond q_d (qw_d,qx_d,qy_d,qz_d);
	
	T_s.setIdentity();
	T_d.setIdentity();
	
	T_s.block<3,3>(0,0) = q_s.toRotationMatrix();
	T_d.block<3,3>(0,0) = q_d.toRotationMatrix();	
	T_s.block<3,1>(0,3) = t_s;
	T_d.block<3,1>(0,3) = t_d;
	
	T_d = T_s*T_d.inverse();
	
	return T_d;	
}

Eigen::Vector3d Dataset_analyzer::getAngles(Eigen::Matrix4d T){
	Eigen::Vector3d angles;
	
	angles(0) = atan2(T(2,1), T(2,2)); 	//angles in "x"
	angles(1) = asin(-T(2,0));			//angles in "y"
	angles(2) = atan2(T(1,0), T(0,0));	//angles in "z"
	
	return angles;
}
*/

/*
#include "ros/ros.h"
int main(int argc, char *argv[]){
	
	ros::init(argc, argv, "dataset");
	ros::NodeHandle n;
	
	Eigen::MatrixXd T;
	Dataset_analyzer d("/home/pedrovieira/ros_workspace/kinect_data/bin/results/cloud_s_truth_pose.txt","/home/pedrovieira/ros_workspace/kinect_data/bin/results/cloud_d_truth_pose.txt");
	//d.print();
	d.correctmodelcloud("/home/pedrovieira/ros_workspace/kinect_data/bin/results/cloud_s_truth.txt","/home/pedrovieira/ros_workspace/kinect_data/bin/results/cloud_s_truth_new_ar_pose.ply" );
	d.testetransformation("/home/pedrovieira/ros_workspace/kinect_data/bin/results/cloud_d_truth.txt","/home/pedrovieira/ros_workspace/kinect_data/bin/results/cloud_d_truth_new_ar_pose.ply" );
}
*/

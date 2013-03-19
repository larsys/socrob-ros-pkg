/*
 *  ICP_plane2plane.hpp
 *  
 *  Created by Pedro Vieira on 11/10/11.
 *  
 */

//#include "ros/ros.h"
//#include "pcl/kdtree/organized_data.h"


inline void ICP_plane2plane::computeCovariance(const PCloud& pcloud, const std::vector<int> &cloud_indices, pcl::KdTreeFLANN< pcl::PointXYZRGB > *kdtree, std::vector<Eigen::Matrix3d> &cov_vec){

	std::vector<int> kk_indices;      kk_indices.reserve (k_correspondences);
	std::vector<float> kk_distances;  kk_distances.reserve (k_correspondences);

	Eigen::Vector3d mean;
	Eigen::Matrix3d U;
	
	int j;
	
	for(unsigned int i=0; i<cloud_indices.size(); i++){
		mean <<0.0,0.0,0.0;
		Eigen::MatrixXd points(3, k_correspondences);
		
		kdtree->nearestKSearch  (cloud_indices[i], k_correspondences, kk_indices, kk_distances); 
	
		for(j=0; j<kk_indices.size(); j++){
			points(0,j) = pcloud->points[kk_indices[j]].x;
			points(1,j) = pcloud->points[kk_indices[j]].y;
			points(2,j) = pcloud->points[kk_indices[j]].z;
			
			mean = mean +  points.block<3,1>(0,j);
		}
		
		if(j!=k_correspondences)
			points.conservativeResize(Eigen::NoChange, j);
		
		mean = mean/j;
			
			
		for (unsigned int k=0; k<j; k++)
			points.block<3,1>(0,k) = points.block<3,1>(0,k) - mean;
	
		cov_vec[i] = points * points.transpose();
	
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov_vec[i], Eigen::ComputeFullU);
		cov_vec[i].setZero ();
    
		U = svd.matrixU ();
		// Reconstitute the covariance matrix with modified singular values using the column     // vectors in V.
    
		cov_vec[i] =  U.col(0) * U.col(0).transpose() + 
					  U.col(1) * U.col(1).transpose() + gicp_epsilon * U.col(2) * U.col(2).transpose(); 
	}
}

inline int ICP_plane2plane::computeMahalanobisMats (){
		
	//Create Trees
	//pcl::OrganizedDataIndex< pcl::PointXYZRGB > kdtree_s;
	//pcl::OrganizedDataIndex< pcl::PointXYZRGB > kdtree_d;
	
	//int k=0;
				
	//mahalanobis.reserve(model_indices.size());
	cov_mod.reserve(model_indices.size());
	cov_dat.reserve(data_indices.size());			
	
	//Eigen::Matrix3d temp;
	//temp.setIdentity();
	
	computeCovariance(cloud_m, model_indices, &kdtree_m, cov_mod);
	computeCovariance(cloud_d, data_indices , &kdtree_d, cov_dat);
	
	/*		
	for(unsigned int i=0; i<model_indices.size();i++){						
		//temp = T.block<3,3>(0,0)*cov_dat[i];
		//temp = cov_mod[i] + temp * T.block<3,3>(0,0).transpose();
		//mahalanobis.push_back(temp.inverse());
		mahalanobis.push_back(temp);
	}	
	*/		
				
				
	//test_f << mahalanobis[i](0,0) << " " << mahalanobis[i](0,1) << " " << mahalanobis[i](0,2) << " "<< mahalanobis[i](1,0) << " "<< mahalanobis[i](1,1) << " "<< mahalanobis[i](1,2) << " "<< mahalanobis[i](2,0) << " "<< mahalanobis[i](2,1) << " "<< mahalanobis[i](2,2) << "\n";
							
	//model_indices[k]=model_indices[i];
	//data_indices[k]=data_indices[i];
	
	//if (k!=model_indices.size()){
	//	model_indices.resize(k);
	//	data_indices.resize(k);
	//}
	
	//return k;
	return 0;
} 


inline void ICP_plane2plane::minimize(const Pair& init_f){
	
	Eigen::VectorXd x(6);	 
	int m = model_indices.size () + init_f.size();
	
	// Translation estimates - initial guess
	x(0) = T(0,3); x(1) = T(1,3); x(2) = T(2,3);
	// Rotation estimates - initial guess quaternion: rx-ry-rz
	x(3) = atan2(T(2,1), T(2,2)); 	//rx
	x(4) = asin(-T(2,0)); 			//ry
	x(5) = atan2(T(1,0), T(0,0));	//rz
	
		
	// Set temporary pointers	
	tmp_idx_mod=&model_indices;
    tmp_idx_dat=&data_indices; 
	tmp_initf  =&init_f;
	 
	
	//~ lmdif_functor f(this,m);
	//~ NumericalDiff<lmdif_functor> numDiff(f);
	//~ LevenbergMarquardt<NumericalDiff<lmdif_functor> > lm(numDiff);
	//~ //lm.iter = 100;
	//~ lm.resetParameters();
	//~ //lm.parameters.ftol = 1.E6*NumTraits<double>::epsilon();
	//~ //lm.parameters.xtol = 1.E6*NumTraits<double>::epsilon();
	//~ lm.parameters.ftol = 1.E4*NumTraits<double>::epsilon();
    //~ lm.parameters.xtol = 1.E4*NumTraits<double>::epsilon();
	//~ //lm.parameters.maxfev = 1000;
	//~ lm.minimize(x);
 
		
	lmder_functor f(this,m);
	LevenbergMarquardt<lmder_functor> lm(f);
	lm.resetParameters();
	lm.parameters.ftol = 1.E4*NumTraits<double>::epsilon();
    lm.parameters.xtol = 1.E4*NumTraits<double>::epsilon();
	lm.lmder1(x);
	
	 
	// New transformation
	T.setIdentity();
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ())
	  * Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY())
	  * Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX()); 
	T.topLeftCorner<3,3> () = R;
	
	T(0,3) = x[0]; T(1,3) = x[1]; T(2,3) = x[2];
	
	tmp_idx_mod = tmp_idx_dat = NULL;
	tmp_initf = NULL;
	
}



/*
inline void ICP_plane2plane::TestTimeCov(){
	
	ros::Time start, end;
	std::vector<int> indices;
	
	for(int i=0; i<cloud_m->size(); i++){
		if(!isnan(cloud_m->points[i].z)){
			indices.push_back(i);
		}
	}	
	
	ROS_INFO("indices: %d", indices.size());
	
	start = ros::Time::now();
	cov_mod.reserve(indices.size());
	computeCovariance(cloud_m, indices, cov_mod);
	end = ros::Time::now();
	ROS_INFO("cov_time: %f", (end-start).toSec());

	start = ros::Time::now();
	pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree;
	kdtree.setInputCloud (cloud_m);
	end = ros::Time::now();
	ROS_INFO("tree_time: %f", (end-start).toSec());
	
	//~ start = ros::Time::now();
	//~ for(int i=0;i<indices.size();i++){
		//~ makeTree(cloud_m);
	//~ }
	//~ end = ros::Time::now();
	//~ ROS_INFO("tree_time2: %f", (end-start).toSec());
}
*/


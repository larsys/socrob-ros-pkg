/*
 *  ICPBlock.hpp
 *  
 *  Created by Pedro Vieira on 11/1/11.
 *  
 */

#include <algorithm> //for sort


inline void ICP::initvariables(){
	
	dmax_c=60.0;
	D = 0.05;
	Dmax = D*dmax_c;
	max_iterations = 100; 
	THRESH_dr =  0.5*3.14159265/180.0; //0.5º	 
	THRESH_dt = 0.002; //0.001   
	
	MAXPAIRS = 4000;
	MINPAIRS = 2000;
	
	T.setIdentity();
	
}


inline bool ICP::select_macth(){
	std::vector<int> indice(1);
	std::vector<float> distance(1);
	pcl::PointXYZRGB point_pcl;

	int r, k=0;
	
	for(int i=0 ; i<MAXPAIRS ; i++){
	
		r = rand() % cloud_d->size();
		
		if (isnan(cloud_d->points[r].z)) continue;
		if (ind[r]==1) continue;
		
		//apply RT
		point_pcl.x = cloud_d->points[r].x*T(0,0) + cloud_d->points[r].y*T(0,1) + cloud_d->points[r].z*T(0,2) + T(0,3);
		point_pcl.y = cloud_d->points[r].x*T(1,0) + cloud_d->points[r].y*T(1,1) + cloud_d->points[r].z*T(1,2) + T(1,3);
		point_pcl.z = cloud_d->points[r].x*T(2,0) + cloud_d->points[r].y*T(2,1) + cloud_d->points[r].z*T(2,2) + T(2,3);
		
			
		if ( kdtree_m.nearestKSearch (point_pcl, 1, indice, distance) != 1) continue;
		
		if (distance[0] < Dmax*Dmax){
			model_indices[k] = indice[0];
			data_indices[k] = r;
			k++;
			ind[r]=1;
		}	
		
		if ((i==MAXPAIRS-1) && (k<MINPAIRS)) i--;		
	}
	
	model_indices.resize(k);
	data_indices.resize(k);
	
	
	for(unsigned int i=0; i<k; i++)
		ind[data_indices[i]]=0;
	
	
	return true;
}

inline bool ICP::select_macth_useAllpoints(){
	std::vector<int> indice(1);
	std::vector<float> distance(1);
	pcl::PointXYZRGB point_pcl;

	int k=0;
	
	for(int i=0 ; i<cloud_d->size() ; i++){
	
		if (isnan(cloud_d->points[i].z)) continue;
		
		//apply RT
		point_pcl.x = cloud_d->points[i].x*T(0,0) + cloud_d->points[i].y*T(0,1) + cloud_d->points[i].z*T(0,2) + T(0,3);
		point_pcl.y = cloud_d->points[i].x*T(1,0) + cloud_d->points[i].y*T(1,1) + cloud_d->points[i].z*T(1,2) + T(1,3);
		point_pcl.z = cloud_d->points[i].x*T(2,0) + cloud_d->points[i].y*T(2,1) + cloud_d->points[i].z*T(2,2) + T(2,3);
		
			
		if ( kdtree_m.nearestKSearch (point_pcl, 1, indice, distance) != 1) continue;
		
		if (distance[0] < Dmax*Dmax){
			model_indices[k] = indice[0];
			data_indices[k] = i;
			k++;
		}	
	}
	
	model_indices.resize(k);
	data_indices.resize(k);
	
	return true;
}



inline void ICP::UpdateAndReject(Pair& init_f){
	
	double sigma=0.0;
	double mean=0.0;
	unsigned int N = data_indices.size() + init_f.size();
	
	Eigen::Vector4d point_s(0.0,0.0,0.0,1.0);
	Eigen::Vector4d point_d(0.0,0.0,0.0,1.0);
	
	std::vector<double> dists(N);
		
	//compute mean
	unsigned int k=0;
	for (unsigned int i=0 ; i < N; i++){
		
		if(i<data_indices.size()){
			point_s(0) = cloud_m->points[ model_indices[i] ].x;	
			point_s(1) = cloud_m->points[ model_indices[i] ].y;
			point_s(2) = cloud_m->points[ model_indices[i] ].z;
		
			point_d(0) = cloud_d->points[ data_indices[i] ].x;	
			point_d(1) = cloud_d->points[ data_indices[i] ].y;
			point_d(2) = cloud_d->points[ data_indices[i] ].z;
		}else{
			point_s(0) = cloud_m->points[ init_f[k].first ].x;	
			point_s(1) = cloud_m->points[ init_f[k].first ].y;
			point_s(2) = cloud_m->points[ init_f[k].first ].z;
		
			point_d(0) = cloud_d->points[ init_f[k].second ].x;	
			point_d(1) = cloud_d->points[ init_f[k].second ].y;
			point_d(2) = cloud_d->points[ init_f[k].second ].z;
			k++;
		}
		
		point_d = T*point_d;				
		
		dists[i]= (point_d - point_s).norm();
		mean = mean + dists[i];
	}
	
	mean = mean/N;
	
	//compute standart diviation
	for (unsigned int i=0; i < N; i++){
		sigma = sigma + (dists[i]-mean)*(dists[i]-mean);
	}
	
	sigma = sigma/N;
	sigma = sqrt(sigma);
	
	
	//How good is the registration
	if (mean<D)						//very good
		Dmax = mean + 3*sigma;
	else if (mean<3*D)				//good
		Dmax = mean + 2*sigma;
	else if (mean<6*D)				//bad
		Dmax = mean + sigma;
	else {							//very bad 
		std::vector<double> dists2 = dists;
		sort (dists2.begin(), dists2.end());
	
		if (dists2.size() % 2 == 0) {
			Dmax = (dists2[dists2.size()/2-1] + dists2[dists2.size()/2]) / 2.0;
		}else {
			Dmax = dists2[dists2.size()/2]; 
		}
	}
	
	//Update the maching
	k=0;
	unsigned int i=0;
	for (i=0 ; i <data_indices.size() ; i++){
		if (dists[i] < Dmax){
			model_indices[k] = model_indices[i];
			data_indices[k] = data_indices[i];
			k++;	
		}
	}
	
	model_indices.resize(k);
	data_indices.resize(k);	
		
	k=0;
	unsigned int j,l;
	for (j=i, l=0; j<N ; j++,l++){
		if (dists[j] < Dmax){
			init_f[k]= init_f[l];
			k++;	
		}
	}

	if(k!=init_f.size()) 
		init_f.resize(k);
	
}

//procustres
inline void ICP::minimize(const Pair& init_f){
	
	Eigen::Vector3d centroide_model(0.0,0.0,0.0), centroide_data(0.0,0.0,0.0);
	Eigen::Matrix3d M;
	
	unsigned int N = data_indices.size() + init_f.size();
	
	Eigen::MatrixXd model(3,N);
	Eigen::MatrixXd  data(3,N);
	
		
	//calcula os centroides
	int k=0;
	for(unsigned int i=0; i<N; i++){
		
		if (i<data_indices.size()){
			model(0,i) = cloud_m->points[ model_indices[i] ].x;
			model(1,i) = cloud_m->points[ model_indices[i] ].y;
			model(2,i) = cloud_m->points[ model_indices[i] ].z;
		
			data(0,i) = cloud_d->points[ data_indices[i] ].x*T(0,0) + cloud_d->points[ data_indices[i] ].y*T(0,1) + cloud_d->points[ data_indices[i] ].z*T(0,2) + T(0,3);
			data(1,i) = cloud_d->points[ data_indices[i] ].x*T(1,0) + cloud_d->points[ data_indices[i] ].y*T(1,1) + cloud_d->points[ data_indices[i] ].z*T(1,2) + T(1,3);
			data(2,i) = cloud_d->points[ data_indices[i] ].x*T(2,0) + cloud_d->points[ data_indices[i] ].y*T(2,1) + cloud_d->points[ data_indices[i] ].z*T(2,2) + T(2,3);
		}else{

			model(0,i) = cloud_m->points[ init_f[k].first ].x;
			model(1,i) = cloud_m->points[ init_f[k].first ].y;
			model(2,i) = cloud_m->points[ init_f[k].first ].z;
		
			data(0,i) = cloud_d->points[ init_f[k].second ].x*T(0,0) + cloud_d->points[ init_f[k].second ].y*T(0,1) + cloud_d->points[ init_f[k].second ].z*T(0,2) + T(0,3);
			data(1,i) = cloud_d->points[ init_f[k].second ].x*T(1,0) + cloud_d->points[ init_f[k].second ].y*T(1,1) + cloud_d->points[ init_f[k].second ].z*T(1,2) + T(1,3);
			data(2,i) = cloud_d->points[ init_f[k].second ].x*T(2,0) + cloud_d->points[ init_f[k].second ].y*T(2,1) + cloud_d->points[ init_f[k].second ].z*T(2,2) + T(2,3);	
			k++;
		}
		
		centroide_model += model.block(0,i,3,1);
		centroide_data  +=  data.block(0,i,3,1);
	}
	
	centroide_data = centroide_data/N;
	centroide_model = centroide_model/N;
	
	
  //subtrai os centroides aos dados
	for (unsigned int i=0; i<N; i++){
		model.block(0,i,3,1) -= centroide_model;
		data.block(0,i,3,1) -= centroide_data;
	}
	
	
	//Determina a transformacao
	M = data*model.transpose();
	
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	
	if (U.determinant()*V.determinant()<0) {
		for (int i=0; i<3; ++i) 
			V(i,2) *=-1;
	}
	
		
	Eigen::Matrix3d r = V * U.transpose();
	Eigen::Vector3d t = centroide_model - r * centroide_data;
	
	//~ T.block<3,3>(0,0) =  r*T.block<3,3>(0,0);
	//~ T.block<3,1>(0,3) += t; 
	T.block<3,1>(0,3) = T.block<3,3>(0,0)*t + T.block<3,1>(0,3); 
	T.block<3,3>(0,0) = T.block<3,3>(0,0)*r;
	
	
} 



inline bool ICP::error(Eigen::VectorXd xopt_ant , Eigen::VectorXd xopt){
	
	double dr;
	double q0;
	
	Eigen::Vector3d dt;
	dt(0) = xopt(4) - xopt_ant(4);
	dt(1) = xopt(5) - xopt_ant(5);
	dt(2) = xopt(6) - xopt_ant(6);
	
	if (xopt_ant(3)>1) {xopt_ant(3) = 1.0;}
	if (xopt_ant(3)<-1){xopt_ant(3) =-1.0;}
	if (xopt(3)>1)     {xopt(3)     = 1.0;}
	if (xopt(3)<-1)    {xopt(3)     =-1.0;}
	
	//q0 = xopt * xopt_ant.inverse()
	q0 = -(-xopt_ant(0))*xopt(0)
		 -(-xopt_ant(1))*xopt(1)
		 -(-xopt_ant(2))*xopt(2) 
		   +xopt_ant(3) *xopt(3);
	dr = 2 * acos(q0);
	
	//std::cout << fabs(dr) << " " << dt.norm() << "  " << THRESH_dr << "   " << THRESH_dt <<"      "<< (fabs(dr) <= THRESH_dr) << " "<< (dt.norm() <= THRESH_dt) <<  "\n";
	
	if ((fabs(dr) <= THRESH_dr) && ( dt.norm() <= THRESH_dt)) return true;

	return false;
	
}

inline void ICP::UpdateXopt(Eigen::VectorXd &xopt, const Eigen::Matrix4d& Tr){
	
	Eigen::Quaterniond q (Tr.block<3,3>(0,0));
 	
	xopt(0) = q.x();		//rx
	xopt(1) = q.y();		//ry
	xopt(2) = q.z();		//rz
	xopt(3) = q.w();		//teta
   	
	xopt(4) = Tr(0,3);	//tx
	xopt(5) = Tr(1,3);	//ty		
	xopt(6) = Tr(2,3);	//tz
	
}




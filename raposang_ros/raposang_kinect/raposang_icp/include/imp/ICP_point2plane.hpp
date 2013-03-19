//forma fechada
inline void ICP_p2plane::minimize(const Pair& init_f){

	Eigen::MatrixXd A(model_indices.size(),6);
	Eigen::VectorXd b(model_indices.size());
	Eigen::Vector3d normal;
	Eigen::Vector4d data_p;
	Eigen::VectorXd x(6);

	Eigen::Matrix4d T_op;
	T_op.setIdentity();
	
	for (unsigned int i=0; i<model_indices.size(); i++) {
		
		ComputeNormal(model_indices[i] , normal);
	
		data_p(0) = cloud_d->points[ data_indices[i] ].x;  
		data_p(1) = cloud_d->points[ data_indices[i] ].y;  
		data_p(2) = cloud_d->points[ data_indices[i] ].z;  
		data_p(3) = 1.0;
				
		data_p = T*data_p;
	
		
		b(i) = normal(0)*cloud_m->points[ model_indices[i] ].x + 
		       normal(1)*cloud_m->points[ model_indices[i] ].y + 
		       normal(2)*cloud_m->points[ model_indices[i] ].z -   normal(0)*data_p(0) - 
		                                                           normal(1)*data_p(1) - 
		                                                           normal(2)*data_p(2);	
		
		A(i,0) = normal(2)*data_p(1) - normal(1)*data_p(2);
		A(i,1) = normal(0)*data_p(2) - normal(2)*data_p(0);
		A(i,2) = normal(1)*data_p(0) - normal(0)*data_p(1);
		A(i,3) = normal(0);
		A(i,4) = normal(1);
		A(i,5) = normal(2);
		
	}
	
	//A.conservativeResize(k,Eigen::NoChange);
	//b.conservativeResize(k);
	
	//A é p*n
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	x =svd.solve(b);
	
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ())
	  * Eigen::AngleAxisd(x(1), Eigen::Vector3d::UnitY())
	  * Eigen::AngleAxisd(x(0), Eigen::Vector3d::UnitX()); 
	
	T_op.block<3,3>(0,0) = R;
	T_op.block<3,1>(0,3) = x.block(3,0,3,1);
	
	T = T*T_op;
	
} 


inline void ICP_p2plane::ComputeNormal(int pind, Eigen::Vector3d &normal){

	
	std::vector<int> kk_indices (k_correspondences);
	std::vector<float> kk_distances (k_correspondences);
	Eigen::MatrixXd points(3, k_correspondences);
	Eigen::Matrix3d cov;
	Eigen::Vector3d mean(0.0,0.0,0.0);
	
	
	kdtree_m.nearestKSearch (pind, k_correspondences, kk_indices, kk_distances); 
	
	
	for (unsigned int i=0; i< kk_indices.size();i++){

			points(0,i) = cloud_m->points[kk_indices[i]].x;
			points(1,i) = cloud_m->points[kk_indices[i]].y;
			points(2,i) = cloud_m->points[kk_indices[i]].z;
			
			mean = mean +  points.block<3,1>(0,i);	
	}
	mean = mean/k_correspondences;
	
	if(kk_indices.size()!=k_correspondences)
			points.conservativeResize(Eigen::NoChange, kk_indices.size());
	
	for (unsigned int i=0; i<kk_indices.size(); i++)
			points.block<3,1>(0,i) = points.block<3,1>(0,i) - mean;
	
	cov = points * points.transpose();
	
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
	
	//the eigenvalues are sorted in increasing order
	normal = eigensolver.eigenvectors().col(0);
	
	
	if (normal(0)*cloud_m->points[pind].x + normal(1)*cloud_m->points[pind].y + normal(2)*cloud_m->points[pind].z > 0) {
		normal(0) = -normal(0);
		normal(1) = -normal(1);
		normal(2) = -normal(2);
	}
}

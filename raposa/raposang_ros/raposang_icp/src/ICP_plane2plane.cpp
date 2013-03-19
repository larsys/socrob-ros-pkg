/*
 *  ICP_plane2plane.cpp
 *  
 *  Created by Pedro Vieira on 11/10/11.
 *  
 */


#include "ICP_plane2plane.h"


//public methods
bool ICP_plane2plane::align(Eigen::Matrix4d Tini, Pair init_f){
	
	srand((unsigned)time(NULL));
	
	Eigen::VectorXd xopt_ant(7);	//7 q(0)->x q(1)->y q(2)->z q(3)->w Tx Ty Tz 
	Eigen::VectorXd xopt(7);		//7 q(0)->x q(1)->y q(2)->z q(3)->w Tx Ty Tz

	T = Tini;

	//Create kdtree with cloud_s and cloud_d
	kdtree_m.setInputCloud (cloud_m);
	kdtree_d.setInputCloud (cloud_d);
	
	UpdateXopt(xopt,T);
	xopt_ant=xopt;
	
	
	//if (init_f.size()>=4){
	//	Dmax = 10*D;
	//}else{
		Dmax = 350*D;
	//}
	
	ind = std::vector<int> (cloud_d->size(),0);	
	
	iter=0;

	for(int i=0; i<max_iterations; i++){	
		iter++;
		std::cout<< "Iteration: " << iter << "\n"; 
		
		//mahalanobis.clear();
		cov_mod.clear();
		cov_dat.clear();
		
		if(cloud_d->size() > MAXPAIRS){
			model_indices = std::vector<int> (MAXPAIRS,0);
			data_indices = std::vector<int> (MAXPAIRS,0);
			select_macth();
		}else{
			model_indices = std::vector<int> (cloud_d->size(),0);
			data_indices = std::vector<int> (cloud_d->size(),0);
			select_macth_useAllpoints();
		}
		
		UpdateAndReject(init_f);
		
		computeMahalanobisMats(); 
		
		minimize(init_f);
		
		UpdateXopt(xopt,T);
		
		if(error(xopt_ant,xopt)) break;
		
		xopt_ant=xopt;
	}

	if(cloud_d->size() > MAXPAIRS) ind.clear();
	
	if(iter==max_iterations)
		return false;			//did not converge
	
	return true;				//converged
	
}




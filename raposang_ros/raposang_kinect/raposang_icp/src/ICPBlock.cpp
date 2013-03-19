/*
 *  ICPBlock.cpp
 *  
 *
 *  Created by Pedro Vieira on 10/24/11.
 * 
 * This file implements the ICP algorithm 
 *      - point 2 point
 *      - point 2 plane
 *      - plane 2 plane
 */

#include <iostream>
#include "ICPBlock.h"

////ICP class

//public methods
bool ICP::align(Eigen::Matrix4d Tini, Pair init_f){
	
	srand((unsigned)time(NULL));
	
	Eigen::VectorXd xopt_ant(7);	//7 q(0) q(1) q(2) q(3) Tx Ty Tz 
	Eigen::VectorXd xopt(7);		//7 q(0) q(1) q(2) q(3) Tx Ty Tz
	iter=0;
	
	T = Tini;
	
	//Create kdtree with cloud_m 
	kdtree_m.setInputCloud (cloud_m);
	
	
	UpdateXopt(xopt,T);
	xopt_ant=xopt;
	
	
	//if (init_f.size()>=4){
	//	Dmax = 3*D;
	//}else{
		Dmax = dmax_c*D;
	//}
	
	//ind.resize(cloud_s->size());
	//for (int i=0; i<ind.size(); i++) { //307200
	//	ind[i]=i;
	//}
	
	if(cloud_d->size() > MAXPAIRS)
		ind = std::vector<int> (cloud_d->size(),0);
	

	for(int i=0; i<max_iterations; i++){
		iter++;
		std::cout<< "Iteration: " << iter << "\n"; 
		
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






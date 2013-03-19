

class TestICP : public ICP{
	
public:
	TestICP(const char *filename1, const char *filename2){
		LoadCloudFromTXT(filename1, cloud_m);
		LoadCloudFromTXT(filename2, cloud_d);			
		initvariables();
	}

/*
	//use allpoints
	bool select_macth(std::vector<int> &source_indices, std::vector<int> &data_indices){
	std::vector<int> indice(1);
	std::vector<float> distance(1);
	source_indices.resize(cloud_m->size());
	data_indices.resize(cloud_d->size());

	Eigen::Vector4d point(0.0,0.0,0.0,1.0);
	pcl::PointXYZRGB point_pcl;

	int r;
	int k=0;
	for(unsigned int i=0 ; i<cloud_d->size() ; i++){
			
		if (isnan(cloud_d->points[i].z)) 
			continue;
		
		//apply RT
		point(0) = cloud_d->points[i].x;	
		point(1) = cloud_d->points[i].y;
		point(2) = cloud_d->points[i].z;
		point = T*point;
		point_pcl.x = point(0);
		point_pcl.y = point(1);
		point_pcl.z = point(2);
		
			
		if ( kdtree.nearestKSearch (point_pcl, 1, indice, distance) != 1) continue;
		
		if (distance[0] < Dmax){
			source_indices[k] = indice[0];
			data_indices[k] = i;
			k++;
		}	
	}
	
	source_indices.resize(k);
	data_indices.resize(k);
	
	return true;
}
*/

	//public methods
	inline void test_align(Eigen::Matrix4d Tini = Eigen::MatrixXd::Identity(4,4), Pair init_f=Pair(0)){
	
		srand((unsigned)time(NULL));
	
		Eigen::VectorXd xopt_ant(7);	//7 q(0) q(1) q(2) q(3) Tx Ty Tz 
		Eigen::VectorXd xopt(7);		//7 q(0) q(1) q(2) q(3) Tx Ty Tz

		T = Tini;
		
		//Create kdtree with cloud_m 
		kdtree_m.setInputCloud (cloud_m);
	
		UpdateXopt(xopt,T);
		xopt_ant=xopt;
	
		int nf = init_f.size();
		if (nf>=4){
			Dmax = 0.02;
			MAXPAIRS = 1000;
		}else{
			Dmax = 30*D;
			MAXPAIRS = 2000;
		}
	
		//ind.resize(cloud_m->size());
		//for (int i=0; i<ind.size(); i++) { //307200
		//	ind[i]=i;
		//}	
	
		int i;
		for(i=0; i<max_iterations; i++){
		
			std::vector<int> source_indices(MAXPAIRS);
			std::vector<int> data_indices(MAXPAIRS);
			//std::vector<int> source_indices(cloud_d->size());
			//std::vector<int> data_indices(cloud_d->size());
		
			select_macth(source_indices,data_indices);
		
			UpdateAndReject(source_indices,data_indices,init_f);
		
			minimize(source_indices,data_indices,init_f);
		
			UpdateXopt(xopt,T);
		
			if(error(xopt_ant,xopt)) break;
		
			xopt_ant=xopt;
		}
	
		std::cout<<"iterations: " << i << std::endl;
		
		//ind.clear();
	
	}
	
	inline void test_tree(){
		
		pcl::OrganizedDataIndex< pcl::PointXYZRGB > kdtree2;
		
		kdtree2.setInputCloud (cloud_m);
		
		std::vector<int> indice;
		std::vector<float> distance;

		//pcl::PointXYZRGB point_pcl;

		int r;
		for(unsigned int i=0 ; i<1000 ; i++){
		
			r = rand() % cloud_m->size();
	
			if (isnan(cloud_m->points[r].z)){ 
				continue;
			}
		
			//apply RT
			//point_pcl.x = cloud_m->points[r].x;
			//point_pcl.y = cloud_m->points[r].y;
			//point_pcl.z = cloud_m->points[r].z;
		
			//std::cout << radiusSearch (r, 0.05, indice, distance,20) << "\n";
			kdtree2.radiusSearch (r, 0.05, indice, distance,20);
			
		}
		
	}
	
	//ponsts disponiveis
	inline int test_cloud_mize(){
		
		int npoints=0;
		
		for(unsigned int i=0; i<cloud_m->size() ; i++){
			if (!isnan(cloud_m->points[i].z)) npoints++;
		}
		
		return npoints;	
	}

	PCloud returnNewcloud(){
		ApplyRTto(cloud_d);
		return cloud_d;
	}
	PCloud returncloudM(){return cloud_m;}
	PCloud returncloudD(){return cloud_d;}
	
private:	
	inline void ApplyRTto(PCloud &pc){
	
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		PCloud pcloud = PCloud(new Cloud());
	
		pcloud->header.stamp = pc->header.stamp;
		pcloud->header.frame_id = pc->header.frame_id;
		pcloud->is_dense = pc->is_dense;
	
		pcloud->width = pc->width;
		pcloud->height = pc->height;	
	
		pcloud->points.resize(pcloud->width*pcloud->height);		
		
		for(unsigned int v=0;v<pc->height;v++) {	 //480
			for(unsigned int u=0;u<pc->width;u++) {  //640
				if (!isnan((*pc)(u,v).z)){
					(*pcloud)(u,v).x = (*pc)(u,v).x*T(0,0) + (*pc)(u,v).y*T(0,1) + (*pc)(u,v).z*T(0,2) + T(0,3);
					(*pcloud)(u,v).y = (*pc)(u,v).x*T(1,0) + (*pc)(u,v).y*T(1,1) + (*pc)(u,v).z*T(1,2) + T(1,3);
					(*pcloud)(u,v).z = (*pc)(u,v).x*T(2,0) + (*pc)(u,v).y*T(2,1) + (*pc)(u,v).z*T(2,2) + T(2,3);	
				}else{
					(*pcloud)(u,v).x = bad_point;
					(*pcloud)(u,v).y = bad_point;
					(*pcloud)(u,v).z = bad_point;	
				}
				(*pcloud)(u,v).r = (*pc)(u,v).r;
				(*pcloud)(u,v).g = (*pc)(u,v).g;
				(*pcloud)(u,v).b = (*pc)(u,v).b;
			}
		}
	
		pc = pcloud;
	}
	
};


#include <cv.h>
#include "highgui.h"
#include <iostream>

class TestICPpl2pl : public ICP_plane2plane{

public:
	TestICPpl2pl (const char *filename1, const char *filename2) : ICP_plane2plane (){
		LoadCloudFromTXT(filename1, cloud_s);
		LoadCloudFromTXT(filename2, cloud_d);			
	}

/*	
	//use allpoints
	bool select_macth(std::vector<int> &source_indices, std::vector<int> &data_indices){
		std::vector<int> indice(1);
		std::vector<float> distance(1);

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
		
			if (distance[0] < Dmax*Dmax){
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
/*	
	int select_RGB_features(std::vector<int> &source_indices, std::vector<int> &data_indices, std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > inliers){
		
		int k = 0;
		double distance;
		
		for(int i=0; i<inliers.size() ; i++){
			distance = (inliers[i].first - inliers[i].second).norm();
			
			if (distance < Dmax){
				source_indices[k] = inliers[i].first;
				data_indices[k] = inliers[i].second;
				k++;
			}
		}
		return k;
	}

	bool select_macth(std::vector<int> &source_indices, std::vector<int> &data_indices, std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > inliers){
		std::vector<int> indice(1);
		std::vector<float> distance(1);

		Eigen::Vector4d point(0.0,0.0,0.0,1.0);
		pcl::PointXYZRGB point_pcl;


		int r;
		int k=0;
		for(int i=0 ; i<MAXPAIRS ; i++){
			
			if (i<inliers.size()){
				
				point.block<3,1>(0,0) = inliers[i].second;
				point = T*point;
				distance[0] = (inliers[i].first - point.block<3,1>(0,0)).norm
				distance[0] *= distance[0];
				indice[0] = 
				
			}else{
				r = rand() % cloud_d->size();
				if (isnan(cloud_d->points[r].z)) continue;
				
				//apply RT
				point(0) = cloud_d->points[r].x;	
				point(1) = cloud_d->points[r].y;
				point(2) = cloud_d->points[r].z;
				point = T*point;
				point_pcl.x = point(0);
				point_pcl.y = point(1);
				point_pcl.z = point(2);
		
				if ( kdtree.nearestKSearch (point_pcl, 1, indice, distance) != 1) continue;
			}
		
			if (distance[0] < Dmax*Dmax){
				source_indices[k] = indice[0];
				data_indices[k] = r;
				k++;
			}
			
			if ((i==MAXPAIRS-1) && (k<MINPAIRS)) i--;		
		}
	
		source_indices.resize(k);
		data_indices.resize(k);
	
		return true;
	}
*/	

/*
	void test_align_RGB(){
		
		//feito no exterior
		RGBFeaturesInit rgbf(cloud_s,cloud_d);
		rgbf.compute();
		
		std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > inliers;
		inliers = rgbf.getInliers();
		
		//////////////////
		
		
		srand(time(NULL));
	
		Eigen::VectorXd xopt_ant(7);	//7 q(0)->x q(1)->y q(2)->z q(3)->w Tx Ty Tz 
		Eigen::VectorXd xopt(7);		//7 q(0)->x q(1)->y q(2)->z q(3)->w Tx Ty Tz

		//Create kdtree with cloud_s 
		kdtree.setInputCloud (cloud_s);
	
		T = Tini;
	
		UpdateXopt(xopt,T);
		xopt_ant=xopt;
	

		for(int i=0; i<max_iterations; i++){	
			
			std::vector<int> source_indices(MAXPAIRS);
			std::vector<int> data_indices(MAXPAIRS);
			//std::vector<int> source_indices(cloud_s->size());
			//std::vector<int> data_indices(cloud_d->size());
			//mahalanobis.clear();
			cov_sor.clear();
			cov_dat.clear();
		
			select_macth(source_indices,data_indices);
			
			std::cout << "S= " << source_indices.size() << " D= " << data_indices.size() << std::endl;
			
			//DisplayMatches(source_indices,data_indices);
			
			computeMahalanobisMats (source_indices, data_indices); 
			std::cout << "\nS2= " << source_indices.size() << " D2= " << data_indices.size() << std::endl;
			
			minimize(source_indices,data_indices);
			std::cout << "\n" << T << std::endl;
		
			//UpdateAndReject(source_indices,data_indices);
		
			UpdateXopt(xopt,T);
		
			if(error(xopt_ant,xopt)) break;
			
			xopt_ant=xopt;
		}
		
	}
*/


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
	
	cv::Mat extractRGBfromCloud(const PCloud& cloud){
	
		cv::Mat RGBImage(cloud->height,cloud->width,CV_8UC3);
		
		for(unsigned int v=0;v<cloud->height;v++) {	    //480
			for(unsigned int u=0;u<cloud->width;u++) {  //640
			
				RGBImage.at<cv::Vec3b>(v,u)[0] = (int) (*cloud)(u,v).b; 
				RGBImage.at<cv::Vec3b>(v,u)[1] = (int) (*cloud)(u,v).g; 
				RGBImage.at<cv::Vec3b>(v,u)[2] = (int) (*cloud)(u,v).r; 	
			}
		}
	
		return RGBImage;
	}
	void featureshow(char* window_name, cv::Mat im, std::vector<cv::KeyPoint> kp, bool block){
		cv::Mat im_out;
		im_out=im;
	
		cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE); 
		cv::drawKeypoints(im, kp, im_out, cv::Scalar(0,0,255)); //BGR
	
		cv::imshow(window_name, im_out); 
	
		if(block)
			cv::waitKey();	
	}
	inline void DisplayMatches(std::vector<int> &source_indices, std::vector<int> &data_indices){
		
		cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Data", CV_WINDOW_AUTOSIZE);
		
		cv::Mat im_s = extractRGBfromCloud(cloud_s);
		cv::Mat im_d = extractRGBfromCloud(cloud_d);
		
		cv::imshow("Source", im_s);
		cv::imshow("Data", im_d);
		cv::waitKey(0);
		
		std::vector<cv::KeyPoint> kp_s(source_indices.size());
		std::vector<cv::KeyPoint> kp_d(data_indices.size());
		std::vector<cv::DMatch> matches(source_indices.size());
		
		for(int i=0;i<source_indices.size();i++){
			kp_s[i].pt.x = floor(source_indices[i]/640);
			kp_s[i].pt.y = source_indices[i] - kp_s[i].pt.x * 640;
			
			kp_d[i].pt.x= floor(data_indices[i]/640);
			kp_d[i].pt.y= data_indices[i] - kp_d[i].pt.x * 640;
			
		}
			
		featureshow("Source", im_s, kp_s, true);	
		featureshow("Data", im_d, kp_d, true);
		
		for(int i=0;i<source_indices.size();i++){
			matches[i].queryIdx = i;
			matches[i].trainIdx = i;
		}
		
		cv::Mat imageMatches;
		cv::drawMatches(im_s,kp_s,im_d,kp_d,matches,imageMatches,cv::Scalar(255,0,0));
		cv::imshow("matches",imageMatches);
		cv::waitKey(0);	
		
		
		
		
	}
	
};

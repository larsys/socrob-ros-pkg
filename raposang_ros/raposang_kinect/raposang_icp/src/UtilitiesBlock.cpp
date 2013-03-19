
#include "UtilitiesBlock.h"
#include <cv.h> 
#include "highgui.h"

#include "ros/ros.h"

#include <fstream>


//save cloud into a .txt file
void SaveCloudToTXT(const char* filename,Cloud cloud){
		
	std::ofstream txt_file(filename);
	
	txt_file << cloud.width << " " << cloud.height << "\n"; //x y
	
	for(int v=0;v<cloud.height;v++) {	  //480
		for(int u=0;u<cloud.width;u++) {  //640
			if (!isnan(cloud(u,v).z)){
				txt_file << cloud(u,v).x << " " << cloud(u,v).y << " " << cloud(u,v).z << " "
						 <<(int) cloud(u,v).r << " " << (int) cloud(u,v).g << " " << (int) cloud(u,v).b << "\n";
			}else{
				txt_file << 0.0 << " " << 0.0 << " " << 0.0 << " " 
						 <<(int) cloud(u,v).r << " " << (int) cloud(u,v).g << " " << (int) cloud(u,v).b << "\n";
			}
			
		}
	}
	
	
	txt_file.close();
}
void SaveCloudToTXT(const char* filename,PCloud pcloud){
		
	std::ofstream txt_file(filename);
	
	txt_file << pcloud->width << " " << pcloud->height << "\n"; //x y
	
	for(int v=0;v<pcloud->height;v++) {	  //480
		for(int u=0;u<pcloud->width;u++) { //640
			if (!isnan((*pcloud)(u,v).z)){
				txt_file << (*pcloud)(u,v).x << " " << (*pcloud)(u,v).y << " " << (*pcloud)(u,v).z << " "
						 <<(int) (*pcloud)(u,v).r << " " << (int) (*pcloud)(u,v).g << " " << (int) (*pcloud)(u,v).b << "\n";
			}else{
				txt_file << 0.0 << " " << 0.0 << " " << 0.0 << " " 
						 <<(int) (*pcloud)(u,v).r << " " << (int) (*pcloud)(u,v).g << " " << (int) (*pcloud)(u,v).b << "\n";
			}
			
		}
	}
	
	
	txt_file.close();
}


//load cloud from a .txt file
void LoadCloudFromTXT(const char* filename, Cloud &cloud){
		
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	
	cloud.header.stamp=ros::Time::now();
	cloud.header.frame_id="/openni_rgb_optical_frame";
	cloud.is_dense=false;
	
	std::ifstream txt_file(filename);
	
	txt_file >> cloud.width;
	txt_file >> cloud.height;

	cloud.points.resize(cloud.width*cloud.height);
	
	int pr;
	
	for(int v=0;v<cloud.height;v++) {	  	//480
		for(int u=0;u<cloud.width;u++) { 	//640
				txt_file >> cloud(u,v).x; 
				txt_file >> cloud(u,v).y;
				txt_file >> cloud(u,v).z;
				
				if(cloud(u,v).z==0.0){
					cloud(u,v).x = bad_point;
					cloud(u,v).y = bad_point;
					cloud(u,v).z = bad_point;		
				}
				
				txt_file >> pr;
				cloud(u,v).r = 	(uint8_t) pr;
				txt_file >> pr;
				cloud(u,v).g = 	(uint8_t) pr;
				txt_file >> pr;
				cloud(u,v).b = 	(uint8_t) pr;
		}
	}
	
	txt_file.close();	
}
void LoadCloudFromTXT(const char* filename, PCloud &pcloud){
		
	pcloud = PCloud (new Cloud());
	
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	
	pcloud->header.stamp=ros::Time::now();
	pcloud->header.frame_id="/openni_rgb_optical_frame";
	pcloud->is_dense=false;
	
	std::ifstream txt_file(filename);
	
	txt_file >> pcloud->width;
	txt_file >> pcloud->height;

	pcloud->points.resize(pcloud->width*pcloud->height);
	
	int pr;
	
	for(int v=0;v<pcloud->height;v++) {	  	//480
		for(int u=0;u<pcloud->width;u++) { 	//640
				txt_file >> (*pcloud)(u,v).x; 
				txt_file >> (*pcloud)(u,v).y;
				txt_file >> (*pcloud)(u,v).z;
				
				if((*pcloud)(u,v).z==0.0){
					(*pcloud)(u,v).x = bad_point;
					(*pcloud)(u,v).y = bad_point;
					(*pcloud)(u,v).z = bad_point;		
				}
				
				txt_file >> pr;
				(*pcloud)(u,v).r = 	(uint8_t) pr;
				txt_file >> pr;
				(*pcloud)(u,v).g = 	(uint8_t) pr;
				txt_file >> pr;
				(*pcloud)(u,v).b = 	(uint8_t) pr;
				
		}
	}
	
	txt_file.close();	
}
void LoadCloudFromTXT(const char* filename, PCloudxyz &pcloud){
		
	pcloud = PCloudxyz (new Cloudxyz());
	
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	
	pcloud->header.stamp=ros::Time::now();
	pcloud->header.frame_id="/openni_rgb_optical_frame";
	pcloud->is_dense=false;
	
	std::ifstream txt_file(filename);
	
	txt_file >> pcloud->width;
	txt_file >> pcloud->height;

	pcloud->points.resize(pcloud->width*pcloud->height);
	
	int pr;
	
	for(int v=0;v<pcloud->height;v++) {	  	//480
		for(int u=0;u<pcloud->width;u++) { 	//640
				txt_file >> (*pcloud)(u,v).x; 
				txt_file >> (*pcloud)(u,v).y;
				txt_file >> (*pcloud)(u,v).z;
				
				if((*pcloud)(u,v).z==0.0){
					(*pcloud)(u,v).x = bad_point;
					(*pcloud)(u,v).y = bad_point;
					(*pcloud)(u,v).z = bad_point;		
				}
				
				txt_file >> pr;
				txt_file >> pr;
				txt_file >> pr;
		}
	}
	
	txt_file.close();	
}
void loadCloudFromPLY(const char* filename, PCloud &pcloud){
	pcloud = PCloud (new Cloud());
		
	pcloud->header.stamp=ros::Time::now();
	pcloud->header.frame_id="/openni_rgb_optical_frame";
	pcloud->is_dense=false;
	
	std::ifstream ply_file(filename);
		
	char buffer[20];
	char buffer2[20];
	int npoints;
	
	ply_file >> buffer;
		
	do{			
		if(strcmp(buffer,"element")==0){
			ply_file >> buffer2;
			if(strcmp(buffer2,"vertex")==0){
				ply_file >> npoints;
			}
		}
			
		ply_file >> buffer;
			
	}while (strcmp(buffer,"end_header")!=0);
		
		
	if(npoints==640*480){
		pcloud->width = 640;
		pcloud->height = 480;
	}else{ 	
		pcloud->width = npoints;
		pcloud->height = 1;
	}
		
	pcloud->points.resize(pcloud->width*pcloud->height);
		
	char line[100];
		
	ply_file.getline(line,100);
		
	for(int i=0; i<npoints;i++){
		int r, g, b; 
		pcl::PointXYZRGB p;
		
		ply_file.getline(line,100);
		
		sscanf(line,"%f %f %f %d %d %d",&p.x, &p.y, &p.z, &r, &g, &b);
		
		p.r = (uint8_t) r;
		p.g = (uint8_t) g;
		p.b = (uint8_t) b;
		
		pcloud->points[i] = p;	
			
	}
		
	ply_file.close();
	
}


//Print cloud into a .ply file
void PrintCloudToPLY(const char* filename, Cloud cloud, bool allpoints){ 
	
	long pos;
	int npoints=0;
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	
	
	if(allpoints){
		ply_file << "element vertex " << cloud.height*cloud.width << "\n";
	}else{
		ply_file << "element vertex ";  pos=ply_file.tellp();
		ply_file << "               \n";
	}
	
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(unsigned int i=0;i<cloud.size();i++){
		if (allpoints || !isnan(cloud.points[i].z)){
			ply_file << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " "
					 <<(int) cloud.points[i].r << " " << (int) cloud.points[i].g << " " << (int) cloud.points[i].b << "\n";
			npoints++;
		}
	} 
	
	if(!allpoints){	
		ply_file.seekp(pos);
		ply_file << npoints;
	}
	
	ply_file.close();
	
}
void PrintCloudToPLY(const char* filename, PCloud pcloud, bool allpoints){ 
	
	long pos;
	int npoints=0;
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	
	
	if(allpoints){
		ply_file << "element vertex " << pcloud->height*pcloud->width << "\n";
	}else{
		ply_file << "element vertex ";  pos=ply_file.tellp();
		ply_file << "               \n";
	}
	
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(unsigned int i=0;i<pcloud->size();i++){
		if (allpoints || !isnan(pcloud->points[i].z)){
			
			ply_file << pcloud->points[i].x << " " << pcloud->points[i].y << " " << pcloud->points[i].z << " "
					 <<(int) pcloud->points[i].r << " " << (int) pcloud->points[i].g << " " << (int) pcloud->points[i].b << "\n";
			
			npoints++;
		}
	} 
	
	
	if(!allpoints){	
		ply_file.seekp(pos);
		ply_file << npoints;
	}
	
	ply_file.close();
	
}
void PrintCloudToPLY(const char* filename, PCloud pcloud, std::vector<int> indexs){ 
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	ply_file << "element vertex " << indexs.size() << "\n";
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(unsigned int i=0;i<indexs.size();i++){
		if (!isnan(pcloud->points[indexs[i]].z)){		
			ply_file << pcloud->points[indexs[i]].x << " " << pcloud->points[indexs[i]].y << " " << pcloud->points[indexs[i]].z << " "
					 <<(int) pcloud->points[indexs[i]].r << " " << (int) pcloud->points[indexs[i]].g << " " << (int) pcloud->points[indexs[i]].b << "\n";
		}
	} 
	
	ply_file.close();
	
}
void PrintCloudToPLYTriang(const char* filename, PCloud pcloud){ 
	
	long pos;
	int npoints=0;
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";


	ply_file << "element vertex "<< pcloud->width*pcloud->height << "\n";
		
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	
	ply_file << "element face ";  pos=ply_file.tellp(); 
	ply_file << "                    \n";
	ply_file << "property list uchar int vertex_indices\n";
	
	ply_file << "end_header\n";

	
	for(unsigned int i=0;i<pcloud->size();i++){
		if (!isnan(pcloud->points[i].z)){	
			ply_file << pcloud->points[i].x << " " << pcloud->points[i].y << " " << pcloud->points[i].z << " "
					 <<(int) pcloud->points[i].r << " " << (int) pcloud->points[i].g << " " << (int) pcloud->points[i].b << "\n";
		}else{
			ply_file << 0 << " " << 0 << " " << 0 << " " << (int) pcloud->points[i].r << " " << (int) pcloud->points[i].g << " " << (int) pcloud->points[i].b << "\n";
		}	
	} 
	
	//~ for(int v=0;v<pcloud->height-1;v++) {	  	//480
		//~ for(int u=0;u<pcloud->width-1;u++) { 	//640
			//~ if (!isnan((*pcloud)(u,v).z) && !isnan((*pcloud)(u+1,v).z) && !isnan((*pcloud)(u,v+1).z) && !isnan((*pcloud)(u+1,v+1).z)){	
				//~ ply_file << 3 << " " << v*pcloud->width+u << " " << v*pcloud->width+u+1  << " " << (v+1)*pcloud->width+u <<"\n";
				//~ ply_file << 3 << " " << (v+1)*pcloud->width+u+1 << " " << v*pcloud->width+u+1  << " " << (v+1)*pcloud->width+u <<"\n";
				//~ npoints2++;
				//~ npoints2++;
			//~ }
		//~ }
	//~ } 
	for(int v=0;v<pcloud->height-1;v++) {	  	//480
		for(int u=0;u<pcloud->width-1;u++) { 	//640
			if (!isnan((*pcloud)(u,v).z)){
				int i,k;
				for(i=v+1;i<pcloud->height;i++)
					if (!isnan((*pcloud)(u,i).z))break;
				
				for(k=u+1;k<pcloud->width;k++)
					if (!isnan((*pcloud)(k,v).z))break;
				
				if(i!=pcloud->height && k!=pcloud->width){
						ply_file << 3 << " " << v*pcloud->width+u << " " << v*pcloud->width+k  << " " << i*pcloud->width+u <<"\n";
					ply_file << 3 << " " << i*pcloud->width+k << " " << v*pcloud->width+k  << " " << i*pcloud->width+u <<"\n";
					npoints++;
					npoints++;					   
				}
			}
				
		}
	} 
	
	/*
	int c1=0, c2=0;
	for(int v=0;v<pcloud->height-1;v++) {	  	//480
		for(int u=0;u<pcloud->width-1;u++) { 	//640
			
			if (isnan((*pcloud)(u,v).z)) continue;
			
			//descobre um ponto onde pode ligar
			if (c1==0 && c2==0){
				for(int k=v+1;k<pcloud->height;k++){ 
					for (int i=0 ; i<pcloud->width ; i++){
						if (isnan((*pcloud)(i,k).z)){
							continue;
						}else{
							c1 = i;
							c2 = k;
							break;
						}
					}
					if (c1!=0) break;
				}	
			}
			
			ply_file << 3 << " " << v*pcloud->width+u << " " << v*pcloud->width+u+1  << " " << c2*pcloud->width+c1;
			
		}	
		
		//tenho o c1 e c2 e tenho o u e o v
		for(int l=c1;l<pcloud->width-1;l++){
			if (isnan((*pcloud)(l,c2).z)) continue;
			
			ply_file << 3 << " " << c2*pcloud->width+l << " " << c2*pcloud->width+l+1  << " " << v*pcloud->width+u;
		}
		
		c1=0;
		c2=0;
	}
	*/
	
	ply_file.seekp(pos);
	ply_file << npoints;
	
	ply_file.close();
	
}
void PrintCloudToPLY(const char* filename, PCloud pcloud, Eigen::Matrix4d T, bool allpoints){ 
	
	long pos;
	int npoints=0;
	Eigen::Vector4d vec(0.0,0.0,0.0,1.0);
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	
	
	if(allpoints){
		ply_file << "element vertex " << pcloud->height*pcloud->width << "\n";
	}else{
		ply_file << "element vertex ";  pos=ply_file.tellp();
		ply_file << "               \n";
	}
	
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(unsigned int i=0;i<pcloud->size();i++){
		if (allpoints || !isnan(pcloud->points[i].z)){
			
			vec(0)= pcloud->points[i].x;
			vec(1)= pcloud->points[i].y;
			vec(2)= pcloud->points[i].z;
			vec(3)=1.0;
			
			vec = T*vec;
			
			ply_file << vec(0) << " " << vec(1) << " " << vec(2) << " "
					 <<(int) pcloud->points[i].r << " " << (int) pcloud->points[i].g << " " << (int) pcloud->points[i].b << "\n";
			
			npoints++;
		}
	} 
	
	
	if(!allpoints){	
		ply_file.seekp(pos);
		ply_file << npoints;
	}
	
	ply_file.close();
	
}
void PrintCloudToPLY(const char* filename, PCloud pcloud, std::vector<Eigen::Vector3d> Normals ){
	
	 float escala =0.2;
	
	long pos;
	int npoints=0;
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	ply_file << "element vertex ";  pos=ply_file.tellp();
	ply_file << "                     \n";
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property float nx\n";
	ply_file << "property float ny\n";
	ply_file << "property float nz\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(int i=0;i<pcloud->size();i++) {
			if (!isnan(pcloud->points[i].z)){
				ply_file << pcloud->points[i].x << " " << pcloud->points[i].y << " " << pcloud->points[i].z << " "
				         << escala*Normals[i](0) << " " << escala*Normals[i](1) << " " << escala*Normals[i](2) << " "
						 << (int) pcloud->points[i].r << " " << (int) pcloud->points[i].g << " " << (int) pcloud->points[i].b << "\n";
						
				npoints++;
			}
	}
		
	ply_file.seekp(pos);
	ply_file << npoints;
	ply_file.close();
	
}

void WriteCameraPLY(const char* filename, std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_position){
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	ply_file << "element vertex " << camera_position.size()*11 <<" \n";
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";
	
	for (int i=0; i<camera_position.size(); i++) {
		ply_file << camera_position[i].first(0) << " " 
		         << camera_position[i].first(1) << " "
						 << camera_position[i].first(2) << " " << 0 << " " << 0 << " " << 0 << "\n";
		
		for (double j=0.1; j<=1; j=j+0.1) {
			
			ply_file << camera_position[i].first(0) + (camera_position[i].second(0) - camera_position[i].first(0))*j << " " 
							 << camera_position[i].first(1) + (camera_position[i].second(1) - camera_position[i].first(1))*j << " "
							 << camera_position[i].first(2) + (camera_position[i].second(2) - camera_position[i].first(2))*j << " " << 160 << " " << 0 << " " << 0 << "\n";	

		}
		
 	}

	ply_file.close();
	
}


void SaveKinectPose(const char* filename, double x,double y,double z,double ax, double ay, double az,double q0, double q1, double q2, double q3){
	
	std::ofstream txt_file(filename);
	
	txt_file << x << " " << y << " " << z << "\n";
	txt_file << ax << " " << ay << " " << az << "\n";
	
	if(q0==0.0 && q1==0.0 && q2==0.0 && q3==0.0){}else{
		txt_file << q0 << " " << q1 << " " << q2 << " " << q3 << "\n";
	}
	
	txt_file.close();
	
}


void saveDepthMatrix(std::string filename, PCloud pcloud){
	
	std::ofstream txt_file(filename.data());

	for(int v=0;v<pcloud->height;v++) {	  	//480
		for(int u=0;u<pcloud->width;u++) { 	//640
			txt_file << (*pcloud)(u,v).z << " ";
		}
		txt_file << "\n";
	}
	
	txt_file.close();
}
void saveDepthImagePNG(std::string filename, PCloud pcloud){
	
	cv::Mat RGBImage(pcloud->height,pcloud->width,CV_8UC3);
		

	for(unsigned int v=0;v<pcloud->height;v++) {	   //480
		for(unsigned int u=0;u<pcloud->width;u++) {  //640
		
		
		int k = 255*6*((*pcloud)(u,v).z-0.55)/(9.75-0.55);
           if (k < 0 or isnan((*pcloud)(u,v).z)) k = 0;
              
		int lb = k & 0xff;
		switch (k / 256) {
			case 0:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 255;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 255-lb;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 255-lb;
				}break;
					
			case 1:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 255;
				RGBImage.at<cv::Vec3b>(v,u)[1] = lb;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 0;
				}break;
			case 2:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 255-lb;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 255;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 0;
				}break;
			case 3:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 255;
				RGBImage.at<cv::Vec3b>(v,u)[0] = lb;
				}break;
			case 4:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 255-lb;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 255;
				}break;
			case 5:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 255-lb;
				}break;
			default:{
				RGBImage.at<cv::Vec3b>(v,u)[2] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 0;
				}
			}
			
			if (k == 0)
               {
				RGBImage.at<cv::Vec3b>(v,u)[2] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[1] = 0;
				RGBImage.at<cv::Vec3b>(v,u)[0] = 0;
               }
		}

	}
	
	
	std::vector<int> param = std::vector<int>(2);
	
	param[0]=CV_IMWRITE_PNG_COMPRESSION;
	param[1]=3;//default(3)  0-9.
	
	cv::imwrite(filename, RGBImage, param);
	
}

void saveRGBimageJPG(std::string filename, PCloud pcloud){
	
	cv::Mat RGBImage(pcloud->height,pcloud->width,CV_8UC3);
		
	for(unsigned int v=0;v<pcloud->height;v++) {	   //480
		for(unsigned int u=0;u<pcloud->width;u++) {  //640
			
			RGBImage.at<cv::Vec3b>(v,u)[0] = (int) (*pcloud)(u,v).b; 
			RGBImage.at<cv::Vec3b>(v,u)[1] = (int) (*pcloud)(u,v).g; 
			RGBImage.at<cv::Vec3b>(v,u)[2] = (int) (*pcloud)(u,v).r; 	
		}
	}
	
	
	std::vector<int> param = std::vector<int>(2);
	
    param[0]=CV_IMWRITE_JPEG_QUALITY;
    param[1]=95;//default(95) 0-100

	cv::imwrite(filename, RGBImage, param);
	
}
void saveRGBimagePNG(std::string filename, PCloud pcloud){
	
	cv::Mat RGBImage(pcloud->height,pcloud->width,CV_8UC3);
		
	for(unsigned int v=0;v<pcloud->height;v++) {	   //480
		for(unsigned int u=0;u<pcloud->width;u++) {  //640
			
			RGBImage.at<cv::Vec3b>(v,u)[0] = (int) (*pcloud)(u,v).b; 
			RGBImage.at<cv::Vec3b>(v,u)[1] = (int) (*pcloud)(u,v).g; 
			RGBImage.at<cv::Vec3b>(v,u)[2] = (int) (*pcloud)(u,v).r; 	
		}
	}
	
	
	std::vector<int> param = std::vector<int>(2);
	
	param[0]=CV_IMWRITE_PNG_COMPRESSION;
    param[1]=3;//default(3)  0-9.
	
	cv::imwrite(filename, RGBImage, param);
}

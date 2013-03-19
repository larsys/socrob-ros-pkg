#include <iostream>
#include <fstream>

#include "ICPBlock.h"
#include "ICP_point2plane.h"

#include "UtilitiesBlock.h"
#include "ICPInitializationBlock.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


char filename_txt[130];
char filename_ply[130];


class TestICP : public ICP{
	
public: 
	TestICP() : ICP(){}
	TestICP(PCloud cl_s, PCloud cl_d): ICP(cl_s,cl_d){}
	
	void test_select_macth();
	void align(Eigen::Matrix4d Tini = Eigen::MatrixXd::Identity(4,4)); 

	//main0
	void PrintInputCloudsToPLY(char *path = "/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds");
	
private:

	
};

//TestICP class
void TestICP::PrintInputCloudsToPLY(char *path){
	
	
	sprintf (filename_ply, "%s/test_cloud_s.ply", path);
	PrintCloudToPLY(filename_ply, cloud_s);
	
	sprintf (filename_ply, "%s/test_cloud_d.ply", path);
	PrintCloudToPLY(filename_ply, cloud_d);
}

void TestICP::test_select_macth(){
	
	std::ofstream txt_file("/home/pedrovieira/ros_workspace/icp_ros/bin/test_select.txt");
	
	//Create kdtree with cloud_s 
	kdtree.setInputCloud (cloud_s);
	
	T.setIdentity();
	
	std::vector<int> source_indices(cloud_d->size());
	std::vector<int> data_indices(cloud_d->size());
	
	txt_file << source_indices.size() << " " << data_indices.size() << "\n"; //x y
	
	select_macth(source_indices, data_indices);
	
	txt_file << source_indices.size() << " " << data_indices.size() << "\n"; //x y
	
	for (int i =0 ; i<source_indices.size() ; i++){
		txt_file << source_indices[i] << " " << data_indices[i] << "\n"; //x y
	}
	
	txt_file.close();
	
}

void TestICP::align(Eigen::Matrix4d Tini){
	
	ros::Time start;
	ros::Time end;
	double a0=0.0, a1=0.0, a2=0.0, a3=0.0;

	Eigen::VectorXd xopt_ant(7);	//7 q(0) q(1) q(2) q(3) Tx Ty Tz 
	Eigen::VectorXd xopt(7);		//7 q(0) q(1) q(2) q(3) Tx Ty Tz

	//Create kdtree with cloud_s 
	start = ros::Time::now();	
	kdtree.setInputCloud (cloud_s);
	end = ros::Time::now();
	a0 = (end-start).toSec(); 
	
	T = Tini;
	
	UpdateXopt(xopt,T);
	xopt_ant=xopt;
	
	int i;
	for(i=0; i<max_iterations; i++){
		
		std::vector<int> source_indices(cloud_d->size());
		std::vector<int> data_indices(cloud_d->size());
		
		start = ros::Time::now();	
		select_macth(source_indices,data_indices);
		end = ros::Time::now();
		a1 = a1 + (end-start).toSec(); 
		
		start = ros::Time::now();	
		UpdateAndReject(source_indices,data_indices);
		end = ros::Time::now();
		a2 = a2 + (end-start).toSec(); 
		
		start = ros::Time::now();	
		minimize(source_indices,data_indices);
		end = ros::Time::now();
		a3 = a3 + (end-start).toSec(); 
		
		UpdateXopt(xopt,T);
		
		if(error(xopt_ant,xopt)) break;
		
		xopt_ant=xopt;
	}
	
	std::cout << "interatins: " << i << "\n"
		      << "Tree: " << a0 << " \n"
		      << "select: " << a1 << "\n"
		      << "reject: " << a2 << "\n"
		      << "minimize: " << a3 << "\n";
 	
}



class MapMaker{
	ros::Publisher  pub_pointcloud; //send frames
	ros::Subscriber sub_pointcloud; //get frames
	
	PCloud cloudrec;
		
	Eigen::Matrix4d T;

public:
	MapMaker(){
		T.setIdentity();
	}
   ~MapMaker(){}
   
	//main1
	void makemap();				//infinitos frames, para com ctrl+c
	void makemap(int nframes);  //usar o topico world
	void makemap2(int nframes); //escreve 1 ficeiro
	void makemap_times(int nframes);  //usar o topico world com tempos
	

private:	
	void get_cloud_Callback(const PCloud& pcloud){cloudrec=pcloud;}
	void ApplyRTto(PCloud &pc);
	
};


//Make Map using only RGB features
void MapMaker::makemap(){
	 /*- tira dois frames
	 - inicializa o icp
	 - compute R T
	 - actualiza R T
	 - aplica R T ao cloud_d
	 - manda para o wordtopic
	 - loop k-2:
	      tira um novo frame
	      update icp class
	      compute R T
	      actualiza R T
	      aplica R e T ao cloud_d
	      manda para o word topic */
	
	ros::NodeHandle n;
	ros::Rate r(2); // 10 hz
	
	pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("world", 1);
	sub_pointcloud = n.subscribe("/pointcloud", 1, &MapMaker::get_cloud_Callback, this);
	
	PCloud cloud_s;
	PCloud cloud_d;
	
	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	r.sleep();
	ros::spinOnce();
	
	RGBFeaturesInit rgbf;
	rgbf.setScloud(cloud_s);
	
	int i=1;
	int k=1;
	while(ros::ok()) {

		std::cout<< i << "\n";

		cloud_d = cloudrec;
		
		//update icp clouds
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
		}
		if (rgbf.compute() > 10 ){
			
			k=0;		
			//update transformation
			T = T*rgbf.T;
			
			ApplyRTto(cloud_d);
	
			pub_pointcloud.publish(cloud_d);
						
		}else{
			k=1;
			std::cout<< "Lost!! " << "\n";
		}
		
		r.sleep();	
		ros::spinOnce();
		i++;
	}
}
void MapMaker::makemap(int nframes){
	 /*- tira dois frames
	 - inicializa o icp
	 - compute R T
	 - actualiza R T
	 - aplica R T ao cloud_d
	 - manda para o wordtopic
	 - loop k-2:
	      tira um novo frame
	      update icp class
	      compute R T
	      actualiza R T
	      aplica R e T ao cloud_d
	      manda para o word topic */
	
	ros::NodeHandle n;	
	ros::Rate r(2); // 10 hz
	
	pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("world", 1);
	sub_pointcloud = n.subscribe("/pointcloud", 1, &MapMaker::get_cloud_Callback, this);
	
	PCloud cloud_s;
	PCloud cloud_d;

	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	r.sleep();
	ros::spinOnce();
	
	RGBFeaturesInit rgbf;
	rgbf.setScloud(cloud_s);

	int k=1;
	for(int i=1; i<nframes; i++){

		std::cout<< i << "\n";

		cloud_d = cloudrec;
		
		//update icp clouds
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
		}
		if (rgbf.compute() > 4){
			
			k=0;		
			//update transformation		
			T = T*rgbf.T;
			
			ApplyRTto(cloud_d);
	
			pub_pointcloud.publish(cloud_d);
						
		}else{
			k=1;
		}
		
		r.sleep();	
		ros::spinOnce();
		
	}	
}
void MapMaker::makemap2(int nframes){
	 /*- tira dois frames
	 - inicializa o icp
	 - compute R T
	 - actualiza R T
	 - aplica R T ao cloud_d
	 - manda para o wordtopic
	 - loop k-2:
	      tira um novo frame
	      update icp class
	      compute R T
	      actualiza R T
	      aplica R e T ao cloud_d
	      manda para o word topic */
	
	long pos;
	unsigned int npoints=0;

	std::ofstream ply_file("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/world_icp.ply");
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	ply_file << "element vertex ";  pos=ply_file.tellp();
	ply_file << "                   \n";
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";


	ros::NodeHandle n;	
	ros::Rate r(2); // 10 hz
	
	sub_pointcloud = n.subscribe("/pointcloud", 1, &MapMaker::get_cloud_Callback, this);
	
	PCloud cloud_s;
	PCloud cloud_d;

	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	std::cout<< 0 << "\n";	
	
	for(int v=0;v<cloud_s->height;v++) {	  //480
		for(int u=0;u<cloud_s->width;u++) {   //640
			if (!isnan((*cloud_s)(u,v).z)){
				ply_file << (*cloud_s)(u,v).x << " " << (*cloud_s)(u,v).y << " " << (*cloud_s)(u,v).z << " "
						 <<(int) (*cloud_s)(u,v).r << " " << (int) (*cloud_s)(u,v).g << " " << (int) (*cloud_s)(u,v).b << "\n";
				npoints++;
			}
		}
	}
	
	r.sleep();
	ros::spinOnce();
	
	RGBFeaturesInit rgbf;
	rgbf.setScloud(cloud_s);

	int k=1;
	for(int i=1; i<nframes; i++){
		//new frame

		std::cout<< i << "\n";

		cloud_d = cloudrec;
		
		//update icp clouds
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
		}
		
		if (rgbf.compute() > 4){
			
			k=0;		
			//update transformation		
			T = T*rgbf.T;
		
			ApplyRTto(cloud_d);
			
			for(int v=0;v<cloud_d->height;v++) {	  //480
				for(int u=0;u<cloud_d->width;u++) {   //640
					if (!isnan((*cloud_d)(u,v).z)){
								
						ply_file << (*cloud_d)(u,v).x << " " << (*cloud_d)(u,v).y << " " << (*cloud_d)(u,v).z << " "
								 <<(int) (*cloud_d)(u,v).r << " " << (int) (*cloud_d)(u,v).g << " " << (int) (*cloud_d)(u,v).b << "\n";
						npoints++;
					}
				}
			}
				
		}else{
			k=1;
		}
		
		r.sleep();	
		ros::spinOnce();
		
	}
	
	ply_file.seekp(pos);
	ply_file << npoints;
	ply_file.close();
	
}
void MapMaker::makemap_times(int nframes){

	ros::Time start;
	ros::Time end;
	
	start = ros::Time::now();	
	
	ros::NodeHandle n;	
	ros::Rate r(2); // 10 hz
	
	pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("world", 1);
	sub_pointcloud = n.subscribe("/pointcloud", 1, &MapMaker::get_cloud_Callback, this);
	
	PCloud cloud_s;
	PCloud cloud_d;
	end = ros::Time::now();
	printf("Inicializaçao de ariaveis (nodeshandles) : %.6f seconds\n", (end-start).toSec());


	//take 1 frame
	start = ros::Time::now();	
	
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	
	r.sleep();
	ros::spinOnce();
	
	end = ros::Time::now();
	printf("1 frame and send do world : %.6f seconds\n", (end-start).toSec());
	
	start = ros::Time::now();	
	
	RGBFeaturesInit rgbf;
	rgbf.setScloud(cloud_s);


	end = ros::Time::now();
	printf("Cria vaiavel icp e mete 1 frame: %.6f seconds\n", (end-start).toSec());

	int k=1;
	int b;
	
	double a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0, a6=0.0;
	
	for(int i=1; i<nframes; i++){

		std::cout<< i << "\n";

		cloud_d = cloudrec;
		
		//update icp clouds
		start = ros::Time::now();	
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
		}
		end = ros::Time::now();
		a1 = a1 + (end-start).toSec();
		
		
		start = ros::Time::now();	
		b = rgbf.compute();
		end = ros::Time::now();
		a2 = a2 + (end-start).toSec();
		
		if (b > 4){
			
			start = ros::Time::now();	
			k=0;		
			//update transformation
			T = T*rgbf.T;
			end = ros::Time::now();
			a3 = a3 + (end-start).toSec();
		
			start = ros::Time::now();	
			ApplyRTto(cloud_d);
			end = ros::Time::now();
			a4 = a4 + (end-start).toSec();
	
			start = ros::Time::now();		
			pub_pointcloud.publish(cloud_d);
			end = ros::Time::now();			
			a5 = a5 + (end-start).toSec();	
						
		}else{
			k=1;
		}
		
		start = ros::Time::now();	
		r.sleep();	
		ros::spinOnce();
		end = ros::Time::now();
		a6 = a6 + (end-start).toSec();
	}	
	
	std::cout << "Update icp clouds: " << a1/(nframes-1) << "\n"
			  << "ComputeRT: "         << a2/(nframes-1) << "\n"
			  << "Update RT: "         << a3/(nframes-1) << "\n"
			  << "ApplyRTto: "         << a4/(nframes-1) << "\n"
			  << "publish: "           << a5/(nframes-1) << "\n"
			  << "spinOnce: "          << a6/(nframes-1) << "\n";
	
	
}

void MapMaker::ApplyRTto(PCloud &pc){
	
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	PCloud pcloud = PCloud(new Cloud());
	
	pcloud->header.stamp = pc->header.stamp;
	pcloud->header.frame_id = pc->header.frame_id;
	pcloud->is_dense = pc->is_dense;
	
	pcloud->width = pc->width;
	pcloud->height = pc->height;	
	
	pcloud->points.resize(pcloud->width*pcloud->height);		
		
	for(int v=0;v<pc->height;v++) {	    //480
		for(int u=0;u<pc->width;u++) {  //640
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


//test ICP functions
void main0(int argc, char* argv[]){
	//recebe cloud_s.txt cloud_d.txt
	ros::init(argc, argv, "test_icp_node");
	ros::NodeHandle n;
	
	
	PCloud cl_s; 
	PCloud cl_d;
	
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
	LoadCloudFromTXT(filename_txt, cl_s);
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);
	LoadCloudFromTXT(filename_txt, cl_d);
	
	TestICP t(cl_s,cl_d);
	
	switch(atoi(argv[4])){
		case 0: {t.PrintInputCloudsToPLY();} break;
		case 1: {t.test_select_macth();} break;
	}
	
	
}

//Make Map using only rgb features with k frames
void main1(int argc, char* argv[]){
	
	if(argc!=3 ){
		std::cout<<"main1 format: ./exe opt nframes\n";
		return;
	}
		
	ros::init(argc, argv, "test_mapmaker");
	ros::NodeHandle aux;
	
	ros::Time start;
	ros::Time end;
	
	std::cout<< "start!!" << "\n";
	MapMaker mm;
	start = ros::Time::now();	
	mm.makemap(atoi(argv[2]));
	end = ros::Time::now();
	printf("Map makin time: %.6f seconds\n", (end-start).toSec()/atoi(argv[2]));
	
	std::cout<< "done!!\n";
	
}

//Make Map using only rgb features
void main2(int argc, char* argv[]){
	
	ros::init(argc, argv, "test_mapmaker");
	
	std::cout<< "start!!" << "\n";
	MapMaker mm;
	mm.makemap();
	std::cout<< "done!!\n";
	
}

//Make Map using only rgb features with k frames with times
void main3(int argc, char* argv[]){
	
	if(argc!=3 ){
		std::cout<<"main1 format: ./exe opt nframes\n";
		return;
	}
		
	ros::init(argc, argv, "test_mapmaker");
	ros::NodeHandle aux;
	
	
	std::cout<< "start!!" << "\n";
	MapMaker mm;
	mm.makemap_times(atoi(argv[2]));
	std::cout<< "done!!\n";
	
}


void main4(int argc, char* argv[]){
	
	if(argc!=4 ){
		std::cout<<"main4 format: ./exe int cloud_s.txt cloud_d.txt\n";
		return;
	}
		
	ros::init(argc, argv, "test_mapmaker");
	ros::NodeHandle aux;
	
	std::cout<< "start!!" << "\n";
	
	PCloud cl_s; 
	PCloud cl_d;
	Eigen::Vector4d point(0.0,0.0,0.0,1.0);

	ros::Time start;
	ros::Time end;
	
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
	LoadCloudFromTXT(filename_txt, cl_s);
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);
	LoadCloudFromTXT(filename_txt, cl_d);
	
	start = ros::Time::now();	
	
	RGBFeaturesInit rgbf(cl_s,cl_d);
	ICP icp(cl_s,cl_d);

	rgbf.compute();
	icp.align(rgbf.T);
	
	end = ros::Time::now();
	std::cout << "Tempo Alinhamento: " <<  (end-start).toSec() << "\n";
	
	TestICP t(cl_s,cl_d);
	t.align(rgbf.T);
	
	
	for(int v=0;v<cl_d->height;v++) {	  //480
		for(int u=0;u<cl_d->width;u++) {  //640
			if (!isnan((*cl_d)(u,v).z)){
				
				point(0) = (*cl_d)(u,v).x;
				point(1) = (*cl_d)(u,v).y;
				point(2) = (*cl_d)(u,v).z;
				
				//std::cout << point.transpose() << " ";
			
				//std::cout<< icp.returnR() << " " << icp.returnT().transpose() << "\n\n"; 
			
				point = icp.returnTr()*point;
			
				//std::cout << point.transpose() << "\n";
		
		
				(*cl_d)(u,v).x = point(0);
				(*cl_d)(u,v).y = point(1);
				(*cl_d)(u,v).z = point(2);
						
			}
		}
	}
	
	
	PrintCloudToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/icp_cloud.ply", cl_d);

	std::cout<< "done!!\n";
	
}


void main5(int argc, char* argv[]){
	
	if(argc!=4 ){
		std::cout<<"main4 format: ./exe int cloud_s.txt cloud_d.txt\n";
		return;
	}
		
	ros::init(argc, argv, "test_mapmaker");
	ros::NodeHandle aux;
	
	std::cout<< "start!!" << "\n";
	
	PCloud cl_s; 
	PCloud cl_d;
	Eigen::Vector4d point(0.0,0.0,0.0,1.0);

	ros::Time start;
	ros::Time end;
	
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
	LoadCloudFromTXT(filename_txt, cl_s);
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);
	LoadCloudFromTXT(filename_txt, cl_d);
	
	start = ros::Time::now();	
	
	RGBFeaturesInit rgbf(cl_s,cl_d);
	ICP_p2plane icp(cl_s,cl_d);

	rgbf.compute();
	icp.align(rgbf.T);
	//icp.align();
	
	end = ros::Time::now();
	std::cout << "Tempo Alinhamento: " <<  (end-start).toSec() << "\n";
	
	
	
	for(int v=0;v<cl_d->height;v++) {	  //480
		for(int u=0;u<cl_d->width;u++) {  //640
			if (!isnan((*cl_d)(u,v).z)){
				
				point(0) = (*cl_d)(u,v).x;
				point(1) = (*cl_d)(u,v).y;
				point(2) = (*cl_d)(u,v).z;
				
				//std::cout << point.transpose() << " ";
			
				//std::cout<< icp.returnR() << " " << icp.returnT().transpose() << "\n\n"; 
			
				point = icp.returnTr()*point;
			
				//std::cout << point.transpose() << "\n";
		
		
				(*cl_d)(u,v).x = point(0);
				(*cl_d)(u,v).y = point(1);
				(*cl_d)(u,v).z = point(2);
						
			}
		}
	}
	
	
	PrintCloudToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/icp_cloud.ply", cl_d);

	std::cout<< "done!!\n";
	
}



int main(int argc, char* argv[]){
	
	if(argc<2){
		std::cout << "0 - test icp functions | cloud_s.txt cloud_d.txt int\n"
				  << "           - int=0 : print clouds\n"
				  << "           - int=1 : test select\n"
		          << "1 - Make Map using only rgb features | number_of_frames \n"
		          << "2 - Make Map using only rgb features (ros loop) \n"
		          << "3 - Make Map using only rgb features (display times) | number_of_frames \n"
		          << "4 - Test only icp 2 frames | cloud_s.txt cloud_d.txt\n"
		          << "5 - Test only icp_p2plane 2 frames | cloud_s.txt cloud_d.txt\n\n"; 
		return 0;
	}
	
	
	switch(atoi(argv[1])){
		case 0: {main0(argc,argv);} break;
		case 1: {main1(argc,argv);} break;
		case 2: {main2(argc,argv);} break;
		case 3: {main3(argc,argv);} break;
		case 4: {main4(argc,argv);} break;
		case 5: {main5(argc,argv);} break;
	}
	
	return 0;
}






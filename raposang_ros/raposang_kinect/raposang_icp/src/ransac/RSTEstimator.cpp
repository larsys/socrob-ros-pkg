#include "ransac/RSTEstimator.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


RSTEstimator::RSTEstimator(double delta, const PCloud& c_s, const PCloud& c_d) : ParameterEstimator<std::pair<int,int> ,double>(3), deltaSquared(delta){
	cloud_s = c_s;
	cloud_d = c_d;
}


/*****************************************************************************/
/*
 * Compute the line parameters  [n_x,n_y,a_x,a_y]
 */
void RSTEstimator::estimate(std::vector<std::pair<int,int> *> &data, 
								  std::vector<double> &parameters)
{
	parameters.clear();
	
	//neste caso minForEstimate = 3 
	//1 ponto danos 3 equaçoes, como temos 12 incognitas precisamos de 4 pontos
	if(data.size()<this->minForEstimate) return;
	
	leastSquaresEstimate(data,parameters);
	
	/*
	for (int i=0; i<parameters.size(); i++) {
		std::cout << parameters[i] << " ";
	}
	
	std::cout << "\n";
	*/
	
}
/*****************************************************************************/
/*
 * Compute the homography parameters
 */
void RSTEstimator::leastSquaresEstimate(std::vector<std::pair<int,int> *> &points, 
									    std::vector<double> &parameters)
{
	
	unsigned int N = points.size(); 
	
	//checks
	/*
	if (N<3) {
		std::cout << "At least 3 point correspondences are needed" << std::endl;
		return;
	}
	*/
	
	Vector3d centroide_model(0.0,0.0,0.0), centroide_data(0.0,0.0,0.0);
	MatrixXd model(3,N);
	MatrixXd data(3,N);
	MatrixXd cov;	
	
	//compute centroids
	for (unsigned int i=0 ; i<N; i++) {
		
		model(0,i) = cloud_s->points[ points[i]->first ].x;
		model(1,i) = cloud_s->points[ points[i]->first ].y;
		model(2,i) = cloud_s->points[ points[i]->first ].z;
		
		data(0,i) = cloud_d->points[ points[i]->second ].x;
		data(1,i) = cloud_d->points[ points[i]->second ].y;
		data(2,i) = cloud_d->points[ points[i]->second ].z;
		
		centroide_model += model.block(0,i,3,1);
		centroide_data  += data.block(0,i,3,1);
	}
		
	centroide_model = centroide_model/N; 
	centroide_data = centroide_data/N; 
		
	
	//Subtract centroids to data
	for (unsigned int i=0; i<N; i++){
		model.block(0,i,3,1) -= centroide_model;
		data.block(0,i,3,1) -= centroide_data;
	}
	
	cov = data * model.transpose(); //maps data into model
	
	JacobiSVD<Matrix3d> svd(cov, ComputeFullU | ComputeFullV);
	
	Matrix3d U = svd.matrixU();
	Matrix3d V = svd.matrixV();

	if (U.determinant()*V.determinant()<0) {
		for (int i=0; i<3; ++i) {
			V(i,2) *=-1;
		}
	}
	
	Matrix3d R = V * U.transpose();
	Vector3d T = centroide_model - R*centroide_data;
			
	
	parameters.push_back(R(0,0));  //R11
	parameters.push_back(R(0,1));  //R12
	parameters.push_back(R(0,2));  //R13
	
	parameters.push_back(R(1,0));  //R21
	parameters.push_back(R(1,1));  //R22
	parameters.push_back(R(1,2));  //R23
	
	parameters.push_back(R(2,0));  //R31
	parameters.push_back(R(2,1));  //R32
	parameters.push_back(R(2,2));  //R33
	
	parameters.push_back(T(0));    //T1
	parameters.push_back(T(1));    //T2
	parameters.push_back(T(2));    //T3
	

	/*
		H = | Theta1   Theta2   Theta3   Theta10 |
			| Theta4   Theta5   Theta6   Theta11 |
			| Theta7   Theta8   Theta9   Theta12 |
			|   0         0        0        1	 |
	 */
	
}

/*****************************************************************************/
/*
 * Given the line parameters  [n_x,n_y,a_x,a_y] check if
 * [n_x, n_y] dot [data.x-a_x, data.y-a_y] < m_delta
 */
bool RSTEstimator::agree(std::vector<double> &parameters, std::pair<int,int> &data)
{
	
	Vector3d X12;
	Vector3d X21;
	Matrix3d R;
	Vector3d T;
	Vector3d dif;
	double E1,E2;
	
	
	R << parameters[0] , parameters[1] , parameters[2],
	     parameters[3] , parameters[4] , parameters[5],
	     parameters[6] , parameters[7] , parameters[8];
	
	T << parameters[9] , parameters[10] , parameters[11];
	
	Eigen::Vector3d model_p;
	Eigen::Vector3d data_p;
	
	model_p(0) = cloud_s->points[ data.first ].x;
	model_p(1) = cloud_s->points[ data.first ].y;
	model_p(2) = cloud_s->points[ data.first ].z;
		
	data_p(0) = cloud_d->points[ data.second ].x;
	data_p(1) = cloud_d->points[ data.second ].y;
	data_p(2) = cloud_d->points[ data.second ].z;
	
	
	X21 = R*data_p + T;
		
	X12 = R.inverse() * (model_p-T);
	
	dif = model_p - X21;
	E1 = dif.transpose()*dif;
	
	
	dif = data_p - X12;
	E2 = dif.transpose()*dif;
	
	//std::cout << "E1= " << E1 << "\nE2= " << E2 << "\nE1+E2= "<< E1+E2 << " " << this->deltaSquared <<"\n";
	
	return ( (E1 + E2) < this->deltaSquared);
}
/*****************************************************************************/

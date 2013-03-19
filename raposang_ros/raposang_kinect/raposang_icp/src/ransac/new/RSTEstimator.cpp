#include "RSTEstimator.h"
#include <Eigen/Dense>

using namespace Eigen;

RSTEstimator::RSTEstimator(double delta) : ParameterEstimator<std::pair<Eigen::Vector3d,Eigen::Vector3d> ,double>(3), deltaSquared(delta) {}


/*****************************************************************************/
/*
 * Compute the line parameters  [n_x,n_y,a_x,a_y]
 */
void RSTEstimator::estimate(std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> *> &data, 
								  std::vector<double> &parameters)
{
	parameters.clear();
	
	//neste caso minForEstimate =4
	//1 ponto danos 3 equaçoes, como temos 12 incognitas precisamos de 4 pontos
	if(data.size()<this->minForEstimate)  
		return;
	
	
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
void RSTEstimator::leastSquaresEstimate(std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> *> &points, 
									    std::vector<double> &parameters)
{
	
	unsigned int N = points.size(); 
	
	//checks
	if (N<3) {
		std::cout << "At least 3 point correspondences are needed" << std::endl;
		return;
	}
	
	Eigen::Vector3d centroide_model(0.0,0.0,0.0), centroide_data(0.0,0.0,0.0);
	Eigen::MatrixXd model(3,N);
	Eigen::MatrixXd data(3,N);

	Eigen::MatrixXd Ryx;
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	
	
	//compute centroids
	for (int i=0 ; i<N; i++) {

		model.block(0,i,3,1) = points[i]->first;
		data.block(0,i,3,1) = points[i]->second;
		
		centroide_model += points[i]->first;
		centroide_data  += points[i]->second;
	}
		
	centroide_model = centroide_model/N; 
	centroide_data = centroide_data/N; 
		
	
	//Subtract centroids to data
	for (int i=0; i<N; i++){
		model.block(0,i,3,1) -= centroide_model;
		data.block(0,i,3,1) -= centroide_data;
	}
	
	Ryx = data * model.transpose(); //maps data into model
	
	JacobiSVD<Eigen::Matrix3d> svd(Ryx, ComputeFullU | ComputeFullV);
	
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();

  if (U.determinant()*V.determinant()<0) {
		for (int i=0; i<3; ++i) {
			V(i,2) *=-1;
		}
	}
	
	R = V * U.transpose();
	T = centroide_model - R*centroide_data;
			
	
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
				|   0         0        0        1	   |
	 */
	
}

/*****************************************************************************/
/*
 * Given the line parameters  [n_x,n_y,a_x,a_y] check if
 * [n_x, n_y] dot [data.x-a_x, data.y-a_y] < m_delta
 */
bool RSTEstimator::agree(std::vector<double> &parameters, std::pair<Eigen::Vector3d,Eigen::Vector3d> &data)
{
	
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	Eigen::Vector3d dif;
	double  E1,E2;
	
	
	R << parameters[0] , parameters[1] , parameters[2],
	     parameters[3] , parameters[4] , parameters[5],
	     parameters[6] , parameters[7] , parameters[8];
	
	T << parameters[9] , parameters[10] , parameters[11];
	

    dif = data.first - R*data.second + T; //X21
	E1 = dif.transpose()*dif;
	
	dif = data.second - R.inverse() * (data.first-T); //X12
	E2 = dif.transpose()*dif;
	
	//std::cout << "E1= " << E1 << "\nE2= " << E2 << "\nE1+E2= "<< E1+E2 << " " << this->deltaSquared <<"\n";
	
	return ( (E1 + E2) < this->deltaSquared);
}
/*****************************************************************************/

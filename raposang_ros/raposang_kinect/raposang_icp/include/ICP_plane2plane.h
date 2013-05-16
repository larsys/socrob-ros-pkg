/*
 *  ICP_plane2plane.h
 *  
 *  Created by Pedro Vieira on 11/10/11.
 *  
 */
 
#ifndef ICP_PLANE2PLANE_H
#define ICP_PLANE2PLANE_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>


#include <unsupported/Eigen/NonLinearOptimization>
//#include <unsupported/Eigen/NumericalDiff>
//#include <cminpack.h>

#include "ICPBlock.h"

using namespace Eigen;

template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

 // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};


class ICP_plane2plane: public ICP {
protected:
	float gicp_epsilon;
	int k_correspondences;
	
	//std::vector<Eigen::Matrix3d> mahalanobis;  //S^-1
	std::vector<Eigen::Matrix3d> cov_mod;  
	std::vector<Eigen::Matrix3d> cov_dat;  
	
	const std::vector<int> *tmp_idx_mod;
    const std::vector<int> *tmp_idx_dat;
    const Pair *tmp_initf;
    
    pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree_d;
    
public:
	ICP_plane2plane () : ICP(){
		gicp_epsilon = 0.0004;//0.0004;
		k_correspondences = 20;
		tmp_idx_mod = tmp_idx_dat = NULL;
		tmp_initf = NULL;
		THRESH_dt = 0.004; //0.001 
	}
	ICP_plane2plane (PCloud cl_s, PCloud cl_d) : ICP(cl_s, cl_d){
		gicp_epsilon = 0.0004;//0.0004;
		k_correspondences= 20;
		tmp_idx_mod = tmp_idx_dat = NULL;
		tmp_initf = NULL;
		THRESH_dt = 0.004; //0.001 
	} 

	
	void setGicpEpsilon(double gicp_e){gicp_epsilon=gicp_e;}
	void setKcorrespondences(int k){k_correspondences=k;}

	
	bool align(Eigen::Matrix4d Tini = Eigen::MatrixXd::Identity(4,4), Pair init_f =Pair(0));

	//void TestTimeCov();
	
protected:
	void computeCovariance(const PCloud& pcloud, const std::vector<int> &cloud_indices, pcl::KdTreeFLANN<pcl::PointXYZRGB> *kdtree, std::vector<Eigen::Matrix3d> &cov_vec);
	int  computeMahalanobisMats (); 
	
	void minimize(const Pair& init_f);

/*
	void CountReapeted(std::vector<int> indices){
		sort (indices.begin(), indices.end());
		
		int c=0;
		for(int s=0;s<indices.size();s++){
			for(int i=s+1;i<indices.size();i++){
				if(indices[s] == indices[i]){
					c++;
					s=i;
				}else{
					break;
				}
			}
		}
	
		std::cout<< "Numeros repetidos: "<< c << "\n";
				
	}
*/
	friend class lmdif_functor;
	friend class lmder_functor;
};



struct lmdif_functor : Functor<double>
{
	ICP_plane2plane *icp_pl2pl;
	
	
    lmdif_functor(void *p, int m) : Functor<double>(6,m) {
		icp_pl2pl = (ICP_plane2plane*)p;
	}
				  
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const{
        assert(x.size()==6);
        assert(fvec.size()==values());

		Eigen::Matrix3d R;
		Eigen::Vector3d t(x[0],x[1],x[2]);
		R = Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ())
		  * Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY())
		  * Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX()); 

		Eigen::Vector3d p_mod;
		Eigen::Vector3d p_dat, p_dat_t;
		Eigen::Vector3d res;
		Eigen::Matrix3d S;
		Eigen::Vector3d temp;

		int k=0;
		for (int i=0; i<values(); i++)
		{
            if( i < icp_pl2pl->tmp_idx_mod->size()){
				p_mod(0) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_idx_mod)[i]].x;
				p_mod(1) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_idx_mod)[i]].y;
				p_mod(2) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_idx_mod)[i]].z;
        
				p_dat(0) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_idx_dat)[i]].x;
				p_dat(1) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_idx_dat)[i]].y;
				p_dat(2) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_idx_dat)[i]].z;
				
				p_dat_t = R*p_dat + t;
       
				// Estimate the distance (cost function)
				// The last coordiante is still guaranteed to be set to 1.0
				//Eigen::Vector3d res(p_mod[0] - p_dat[0], p_mod[1] - p_dat[1], p_mod[2] - p_dat[2]);
				res << p_dat_t[0] - p_mod[0], 
					   p_dat_t[1] - p_mod[1], 
					   p_dat_t[2] - p_mod[2];
				
				S = R*icp_pl2pl->cov_dat[i];
				S = icp_pl2pl->cov_mod[i] + S * R.transpose();
				temp = S.inverse()*res;
				//Eigen::Vector3d temp = icp_pl2pl->mahalanobis[i]*res;
			
				fvec[i] = double(res.transpose() * temp);
			}else{
				p_mod(0) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_initf)[k].first].x;
				p_mod(1) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_initf)[k].first].y;
				p_mod(2) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_initf)[k].first].z;
        
				p_dat(0) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_initf)[k].second].x;
				p_dat(1) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_initf)[k].second].y;
				p_dat(2) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_initf)[k].second].z;
    		
				p_dat_t = R*p_dat + t;
       
				// Estimate the distance (cost function)
				// The last coordiante is still guaranteed to be set to 1.0
				//Eigen::Vector3d res(p_mod[0] - p_dat[0], p_mod[1] - p_dat[1], p_mod[2] - p_dat[2]);
				res<<p_dat_t[0] - p_mod[0], 
					 p_dat_t[1] - p_mod[1], 
					 p_dat_t[2] - p_mod[2];
				
				fvec[i] = double(res.transpose() * res);
				k++;	
			}
        }
        return 0;
    }
};


struct lmder_functor : lmdif_functor
{
    lmder_functor(void *p, int m) :  lmdif_functor(p,m){}
			
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const {
     
		Eigen::Matrix3d R;
		Eigen::Vector3d t(x(0),x(1),x(2));
		R = Eigen::AngleAxisd(x(5), Eigen::Vector3d::UnitZ())
		  * Eigen::AngleAxisd(x(4), Eigen::Vector3d::UnitY())
		  * Eigen::AngleAxisd(x(3), Eigen::Vector3d::UnitX()); 
	 
		Eigen::Vector3d p_mod, p_dat, p_dat_t, res;
		Eigen::Matrix3d S, S_1;
		
		int k=0;
        for (int i = 0; i < values(); i++)
        {	
			if( i < icp_pl2pl->tmp_idx_mod->size()){
				p_mod(0) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_idx_mod)[i]].x;
				p_mod(1) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_idx_mod)[i]].y;
				p_mod(2) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_idx_mod)[i]].z;
        
				p_dat(0) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_idx_dat)[i]].x;
				p_dat(1) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_idx_dat)[i]].y;
				p_dat(2) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_idx_dat)[i]].z;
       
				p_dat_t = R*p_dat + t;				
				
				// The last coordiante is still guaranteed to be set to 1.0
	
				res = p_dat_t - p_mod;
				
				S = R*icp_pl2pl->cov_dat[i];
				S = icp_pl2pl->cov_mod[i] + S * R.transpose();
				S_1 = S.inverse();
				//Eigen::Vector3d temp = icp_pl2pl->mahalanobis[i]*res;
				
				//res=(S+S.transpose())*res;
				res=S_1*res;
				
				
				// Increment translation gradient
				//// g_t+= 2*M*res/num_matches
				//g_t =  S.inverse() * res;
				fjac(i,0) = res(0);
				fjac(i,1) = res(1);
				fjac(i,2) = res(2);
				
				computeRDerivative(x, p_dat, res, fjac(i,3), fjac(i,4), fjac(i,5));
			}else{
				p_mod(0) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_initf)[k].first].x;
				p_mod(1) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_initf)[k].first].y;
				p_mod(2) = icp_pl2pl->cloud_m->points[(*icp_pl2pl->tmp_initf)[k].first].z;
        
				p_dat(0) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_initf)[k].second].x;
				p_dat(1) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_initf)[k].second].y;
				p_dat(2) = icp_pl2pl->cloud_d->points[(*icp_pl2pl->tmp_initf)[k].second].z;
					
				p_dat_t = R*p_dat + t;				
			
				res = p_dat_t - p_mod;
				
				// Estimate the distance (cost function)
				fjac(i,0) = res(0);
				fjac(i,1) = res(1);
				fjac(i,2) = res(2);
					
				computeRDerivative(x, p_dat ,res, fjac(i,3), fjac(i,4), fjac(i,5));
				k++;	
			}                      
        }
        return 0;
    }
     
	void computeRDerivative(const Eigen::VectorXd &x,const Eigen::Vector3d &p_dat,const Eigen::Vector3d &res, double &fjacx, double &fjacy, double &fjacz) const{
		Eigen::Matrix3d dR_dPhi;
		Eigen::Matrix3d dR_dTheta;
		Eigen::Matrix3d dR_dPsi;
		
		Eigen::Matrix3d Rdf = res*p_dat.transpose();
		
		// phi=ax theta=ay psi=az
		double phi = x(3), theta = x(4), psi = x(5);
  
		double cphi = cos(phi), sphi = sin(phi);
		double ctheta = cos(theta), stheta = sin(theta);
		double cpsi = cos(psi), spsi = sin(psi);
      
		dR_dPhi(0,0) = 0.;
		dR_dPhi(1,0) = 0.;
		dR_dPhi(2,0) = 0.;

		dR_dPhi(0,1) = sphi*spsi + cphi*cpsi*stheta;
		dR_dPhi(1,1) = -cpsi*sphi + cphi*spsi*stheta;
		dR_dPhi(2,1) = cphi*ctheta;

		dR_dPhi(0,2) = cphi*spsi - cpsi*sphi*stheta;
		dR_dPhi(1,2) = -cphi*cpsi - sphi*spsi*stheta;
		dR_dPhi(2,2) = -ctheta*sphi;

		dR_dTheta(0,0) = -cpsi*stheta;
		dR_dTheta(1,0) = -spsi*stheta;
		dR_dTheta(2,0) = -ctheta;

		dR_dTheta(0,1) = cpsi*ctheta*sphi;
		dR_dTheta(1,1) = ctheta*sphi*spsi;
		dR_dTheta(2,1) = -sphi*stheta;

		dR_dTheta(0,2) = cphi*cpsi*ctheta;
		dR_dTheta(1,2) = cphi*ctheta*spsi;
		dR_dTheta(2,2) = -cphi*stheta;

		dR_dPsi(0,0) = -ctheta*spsi;
		dR_dPsi(1,0) = cpsi*ctheta;
		dR_dPsi(2,0) = 0.;

		dR_dPsi(0,1) = -cphi*cpsi - sphi*spsi*stheta;
		dR_dPsi(1,1) = -cphi*spsi + cpsi*sphi*stheta;
		dR_dPsi(2,1) = 0.;

		dR_dPsi(0,2) = cpsi*sphi - cphi*spsi*stheta;
		dR_dPsi(1,2) = sphi*spsi + cphi*cpsi*stheta;
		dR_dPsi(2,2) = 0.;
      
		// set d/d_rx = tr(dR_dPhi'*gsl_temp_mat_r) [= <dR_dPhi, gsl_temp_mat_r>]
		//fjacx = (dR_dPhi*p_dat).transpose()*res;
		fjacx = (dR_dPhi.transpose()*Rdf).trace();
		// set d/d_ry = tr(dR_dTheta'*gsl_temp_mat_r) = [<dR_dTheta, gsl_temp_mat_r>]
		//fjacy = (dR_dTheta*p_dat).transpose()*res;
		fjacy = (dR_dTheta.transpose()*Rdf).trace();
		// set d/d_rz = tr(dR_dPsi'*gsl_temp_mat_r) = [<dR_dPsi, gsl_temp_mat_r>]
		//fjacz = (dR_dPsi*p_dat).transpose()*res;
		fjacz = (dR_dPsi.transpose()*Rdf).trace();
	}
};



#include "imp/ICP_plane2plane.hpp"
#endif

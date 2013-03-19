/*
 *  RotQuaUtilities.cpp
 *  
 *  Created by Pedro Vieira on 7/22/11.
 *
 */
 
 
#include <stdlib.h>
#include "RotQuaUtilities.h"


using namespace Eigen;


Vector4d RtoQuaternions_eig(Matrix3d R){

	Vector4d q;

	q(3) =  0.5 * sqrt(1 + R(0,0) + R(1,1) + R(2,2));
	q(0) = (R(1,2) - R(2,1)) / (4*q(3));
	q(1) = (R(2,0) - R(0,2)) / (4*q(3));
	q(2) = (R(0,1) - R(1,0)) / (4*q(3));
	
	return q;
	
}

double *RtoQuaternions_d(Matrix3d R){
	
	double *q;
	
	q = (double*)malloc (4*sizeof(double));
	
	if(q==NULL){
		return NULL;
	}
	
	q[3] =  0.5 * sqrt(1 + R(0,0) + R(1,1) + R(2,2));
	q[0] = (R(1,2) - R(2,1)) / (4*q[3]);
	q[1] = (R(2,0) - R(0,2)) / (4*q[3]);
	q[2] = (R(0,1) - R(1,0)) / (4*q[3]);	
	
	return q;
}
double *RtoQuaternions_d(double R11, double R12, double R13, double R21, double R22, double R23, double R31, double R32 , double R33){

	double *q;
	
	q = (double*)malloc (4*sizeof(double));
	
	if(q==NULL){
		return NULL;
	}
		
	q[3] =  0.5 * sqrt(1 + R11 + R22 + R33);
	q[0] = (R23 - R32) / (4*q[3]);
	q[1] = (R31 - R13) / (4*q[3]);
	q[2] = (R12 - R21) / (4*q[3]);	
	
	return q;
}


void RtoAngles(Matrix3d R, double &ax, double &ay, double &az){
	
	double q[4];
	const double PI = 3.14159265;
	
	q[3] =  0.5 * sqrt(1 + R(0,0) + R(1,1) + R(2,2));
	q[0] = (R(1,2) - R(2,1)) / (4*q[3]);
	q[1] = (R(2,0) - R(0,2)) / (4*q[3]);
	q[2] = (R(0,1) - R(1,0)) / (4*q[3]);
	
	ax = atan2( 2 * (q[1]*q[2]-q[0]*q[3]) , (1-2*(q[0]*q[0]+q[1]*q[1])) );
	ax = ax*180/PI;
	
	ay = asin( -2*(q[3]*q[1]+q[2]*q[0]));
	ay = ay*180/PI;
	
	az = atan2( 2* (q[0]*q[1] - q[2]*q[3]) , (1-2*(q[1]*q[1]+q[2]*q[2])) );
	az = az*180/PI; 
	
}

Vector3d RtoAngles(Matrix3d R){
	
	double q[4];
	Vector3d angles; //[x y z]
	const double PI = 3.14159265;
	
	q[3] =  0.5 * sqrt(1 + R(0,0) + R(1,1) + R(2,2));
	q[0] = (R(1,2) - R(2,1)) / (4*q[3]);
	q[1] = (R(2,0) - R(0,2)) / (4*q[3]);
	q[2] = (R(0,1) - R(1,0)) / (4*q[3]);
	
	angles(0) = atan2( 2 * (q[1]*q[2]-q[0]*q[3]) , (1-2*(q[0]*q[0]+q[1]*q[1])) );
	angles(0) = angles(0)*180/PI;
	
	angles(1) = asin( -2*(q[3]*q[1]+q[2]*q[0]));
	angles(1) = angles(1)*180/PI;
	
	angles(2) = atan2( 2* (q[0]*q[1] - q[2]*q[3]) , (1-2*(q[1]*q[1]+q[2]*q[2])) );
	angles(2) = angles(2)*180/PI; 
	
	return angles;
}


Matrix3d AnglesToR(double ax, double ay, double az){
	
	Matrix3d R;
	const double PI = 3.14159265; 
	
	R << cos(az*PI/180)*cos(ay*PI/180) ,												//R11
		-sin(az*PI/180)*cos(ax*PI/180) + cos(az*PI/180)*sin(ay*PI/180)*sin(ax*PI/180) ,	//R12	 
	     sin(az*PI/180)*sin(ax*PI/180) + cos(az*PI/180)*sin(ay*PI/180)*cos(ax*PI/180) ,	//R13
	     sin(az*PI/180)*cos(ay*PI/180) ,												//R21
	     cos(az*PI/180)*cos(ax*PI/180) + sin(az*PI/180)*sin(ay*PI/180)*sin(ax*PI/180) ,	//R22
	    -cos(az*PI/180)*sin(ax*PI/180) + sin(az*PI/180)*sin(ay*PI/180)*cos(ax*PI/180) ,	//R23
	    -sin(ay*PI/180) ,																//R31
	     cos(ay*PI/180)*sin(ax*PI/180) ,												//R32
	     cos(ay*PI/180)*cos(ax*PI/180) ;												//R33
	return R;
}
Matrix3f AnglesToR(float ax, float ay, float az){
	
	Matrix3f R;
	const float PI = 3.14159265; 
	
	R << cos(az*PI/180.0)*cos(ay*PI/180.0) ,														//R11
		-sin(az*PI/180.0)*cos(ax*PI/180.0) + cos(az*PI/180.0)*sin(ay*PI/180.0)*sin(ax*PI/180.0) ,	//R12	 
	     sin(az*PI/180.0)*sin(ax*PI/180.0) + cos(az*PI/180.0)*sin(ay*PI/180.0)*cos(ax*PI/180.0) ,	//R13
	     sin(az*PI/180.0)*cos(ay*PI/180.0) ,													    //R21
	     cos(az*PI/180.0)*cos(ax*PI/180.0) + sin(az*PI/180.0)*sin(ay*PI/180.0)*sin(ax*PI/180.0) ,	//R22
	    -cos(az*PI/180.0)*sin(ax*PI/180.0) + sin(az*PI/180.0)*sin(ay*PI/180.0)*cos(ax*PI/180.0) ,	//R23
	    -sin(ay*PI/180.0) ,																			//R31
	     cos(ay*PI/180.0)*sin(ax*PI/180.0) ,														//R32
	     cos(ay*PI/180.0)*cos(ax*PI/180.0) ;														//R33
	return R;
}


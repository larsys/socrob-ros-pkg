/*
 *  surfels_rgbd.cpp
 *  
 *
 *  Created by Pedro Vieira on 01/17/11.
 * 
 * This file implements the RGBD model based on Surfels
 */
 
#include <iostream>
#include "surfels_rgbd.h"	

bool SurfelsRGBDModel::addNewView(const PCloud &cloud_d, Eigen::Matrix4d T_world){
	
	
	pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree;
	kdtree.setInputCloud (cloud_d);
	std::vector<int> indice(1);
	std::vector<float> distance(1);
	
	std::vector<int> covered_pixels(cloud_d->size(),0);
	
	Eigen::Vector4d auxp;
	Eigen::Vector4d surfel_normal;
	Eigen::Vector4d cloudd_normal;
	
	pcl::PointXYZRGB surfel_pt;
	
	int ptoremove=0;
	pcl::PointSurfel remove_aux;
	
	
	//update surfels
	for(unsigned int i=0; i<m_surfels.size()-ptoremove; i++){
			
		auxp << m_surfels[i].x, m_surfels[i].y, m_surfels[i].z, 1.0;
		//std::cout << auxp.transpose() <<  "\n";
		auxp = T_world.inverse()*auxp;
		
		surfel_pt.x = auxp(0);
		surfel_pt.y = auxp(1);
		surfel_pt.z = auxp(2);
	
		//std::cout << surfel_pt.x  << " "  << surfel_pt.y  << " " << surfel_pt.z <<  "\n\n";
		
		//checks if there is any point in range
		if (kdtree.nearestKSearch (surfel_pt, 1, indice, distance) != 1) continue;
		
		//std::cout << "depois tree " << distance[0] << " " << mindist*mindist << " " << mindist <<"\n";
		
		if (distance[0]>mindist*mindist){

			if (m_surfels[i].confidence < 2)
			{	
				remove_aux=m_surfels[i];
				m_surfels[i]=m_surfels[m_surfels.size()-1-ptoremove];
				m_surfels[m_surfels.size()-1-ptoremove]=remove_aux;
				ptoremove++;
				i--;	
			}
			continue;
		}
		
		//checks normal angles
		surfel_normal << m_surfels[i].normal_x,m_surfels[i].normal_y, m_surfels[i].normal_z, 1.0;
		cloudd_normal = computeNormal(indice[0],&kdtree,cloud_d);
		surfel_normal =T_world.inverse()*surfel_normal;
		normalize(surfel_normal);

		
		float normal_angle = acos(cloudd_normal(0)*surfel_normal(0) + cloudd_normal(1)*surfel_normal(1) + cloudd_normal(2)*surfel_normal(2));
		
		//std::cout << normal_angle*180.0/3.14159265 <<  " " << update_max_normal_angle << "\n";
		
		if (normal_angle > (update_max_normal_angle*3.14159265/180.0))
		{
			// Removal check. If a surfel has a different normal and is closer to the camera
			// than the new scan, remove it.
			if ((surfel_pt.z > cloud_d->points[indice[0]].z) && (m_surfels[i].confidence < 2))
			{	
				remove_aux=m_surfels[i];
				m_surfels[i]=m_surfels[m_surfels.size()-1-ptoremove];
				m_surfels[m_surfels.size()-1-ptoremove]=remove_aux;
				ptoremove++;
				i--;	
			}else
				covered_pixels[indice[0]] = 1;
			continue;
		}
		
		
		//checks distance
		// If existing surfel is far from new depth value:
		// - If existing one had a worst point of view, and was seen only once, remove it.
		// - Otherwise do not include the new one.
		if (std::fabs(surfel_pt.z - cloud_d->points[indice[0]].z) > update_max_dist){
			
			if (m_surfels[i].confidence < 2){
			  remove_aux=m_surfels[i];
			  m_surfels[i]=m_surfels[m_surfels.size()-1-ptoremove];
			  m_surfels[m_surfels.size()-1-ptoremove]=remove_aux;
			  ptoremove++;
			  i--;
			}
			else
			  covered_pixels[indice[0]] = 1;
			continue;
		}
		
		
		
		
		//update surfel
		m_surfels[i].x = (m_surfels[i].x + cloud_d->points[indice[0]].x*T_world(0,0) + cloud_d->points[indice[0]].y*T_world(0,1) + cloud_d->points[indice[0]].z*T_world(0,2) + T_world(0,3))/2;
		m_surfels[i].y = (m_surfels[i].y + cloud_d->points[indice[0]].x*T_world(1,0) + cloud_d->points[indice[0]].y*T_world(1,1) + cloud_d->points[indice[0]].z*T_world(1,2) + T_world(1,3))/2;
		m_surfels[i].z = (m_surfels[i].z + cloud_d->points[indice[0]].x*T_world(2,0) + cloud_d->points[indice[0]].y*T_world(2,1) + cloud_d->points[indice[0]].z*T_world(2,2) + T_world(2,3))/2;
	
		Eigen::Vector4d cloudd_normal2 = T_world*cloudd_normal;
		normalize(cloudd_normal2);
		
		m_surfels[i].normal_x = (m_surfels[i].normal_x + cloudd_normal2(0))/2;
		m_surfels[i].normal_y = (m_surfels[i].normal_y + cloudd_normal2(1))/2;
		m_surfels[i].normal_z = (m_surfels[i].normal_z + cloudd_normal2(2))/2;
	
		unsigned char rgba_ptr[3]; 
		rgba_ptr[0] = m_surfels[i].rgba >> 24;		
		rgba_ptr[1] = m_surfels[i].rgba >> 16; 		
		rgba_ptr[2] = m_surfels[i].rgba >> 8; 		
		rgba_ptr[0] = (int)( (int)rgba_ptr[0] + cloud_d->points[indice[0]].r )/2; 
		rgba_ptr[1] = (int)( (int)rgba_ptr[1] + cloud_d->points[indice[0]].g )/2;
		rgba_ptr[2] = (int)( (int)rgba_ptr[2] + cloud_d->points[indice[0]].b )/2;
		
		m_surfels[i].rgba = (rgba_ptr[0] <<  24) | (rgba_ptr[1] << 16) | (rgba_ptr[2] << 8);
		 
		m_surfels[i].radius = std::min((double)m_surfels[i].radius, (1.0/1.414213562) * cloud_d->points[indice[0]].z / (fx * cloudd_normal(2)));
		m_surfels[i].confidence = m_surfels[i].confidence + 1.0;
		covered_pixels[indice[0]] = 1;
	}
	
	//Add new surfels
	// Surfel addition
    for (unsigned int i = 0; i < cloud_d->size(); i++) 
    {
      if (isnan(cloud_d->points[i].z) || covered_pixels[i]==1) continue;
      
		pcl::PointSurfel surfel;
      
		surfel.x = cloud_d->points[i].x*T_world(0,0) + cloud_d->points[i].y*T_world(0,1) + cloud_d->points[i].z*T_world(0,2) + T_world(0,3);
		surfel.y = cloud_d->points[i].x*T_world(1,0) + cloud_d->points[i].y*T_world(1,1) + cloud_d->points[i].z*T_world(1,2) + T_world(1,3);
		surfel.z = cloud_d->points[i].x*T_world(2,0) + cloud_d->points[i].y*T_world(2,1) + cloud_d->points[i].z*T_world(2,2) + T_world(2,3);
	
		cloudd_normal = computeNormal(i,&kdtree,cloud_d);
		cloudd_normal = T_world*cloudd_normal;
		normalize(cloudd_normal);
		surfel.normal_x = cloudd_normal(0);
		surfel.normal_y = cloudd_normal(1);
		surfel.normal_z = cloudd_normal(2);
		 
		surfel.rgba = (cloud_d->points[i].r <<  24) | (cloud_d->points[i].g << 16) | (cloud_d->points[i].b << 8);
		 
		double camera_normal_z = std::max((float)cloudd_normal(2), 0.3f);
		surfel.radius = (2.0/1.414213562) * cloud_d->points[i].z / (fx * camera_normal_z);
		
		surfel.confidence = 1.0;
      
		m_surfels.push_back(surfel);
	}
	
	//remove Surfels
	for(int k=0;k<ptoremove;k++){
		m_surfels.pop_back();
	}
  	
}


void SurfelsRGBDModel::computeMesh(){
	std::cout<< "\n" << m_surfels.size()<<"\n";
}

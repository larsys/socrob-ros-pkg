#include "MarkersConfig.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <list>
#include <ros/ros.h>
#include <cstdio>



MarkersConfig::MarkersConfig(float default_val):
msize(__number_of_possible_markers,default_val),
mposition(__number_of_possible_markers,cv::Point3f(0,0,0))
{

}

MarkersConfig::MarkersConfig(std::string path, float default_val)throw(cv::Exception):
    msize(__number_of_possible_markers,default_val),
    mposition(__number_of_possible_markers,cv::Point3f(0,0,0))
{
    std::ifstream file(path.c_str());
    if(!file) throw  cv::Exception(9100, "Could not open file:"+path,"::constructor::",__FILE__,__LINE__);

    std::list<int> readIDs;
    char line[1024];
    while(!file.eof()){
        file.getline(line,1024);
        int id;
        float size,m_x,m_y,m_z;
        switch (sscanf(line,"%d = %f (%f,%f,%f)",&id,&size,&m_x,&m_y,&m_z)){
        case 2:
            if(this->msize.at(id) != default_val){
                ROS_WARN_STREAM("Warning: ID Marker " << id << " is " << size << " instead of the previous " << this->msize.at(id));
            }

            this->msize.at(id)=size;
            readIDs.push_back(id);

            //debug:
            ROS_INFO_STREAM(id << " = " << size );
            break;
        case 5:
            if(this->msize.at(id) != default_val){
                std::cout << "Warning: ID Marker " << id << " is " << size << " instead of the previous " << this->msize.at(id) << std::endl;
                std::cout << "Probably you want to review the file configuration: " << std::endl;
            }
            this->msize.at(id)=size;
            this->mposition.at(id)= cv::Point3f(m_x,m_y,m_z);
            readIDs.push_back(id);
            //debug:

            ROS_INFO_STREAM(id << " = " << size
                             << " (" << m_x
                             << ',' << m_y
                             << ',' << m_z << ')');
            break;

        }

    }
}

void MarkersConfig::parse_from_file(std::string path, float default_val)throw(cv::Exception)
{
    msize = std::vector<float>(__number_of_possible_markers,default_val);
    mposition = std::vector<cv::Point3f>(__number_of_possible_markers,cv::Point3f(0,0,0));

    std::ifstream file(path.c_str());
    if(!file) throw  cv::Exception(9100, "Could not open file:"+path,"::constructor::",__FILE__,__LINE__);

    std::list<int> readIDs;
    char line[1024];
    while(!file.eof()){
        file.getline(line,1024);
        int id;
        float size,m_x,m_y,m_z;
        switch (sscanf(line,"%d = %f (%f,%f,%f)",&id,&size,&m_x,&m_y,&m_z)){
        case 2:
            if(this->msize.at(id) != default_val){
                ROS_WARN_STREAM("Warning: ID Marker " << id << " is " << size << " instead of the previous " << this->msize.at(id));
            }

            this->msize.at(id)=size;
            readIDs.push_back(id);

            //debug:
            ROS_INFO_STREAM(id << " = " << size );
            break;
        case 5:
            if(this->msize.at(id) != default_val){
                ROS_WARN_STREAM("Warning: ID Marker " << id << " is " << size << " instead of the previous " << this->msize.at(id));
            }
            this->msize.at(id)=size;
            this->mposition.at(id)= cv::Point3f(m_x,m_y,m_z);
            readIDs.push_back(id);
            //debug:

            ROS_INFO_STREAM(id << " = " << size
                             << " (" << m_x
                             << ',' << m_y
                             << ',' << m_z << ')');
            break;

        }

    }
}

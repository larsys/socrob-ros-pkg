#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <opencv2/core/core.hpp>

#define __number_of_possible_markers 1024



class MarkersConfig
{
public:

    MarkersConfig(float default_val=-1);

    MarkersConfig(std::string path, float default_val=-1)throw(cv::Exception);

    void parse_from_file(std::string path, float default_val=-1)throw(cv::Exception);

    std::vector<float> msize;
    std::vector<cv::Point3f> mposition;

};

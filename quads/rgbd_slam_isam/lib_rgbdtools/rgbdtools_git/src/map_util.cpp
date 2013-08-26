#include "rgbdtools/map_util.h"
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>


namespace rgbdtools {
 

void concatenate_clouds(PointCloudT& map_cloud,KeyframeVector keyframes_)
{
//    PointCloudT::Ptr aggregate_cloud(new PointCloudT());
//    aggregate_cloud->header.frame_id = keyframes_[0].header.frame_id;
//    aggregate_cloud->sensor_origin_[3] = 1;
//    // aggregate all frames into single cloud
//    for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
//    {
//        const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
//        PointCloudT cloud;
//        keyframe.constructDensePointCloud(cloud, 10, 1);

//        PointCloudT cloud_tf;
//        pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
//        cloud_tf.header.frame_id = keyframes_[0].header.frame_id;
////        pcl::concatenateFields(*aggregate_cloud,cloud_tf,*aggregate_cloud);
//        *aggregate_cloud += cloud_tf;


//    }
//    PointT min,max;
//    pcl::getMinMax3D(*aggregate_cloud,min,max);


}

void floatMatrixToUintMatrix(
  const cv::Mat& mat_in, 
  cv::Mat& mat_out, 
  float scale)
{
  if (scale == 0)
  {
    // normalize the matrix
    float max_val = 0;
    
    for (int u = 0; u < mat_in.cols; ++u)
    for (int v = 0; v < mat_in.rows; ++v)   
    {
      float val_in = mat_in.at<float>(v, u);
      if (val_in > max_val) max_val = val_in;
    }
  
    scale = 255.0 / max_val;
  }
   
  mat_out = cv::Mat::zeros(mat_in.size(), CV_8UC1);
  for (int u = 0; u < mat_in.cols; ++u)
  for (int v = 0; v < mat_in.rows; ++v)   
  {
    float val_in = mat_in.at<float>(v, u) ;
    uint8_t& val_out = mat_out.at<uint8_t>(v, u); 

    val_out = val_in * scale;
  }
}

void thresholdMatrix(
  const cv::Mat& mat_in, 
  cv::Mat& mat_out,
  int threshold)
{
  mat_out = cv::Mat::zeros(mat_in.size(), CV_8UC1);
  
  for (int u = 0; u < mat_in.cols; ++u)
  for (int v = 0; v < mat_in.rows; ++v)   
  {
    uint16_t val_in = mat_in.at<uint16_t>(v, u) ;
    uint8_t& val_out = mat_out.at<uint8_t>(v, u); 
    
    if (val_in >= threshold) val_out = 1;
  }
}

void buildExpectedPhiHistorgtam(
  cv::Mat& histogram,
  double degrees_per_bin,
  double stdev)
{
  int n_bins = (int)(360.0 / degrees_per_bin);
  histogram = cv::Mat::zeros(1, n_bins, CV_32FC1);

  double s = stdev  / degrees_per_bin;

  double mean[4];
  mean[0] =   0.0 / degrees_per_bin;
  mean[1] =  90.0 / degrees_per_bin;
  mean[2] = 180.0 / degrees_per_bin;
  mean[3] = 270.0 / degrees_per_bin;

  double a = 1.0 / (s * sqrt(2.0 * M_PI));
  double b = 2.0 * s * s; 

  for (int u = 0; u < n_bins; ++u)
  {
    float& bin = histogram.at<float>(0, u);

    // accumulate 4 gaussians
    for (int g = 0; g < 4; g++)
    {
      int x = u - mean[g];
  
      // wrap around to closer distance
      if (x < -n_bins/2) x += n_bins;
      if (x >= n_bins/2) x -= n_bins;

      float r = a * exp(-x*x / b);

      bin += r;
    }
  }

  normalizeHistogram(histogram);
}  

void buildPhiHistogram(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud,
  cv::Mat& histogram,
  double degrees_per_bin)
{
  int phi_bins = (int)(360.0 / degrees_per_bin);
  histogram = cv::Mat::zeros(1, phi_bins, CV_32FC1);
  
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& p = cloud.points[i]; 

    double nx = p.normal_x;
    double ny = p.normal_y;
    double nz = p.normal_z;

    if (isnan(nx) || isnan(ny) || isnan(nz)) continue;

    double r = sqrt(nx*nx + ny*ny + nz*nz);
    double theta = acos(nz/r);
    double phi   = atan2(ny, nx);

    double phi_deg   = phi   * 180.0 / M_PI;
    double theta_deg = theta * 180.0 / M_PI; 

    // normalize phi to [0, 360)
    if (phi_deg < 0.0) phi_deg += 360.0;

    // only consider points which are close to vertical
    if (std::abs(90-theta_deg) > 3.0) continue; 

    int idx_phi = (int)(phi_deg / degrees_per_bin);

    float& bin = histogram.at<float>(0, idx_phi);
    bin = bin + 1.0; 
  }

  normalizeHistogram(histogram);
}

void shiftHistogram(
  const cv::Mat& hist_in,
  cv::Mat& hist_out,
  int bins)
{
  hist_out = cv::Mat(hist_in.size(), CV_32FC1);
  int w = hist_in.cols;
  for (int u = 0; u < w; ++u)
  {
    int u_shifted = (u + bins) % w;

    hist_out.at<float>(0, u_shifted) = hist_in.at<float>(0, u);
  } 
}

void normalizeHistogram(cv::Mat& histogram)
{
  float sum = 0;

  for (int u = 0; u < histogram.cols; ++u)
    sum += histogram.at<float>(0,u);

  histogram = histogram / sum;
}

bool alignHistogram(
  const cv::Mat& hist,
  const cv::Mat& hist_exp,
  double hist_resolution,
  double& best_angle)
{
  // check diff
  int best_i = 0;
  double best_diff = 9999999999;

  for (int i = 0; i < 90.0 / hist_resolution; ++i)
  {
    cv::Mat hist_shifted;
    shiftHistogram(hist, hist_shifted, i);

    double diff = cv::compareHist(hist_shifted, hist_exp, CV_COMP_BHATTACHARYYA);
    if (std::abs(diff) < best_diff)
    {
      best_diff = std::abs(diff);
      best_i = i;
    }
  }

  best_angle = best_i * hist_resolution;

  cv::Mat hist_best;
  shiftHistogram(hist, hist_best, best_i);
  cv::Mat hist_best_img;
  createImageFromHistogram(hist_best, hist_best_img);
  cv::imshow("hist_best_img", hist_best_img);

  return true;
}

void create8bitHistogram(
  const cv::Mat& histogram,
  cv::Mat& histogram_norm)
{
  // find max value
  double min_val, max_val;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(histogram, &min_val, &max_val, &minLoc, &maxLoc);

  // rescale so that max = 255, for visualization purposes
  cv::Mat temp = histogram.clone();
  temp = temp * 255.0 / max_val;

  // convert to uint
  histogram_norm = cv::Mat::zeros(histogram.size(), CV_8UC1); 
  temp.convertTo(histogram_norm, CV_8UC1); 
}

void createImageFromHistogram(
  const cv::Mat& histogram,
  cv::Mat& image)
{
  // normalize the histogram in range 0 - 255
  cv::Mat hist_norm;
  create8bitHistogram(histogram, hist_norm);

  image = cv::Mat::zeros(256, histogram.cols, CV_8UC1);
  for (int u = 0; u < histogram.cols; ++u)
  {
    uint8_t val = hist_norm.at<uint8_t>(0, u);
    for (int v = 0; v < val; ++v)
      image.at<uint8_t>(255-v, u) = 255;
  }
}

void create2DProjectionImage(
  const PointCloudT& cloud, 
  cv::Mat& img,
  double min_z,
  double max_z)
{
  // TODO: thses shouldnt be hard-coded
  double resolution = 0.02; // 2cm
  double w = 20.0;
  double h = 20.0;
  double cx = w/2;
  double cy = h/2;
  
  img = cv::Mat::zeros(h/resolution, w/resolution, CV_8UC1);

  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    const PointT& p = cloud.points[i];

    // filter z
    if (isnan(p.z) || p.z >= max_z || p.z < min_z) continue;

    int u = (h - p.x + cx)/resolution;
    int v = (p.y + cy)/resolution;

    uint8_t& bin = img.at<uint8_t>(v, u);
    if(bin < 255) bin++;
  }
}

void filterCloudByHeight(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out,
  double min_z,
  double max_z)
{
  for (unsigned int i = 0; i < cloud_in.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& p = cloud_in.points[i]; 
    
    if (p.z >= min_z && p.z < max_z)
      cloud_out.push_back(p); 
  }
}

/*
void prepareFeaturesForRANSAC(KeyframeVector& keyframes)
{
  double init_surf_threshold = 400.0;
  double min_surf_threshold = 25;
  int n_keypoints = 400;

  printf("preparing SURF features for RANSAC associations...\n");  

  cv::SurfDescriptorExtractor extractor;
 
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    RGBDKeyframe& keyframe = keyframes[kf_idx];

    double surf_threshold = init_surf_threshold;

    while (surf_threshold >= min_surf_threshold)
    {
      cv::SurfFeatureDetector detector(surf_threshold);
      keyframe.keypoints.clear();
      detector.detect(keyframe.rgb_img, keyframe.keypoints);

      printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n", 
        (int)kf_idx, (int)keyframes.size(), 
        (int)keyframe.keypoints.size(), surf_threshold); 

      if ((int)keyframe.keypoints.size() < n_keypoints)
        surf_threshold /= 2.0;
      else break;
    }

    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
    keyframe.computeDistributions();
  }
}
*/

void pairwiseMatchingRANSAC(
  const RGBDFrame& frame_a, const RGBDFrame& frame_b,
  double max_eucl_dist_sq, 
  double max_desc_dist,
  double sufficient_inlier_ratio,
  int max_ransac_iterations,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // params
  bool use_ratio_test = true;
  float max_desc_ratio = 0.75;

  // constants
  int min_sample_size = 3;

  cv::FlannBasedMatcher matcher;          // for SURF
  TransformationEstimationSVD svd;

  std::vector<cv::DMatch> candidate_matches;

  // **** build candidate matches ***********************************
  // assumes detectors and distributions are computed
  // establish all matches from b to a

  if (use_ratio_test)
  {
    std::vector<std::vector<cv::DMatch> > all_matches2;
    
    matcher.knnMatch(
      frame_b.descriptors, frame_a.descriptors, all_matches2, 2);

    for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
    {
      const cv::DMatch& match1 = all_matches2[m_idx][0];
      const cv::DMatch& match2 = all_matches2[m_idx][1];
      
      double ratio =  match1.distance / match2.distance;
      
      // remove bad matches - ratio test, valid keypoints
      if (ratio < max_desc_ratio)
      {
        int idx_b = match1.queryIdx;
        int idx_a = match1.trainIdx; 

        if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
          candidate_matches.push_back(match1);
      }
    }
  }
  else
  {
    matcher.match(
      frame_b.descriptors, frame_a.descriptors, all_matches);

    for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
    {
      const cv::DMatch& match = all_matches[m_idx];

      // remove bad matches - descriptor distance, valid keypoints
      if (match.distance < max_desc_dist)
      {      
        int idx_b = match.queryIdx;
        int idx_a = match.trainIdx; 
        
        if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
          candidate_matches.push_back(match);
      }
    }
  }

  int size = candidate_matches.size();
  //printf("size: %d\n", size);
  
  if (size < min_sample_size) return;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(size);
  features_b.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }

  // **** main RANSAC loop ****************************************
  
  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  
  for (int iteration = 0; iteration < max_ransac_iterations; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];

      float dist_sq = distEuclideanSq(p_a, p_b);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        svd.estimateRigidTransformation(
          features_b, inlier_idx,
          features_a, inlier_idx,
          transformation);
        pcl::transformPointCloud(features_b, features_b_tf, transformation);
      }
    }
    
    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }
}

void getRandomIndices(
  int k, int n, IntVector& output)
{
  while ((int)output.size() < k)
  {
    int random_number = rand() % n;
    bool duplicate = false;    

    for (unsigned int i = 0; i < output.size(); ++i)
    {
      if (output[i] == random_number)
      {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      output.push_back(random_number);
  }
}

void get3RandomIndices(
  int n, std::set<int>& mask, IntVector& output)
{
//  int key;
  
//  for(u_int i=0;i<999999;i++)
//  {
//    output.clear();
//    getRandomIndices(3, n, output);
    
//    // calculate a key based on the 3 random indices
//    key = output[0] * n * n + output[1] * n + output[2];
           
//    //printf("%d %d %d\n", output[0], output[1], output[2]);
    
//    // value not present in mask
//    if (mask.find(key) == mask.end())
//      break;
//  }
  getRandomIndices(3, n, output);

//  mask.insert(key);
}

double distEuclideanSq(const PointFeature& a, const PointFeature& b)
{
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return dx*dx + dy*dy + dz*dz;
}

// a = ground truth, b=meas
void compareAssociationMatrix(
  const cv::Mat& a,
  const cv::Mat& b,
  int& false_pos,
  int& false_neg,
  int& total)
{
  false_pos = 0;
  false_neg = 0;
  total = 0;
  
  // assert both matrices are square, and same size
  assert(a.rows == a.cols);
  assert(b.rows == b.cols);
  assert(a.rows == b.rows);
  
  int size = a.rows;
  
  for (int u = 0; u < size; ++u)
  for (int v = 0; v < size; ++v)
  {
    if (u !=v)
    {
      int val_a = a.at<uint8_t>(v,u);
      int val_b = b.at<uint8_t>(v,u);
    
      if (val_a != 0 && val_b == 0) false_neg++;
      if (val_a == 0 && val_b != 0) false_pos++;
    
      if (val_a != 0) total++;
    }
  }
}

void makeSymmetricOR(cv::Mat mat)
{
  assert(mat.rows == mat.cols);
  int size = mat.rows;
  
  for (int u = 0; u < size; ++u)
  for (int v = 0; v < size; ++v)
  {
    if (mat.at<uint8_t>(v, u) != 0 ||
        mat.at<uint8_t>(u, v) != 0)
    {
      mat.at<uint8_t>(v, u) = 1;
      mat.at<uint8_t>(u, v) = 1;
    }
  }
}

void trainSURFMatcher(
  const KeyframeVector& keyframes,
  cv::FlannBasedMatcher& matcher)
{  
  std::vector<cv::Mat> descriptors_vector;
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  matcher.train();
}

void trainSURFMatcher_Iterative(
  const KeyframeVector& keyframes,u_int min, u_int max,
  cv::FlannBasedMatcher& matcher)
{
  std::vector<cv::Mat> descriptors_vector;

  for (unsigned int kf_idx = min; kf_idx < max; ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  matcher.train();
}

/*
// a = array
// s = array size
// n = number of items
void shuffle(int * a, int s, n)
{
  int i = s - 1;
  int j, temp;
  
  while (i > 0)
  {
    j = rand() % (i + 1);
    temp = a[i];
    a[i] = a[j];
    a[j] = temp;
    i = i - 1;
  }
}*/

} // namespace ccny_rgbd

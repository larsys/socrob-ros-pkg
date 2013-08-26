/**
 *  @file keyframe_graph_detector.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  @section LICENSE
 *
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
  *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rgbdtools/graph/keyframe_graph_detector.h"
#include "rgbdtools/matcher.h"

namespace rgbdtools {

aruco::MarkerDetector MDetector;
KeyframeGraphDetector::KeyframeGraphDetector()
{
    srand(time(NULL));

    MDetector.setThresholdParams( 10.0,7.0);
    MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
    marker_size = 0.2;
    // SURF params
    n_keypoints_ = 600;
    init_surf_threshold_ = 400;
    n_matches_accept = 16; //this or more are needed

    // Pairwise matching params
    pairwise_matching_method_ = PAIRWISE_MATCHING_RANSAC;

    // tree algorithm params
    candidate_method_ = CANDIDATE_GENERATION_SURF_TREE;
    n_candidates_ = 10;
    k_nearest_neighbors_ = 4;

    // matcher params
    pairwise_matcher_index_ = PAIRWISE_MATCHER_KDTREE;
    matcher_use_desc_ratio_test_ = true;
    matcher_max_desc_ratio_ = 1;  // when ratio_test = true
    matcher_max_desc_dist_ = 0.5;    // when ratio_test = false
    n_original_poses = 0;
    // common SAC params
    sac_max_eucl_dist_sq_ = 0.05 * 0.05;
    sac_min_inliers_ = 15;  // this or more are needed
    sac_reestimate_tf_ = false;

    // RANSAC params
    ransac_confidence_ = 0.97;
    ransac_max_iterations_ = 1000;
    ransac_sufficient_inlier_ratio_ = 0.65;

    // output params
    verbose_ = true;     // console output
    sac_save_results_ = true;

    // derived parameters
    log_one_minus_ransac_confidence_ = log(1.0 - ransac_confidence_);

    setOutputPath(/*std::getenv("HOME")*/ "~/SAC");
}

KeyframeGraphDetector::~KeyframeGraphDetector()
{

}
void KeyframeGraphDetector::setMapping(bool mapping)
{
    mapping_ = mapping;
}
void KeyframeGraphDetector::setMaxIterations(bool ransac_max_iterations)
{
    ransac_max_iterations_ = ransac_max_iterations;
}

void KeyframeGraphDetector::setSacMinInliers(int min)
{
    assert(min > 0);
    sac_min_inliers_ = min;
}

void KeyframeGraphDetector::setSACReestimateTf(bool sac_reestimate_tf)
{
    sac_reestimate_tf_ = sac_reestimate_tf;
}

void KeyframeGraphDetector::setMatcherUseDescRatioTest(bool matcher_use_desc_ratio_test)
{
    matcher_use_desc_ratio_test_ = matcher_use_desc_ratio_test;
}

void KeyframeGraphDetector::setMatcherMaxDescRatio(double matcher_max_desc_ratio)
{
    matcher_max_desc_ratio_ = matcher_max_desc_ratio;
}

void KeyframeGraphDetector::setMatcherMaxDescDist(double matcher_max_desc_dist)
{
    matcher_max_desc_dist_ = matcher_max_desc_dist;
}

void KeyframeGraphDetector::setNCandidates(int n_candidates)
{
    n_candidates_ = n_candidates;
}

void KeyframeGraphDetector::setKNearestNeighbors(int k_nearest_neighbors)
{
    k_nearest_neighbors_ = k_nearest_neighbors;
}

void KeyframeGraphDetector::setNKeypoints(int n_keypoints)
{
    n_keypoints_ = n_keypoints;
}

void KeyframeGraphDetector::setOutputPath(const std::string& output_path)
{
    output_path_ = output_path;
    boost::filesystem::create_directories(output_path_);
}

void KeyframeGraphDetector::setSACSaveResults(bool sac_save_results)
{
    sac_save_results_ = sac_save_results;
}

void KeyframeGraphDetector::setVerbose(bool verbose)
{
    verbose_ = verbose;
}

void KeyframeGraphDetector::setCandidateGenerationMethod(
        CandidateGenerationMethod candidate_method)
{
    assert(candidate_method == CANDIDATE_GENERATION_BRUTE_FORCE ||
           candidate_method == CANDIDATE_GENERATION_SURF_TREE);

    candidate_method_ = candidate_method;
}

void KeyframeGraphDetector::setPairwiseMatchingMethod(
        PairwiseMatchingMethod pairwise_matching_method)
{
    assert(pairwise_matching_method == PAIRWISE_MATCHING_BFSAC ||
           pairwise_matching_method == PAIRWISE_MATCHING_RANSAC);

    pairwise_matching_method_ = pairwise_matching_method;
}

void KeyframeGraphDetector::setPairwiseMatcherIndex(
        PairwiseMatcherIndex pairwise_matcher_index)
{
    assert(pairwise_matcher_index == PAIRWISE_MATCHER_LINEAR ||
           pairwise_matcher_index == PAIRWISE_MATCHER_KDTREE);

    pairwise_matcher_index_ = pairwise_matcher_index;
}

void KeyframeGraphDetector::generateKeyframeAssociations(
        KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    //  prepareFeaturesForRANSAC(keyframes);
    prepareFeaturesForRANSAC(keyframes);

    buildAssociationMatrix(keyframes, associations);
}
bool first = true;
void KeyframeGraphDetector::generateKeyframeAssociations_Iterative(
        KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    if(first == true)
    {
        n_original_poses = keyframes.size();
        first = false;
    }
    //  prepareFeaturesForRANSAC(keyframes);
    prepareFeaturesForRANSAC(keyframes);

    buildAssociationMatrix_Iterative(keyframes, associations);
}

void KeyframeGraphDetector::prepareMatchers(
        const KeyframeVector& keyframes)
{
    if(verbose_) printf("training individual keyframe matchers ...\n");
    
    matchers_.clear();
    matchers_.resize(keyframes.size());

    for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
    {
        const RGBDKeyframe& keyframe = keyframes[kf_idx];
        cv::FlannBasedMatcher& matcher = matchers_[kf_idx];

        // build matcher
        cv::Ptr<cv::flann::IndexParams> indexParams;

        if (pairwise_matcher_index_ == PAIRWISE_MATCHER_LINEAR)
            indexParams = new cv::flann::LinearIndexParams();
        else if (pairwise_matcher_index_ == PAIRWISE_MATCHER_KDTREE)
            indexParams = new cv::flann::KDTreeIndexParams();

        cv::Ptr<cv::flann::SearchParams> searchParams = new cv::flann::SearchParams(32);

        matcher = cv::FlannBasedMatcher(indexParams, searchParams);

        // train
        std::vector<cv::Mat> descriptors_vector;
        descriptors_vector.push_back(keyframe.descriptors);
        matcher.add(descriptors_vector);
        matcher.train();
    }
}
void KeyframeGraphDetector::prepareFeaturesForRANSAC_Iterative(u_int kf_idx,
                                                               KeyframeVector& keyframes)
{
    bool upright = true;
    double min_surf_threshold = 25;

    if(verbose_) printf("preparing SURF features for matching...\n");
    cv::SurfDescriptorExtractor extractor;
//    cv::OrbDescriptorExtractor extractor;

    RGBDKeyframe& keyframe = keyframes[kf_idx];

    double surf_threshold = init_surf_threshold_;

    while (surf_threshold >= min_surf_threshold)
    {
        cv::SurfFeatureDetector detector(surf_threshold, 4, 2, true, upright);
        keyframe.keypoints.clear();
        detector.detect(keyframe.rgb_img, keyframe.keypoints);

        if ((int)keyframe.keypoints.size() < n_keypoints_)
        {
            if(verbose_)
                printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n",
                       (int)kf_idx+1, (int)keyframes.size(),
                       (int)keyframe.keypoints.size(), surf_threshold);

            surf_threshold /= 2.0;
        }
        else
        {
            keyframe.keypoints.resize(n_keypoints_);

            if(verbose_)
                printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n",
                       (int)kf_idx+1, (int)keyframes.size(),
                       (int)keyframe.keypoints.size(), surf_threshold);

            break;
        }
    }

    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
    keyframe.computeDistributions(20,20);
}

void KeyframeGraphDetector::prepareFeaturesForRANSAC(
        KeyframeVector& keyframes)
{
    bool upright = true;
    double min_surf_threshold = 25;

    if(verbose_) printf("preparing SURF features for matching...\n");
    //  found_markers.resize(keyframes.size());
    cv::SurfDescriptorExtractor extractor;

    for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
    {
        RGBDKeyframe& keyframe = keyframes[kf_idx];
        //    std::vector<aruco::Marker> markers;
        //    MDetector.detect(keyframe.rgb_img,markers);
        //    if (markers.size() > 0)
        //    {
        //        found_markers[kf_idx]=markers;
        //        std::cout << "Found a marker in kframe " + kf_idx << std::endl;
        //    }
        double surf_threshold = init_surf_threshold_;

        while (surf_threshold >= min_surf_threshold)
        {
            cv::SurfFeatureDetector detector(surf_threshold, 4, 2, true, upright);
            keyframe.keypoints.clear();
            detector.detect(keyframe.rgb_img, keyframe.keypoints);

            if ((int)keyframe.keypoints.size() < n_keypoints_)
            {
                if(verbose_)
                    printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n",
                           (int)kf_idx+1, (int)keyframes.size(),
                           (int)keyframe.keypoints.size(), surf_threshold);

                surf_threshold /= 2.0;
            }
            else
            {
                keyframe.keypoints.resize(n_keypoints_);

                if(verbose_)
                    printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n",
                           (int)kf_idx+1, (int)keyframes.size(),
                           (int)keyframe.keypoints.size(), surf_threshold);

                break;
            }
        }

        if (sac_save_results_)
        {
            cv::Mat kp_img;
            cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
            std::stringstream ss1;
            ss1 << "kp_" << kf_idx;
            cv::imwrite(output_path_ + "/" + ss1.str() + ".png", kp_img);
        }

        extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
        keyframe.computeDistributions(20,20);
    }
}

void KeyframeGraphDetector::prepareFeaturesForRANSAC_mine(
        KeyframeVector& keyframes)
{
    bool upright = true;

    if(verbose_) printf("preparing SURF features for matching...\n");

    cv::SurfDescriptorExtractor extractor;

    for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
    {
        RGBDKeyframe& keyframe = keyframes[kf_idx];

        double surf_threshold = init_surf_threshold_;


        cv::SurfFeatureDetector detector(surf_threshold, 4, 2, true, upright);
        keyframe.keypoints.clear();
        detector.detect(keyframe.rgb_img, keyframe.keypoints);



        keyframe.keypoints.resize(n_keypoints_);

        if(verbose_)
            printf("[KF %d of %d] %d SURF keypoints detected\n",
                   (int)kf_idx+1, (int)keyframes.size(),
                   (int)keyframe.keypoints.size());


        if (sac_save_results_)
        {
            cv::Mat kp_img;
            cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
            std::stringstream ss1;
            ss1 << "kp_" << kf_idx;
            cv::imwrite(output_path_ + "/" + ss1.str() + ".png", kp_img);
        }

        extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
        keyframe.computeDistributions();
    }
}


void KeyframeGraphDetector::buildAssociationMatrix(
        const KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    prepareMatchers(keyframes);

    // 1. Create the candidate matrix
    buildCandidateMatrix(keyframes);

    // 2. Perfrom pairwise matching for all candidates
    //  buildCorrespondenceMatrix(keyframes, associations);
    buildCorrespondenceMatrix_mine(keyframes,associations);
    //  buildCorrespondenceMatrix_onlyLandmarks(keyframes,associations);
}
void KeyframeGraphDetector::buildAssociationMatrix_Iterative(
        const KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    prepareMatchers(keyframes);

    // 1. Create the candidate matrix
    buildCandidateMatrix(keyframes);

}
void KeyframeGraphDetector::buildCandidateMatrix(
        const KeyframeVector& keyframes)
{
    if (candidate_method_ == CANDIDATE_GENERATION_BRUTE_FORCE) // brute-force
    {
        // create a candidate matrix which considers all posiible combinations
        int size = keyframes.size();
        candidate_matrix_ = cv::Mat::ones(size, size, CV_8UC1);
    }
    else if (candidate_method_ == CANDIDATE_GENERATION_SURF_TREE) // tree-based
    {
        // build a math knn matrix using a kdtree
        buildMatchMatrixSurfTree(keyframes);

        // keep only the top n candidates
        buildCandidateMatrixSurfTree();
    }
}
/** @brief Builds a matrix of nearest neighbor matches between keyframes
* using a kdtree
*
* match_matrix[query, train] = X correspondences
*/
cv::FlannBasedMatcher matcher;
void KeyframeGraphDetector::buildMatchMatrixSurfTree_Iterative(unsigned int kf_idx,
                                                               const KeyframeVector& keyframes)
{
    if(match_matrix_.cols == 0)
        match_matrix_ = cv::Mat::zeros(1000, 1000, CV_16UC1);


    unsigned int kf_size = keyframes.size();


    // train matcher from all the features
    int max,min;
    if (!mapping_)
    {
        min = 0;
        max = n_original_poses;
    }
    else
    {
        min = keyframes.size() -1;
        max = keyframes.size();
    }
    trainSURFMatcher_Iterative(keyframes,min, max, matcher);


    // lookup per frame
    if(verbose_) printf("Keyframe lookups...\n");


    if(verbose_)
        printf("[KF %d of %d]: Computing SURF neighbors\n", (int)kf_idx+1, (int)kf_size);
    const RGBDFrame& keyframe = keyframes[kf_idx];

    // find k nearest matches for each feature in the keyframe
    std::vector<std::vector<cv::DMatch> > matches_vector;
    matcher.knnMatch(keyframe.descriptors, matches_vector, k_nearest_neighbors_);

    // create empty bins vector of Pairs <count, image_index>
    std::vector<std::pair<int, int> > bins;
    bins.resize(kf_size);
    for (unsigned int b = 0; b < bins.size(); ++b)
        bins[b] = std::pair<int, int>(0, b);

    // fill out bins with match indices
    for (unsigned int j = 0; j < matches_vector.size(); ++j)
    {
        std::vector<cv::DMatch>& matches = matches_vector[j];
        for (unsigned int k = 0; k < matches.size(); ++k)
        {
            if( !mapping_ && bins[matches[k].imgIdx].second >= n_original_poses)
                continue;
            bins[matches[k].imgIdx].first++;
        }
    }

    for (unsigned int b = 0; b < kf_size; ++b)
    {
        unsigned int index_a = kf_idx;
        unsigned int index_b = bins[b].second;
        int corresp_count = bins[b].first;

        if (index_a != index_b)
            match_matrix_.at<uint16_t>(index_a, index_b) = corresp_count;

    }

}

/** @brief Builds a matrix of nearest neighbor matches between keyframes 
* using a kdtree
* 
* match_matrix[query, train] = X correspondences
*/  
void KeyframeGraphDetector::buildMatchMatrixSurfTree(
        const KeyframeVector& keyframes)
{
    unsigned int kf_size = keyframes.size();

    // train matcher from all the features
    cv::FlannBasedMatcher matcher;
    trainSURFMatcher(keyframes, matcher);

    // lookup per frame
    if(verbose_) printf("Keyframe lookups...\n");

    match_matrix_ = cv::Mat::zeros(kf_size, kf_size, CV_16UC1);
    for (unsigned int kf_idx = 0; kf_idx < kf_size; ++kf_idx)
    {
        if(verbose_)
            printf("[KF %d of %d]: Computing SURF neighbors\n", (int)kf_idx+1, (int)kf_size);
        const RGBDFrame& keyframe = keyframes[kf_idx];

        // find k nearest matches for each feature in the keyframe
        std::vector<std::vector<cv::DMatch> > matches_vector;
        matcher.knnMatch(keyframe.descriptors, matches_vector, k_nearest_neighbors_);

        // create empty bins vector of Pairs <count, image_index>
        std::vector<std::pair<int, int> > bins;
        bins.resize(kf_size);
        for (unsigned int b = 0; b < bins.size(); ++b)
            bins[b] = std::pair<int, int>(0, b);

        // fill out bins with match indices
        for (unsigned int j = 0; j < matches_vector.size(); ++j)
        {
            std::vector<cv::DMatch>& matches = matches_vector[j];
            for (unsigned int k = 0; k < matches.size(); ++k)
            {
                bins[matches[k].imgIdx].first++;
            }
        }

        for (unsigned int b = 0; b < kf_size; ++b)
        {
            unsigned int index_a = kf_idx;
            unsigned int index_b = bins[b].second;
            int corresp_count = bins[b].first;

            if (index_a != index_b)
                match_matrix_.at<uint16_t>(index_a, index_b) = corresp_count;
        }
    }
}
/** @brief Takes in a matrix of matches from a SURF tree (match_matrix_)
 * and marks the top n_candidates in each row into (candidate_matrix)
 */
void KeyframeGraphDetector::buildCandidateMatrixSurfTree_Iterative(int v)
{
    // check for square matrix
    assert(match_matrix_.rows == match_matrix_.cols);

    // check for validity of n_candidates argument
    int size = match_matrix_.rows;
    int temp_n_candidates_ = n_candidates_;
    if(n_candidates_ > size)
    {
        temp_n_candidates_ = size;
    }
    assert(temp_n_candidates_ <= size);

    // initialize candidate matrix as all 0
    candidate_matrix_ = cv::Mat::eye(match_matrix_.size(), CV_8UC1);

    // create a vector from the current row
    std::vector<std::pair<int, int> > values(match_matrix_.cols);
    for (int u = 0; u < match_matrix_.cols; ++u)
    {
        int value = match_matrix_.at<uint16_t>(v,u);
        values[u] =  std::pair<int, int>(value, u);
    }

    // sort the vector based on values, highest first
    std::sort(values.begin(), values.end(), std::greater<std::pair<int, int> >());

    // mark 1 for the top n_candidates, if > 0
    for (int u = 0; u < temp_n_candidates_; ++u)
    {
        if (values[u].first == 0) continue;
        if (!mapping_ && values[u].second >= n_original_poses)
        {
            temp_n_candidates_++;
            continue;
        }
        unsigned int uc = values[u].second;
        candidate_matrix_.at<uint8_t>(v,uc) = 1;
        std::cout << v << " with " << uc <<std::endl;
    }

}
/** @brief Takes in a matrix of matches from a SURF tree (match_matrix_)
 * and marks the top n_candidates in each row into (candidate_matrix)
 */
void KeyframeGraphDetector::buildCandidateMatrixSurfTree()
{
    // check for square matrix
    assert(match_matrix_.rows == match_matrix_.cols);

    // check for validity of n_candidates argument
    int size = match_matrix_.rows;
    int temp_n_candidates_ = n_candidates_;
    if(n_candidates_ > size)
    {
        temp_n_candidates_ = size;
    }
    assert(temp_n_candidates_ <= size);

    // initialize candidate matrix as all 0
    candidate_matrix_ = cv::Mat::eye(match_matrix_.size(), CV_8UC1);

    for (int v = 0; v < match_matrix_.rows; ++v)
    {
        // create a vector from the current row
        std::vector<std::pair<int, int> > values(match_matrix_.cols);
        for (int u = 0; u < match_matrix_.cols; ++u)
        {
            int value = match_matrix_.at<uint16_t>(v,u);
            values[u] =  std::pair<int, int>(value, u);
        }

        // sort the vector based on values, highest first
        std::sort(values.begin(), values.end(), std::greater<std::pair<int, int> >());

        // mark 1 for the top n_candidates, if > 0
        for (int u = 0; u < temp_n_candidates_; ++u)
        {
            if (values[u].first == 0) continue;
            unsigned int uc = values[u].second;
            candidate_matrix_.at<uint8_t>(v,uc) = 1;
        }
    }
}
void KeyframeGraphDetector::buildCorrespondenceMatrix(
        const KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    // check for square matrix
    assert(candidate_matrix_.rows == candidate_matrix_.cols);
    int size = candidate_matrix_.rows;

    // initialize correspondence matrix
    correspondence_matrix_ = cv::Mat::zeros(size, size, CV_16UC1);
    association_matrix_    = cv::Mat::zeros(size, size, CV_8UC1);

    for (int kf_idx_q = 0; kf_idx_q < size; ++kf_idx_q)
        for (int kf_idx_t = 0; kf_idx_t < size; ++kf_idx_t)
        {
            const RGBDKeyframe& keyframe_q = keyframes[kf_idx_q];
            const RGBDKeyframe& keyframe_t = keyframes[kf_idx_t];

            if (kf_idx_q == kf_idx_t)
            {
                // self-association

                //correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.keypoints.size();
                correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.n_valid_keypoints;

                association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_q) = 1;
            }
            else
            {
                // skip non-candidates
                if (candidate_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) != 0)
                {
                    if(verbose_) printf("[RANSAC %d to %d]: ", kf_idx_q, kf_idx_t);

                    std::vector<cv::DMatch> inlier_matches;

                    // perform ransac matching, b onto a
                    Eigen::Matrix4f transformation;

                    // query, train
                    int iterations = pairwiseMatching(
                                kf_idx_q, kf_idx_t, keyframes, inlier_matches, transformation);

                    correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_t) = inlier_matches.size();

                    if (inlier_matches.size() >= sac_min_inliers_)
                    {
                        // mark the association matrix
                        association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;

                        // add an association
                        KeyframeAssociation association;
                        association.type = KeyframeAssociation::RANSAC;
                        association.kf_idx_a = kf_idx_t;
                        association.kf_idx_b = kf_idx_q;
                        association.matches  = inlier_matches;
                        association.a2b = transformation;
                        associations.push_back(association);

                        // output the results to screen
                        if(verbose_)
                            printf("pass [%d][%d]\n", iterations, (int)inlier_matches.size());

                        // save the results to file
                        if (sac_save_results_)
                        {
                            cv::Mat img_matches;
                            cv::drawMatches(keyframe_q.rgb_img, keyframe_q.keypoints,
                                            keyframe_t.rgb_img, keyframe_t.keypoints,
                                            inlier_matches, img_matches);

                            std::stringstream ss1;
                            ss1 << kf_idx_q << "_to_" << kf_idx_t;
                            cv::imwrite(output_path_ + "/" + ss1.str() + ".png", img_matches);
                        }
                    }
                    else
                    {
                        association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;

                        if(verbose_)
                            printf("fail [%d][%d]\n", iterations, (int)inlier_matches.size());
                    }
                }
            }
        }
}
void KeyframeGraphDetector::buildCorrespondenceMatrix_onlyLandmarks(
        const KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    // check for square matrix
    assert(candidate_matrix_.rows == candidate_matrix_.cols);
    int size = candidate_matrix_.rows;

    // initialize correspondence matrix
    correspondence_matrix_ = cv::Mat::zeros(size, size, CV_16UC1);
    association_matrix_    = cv::Mat::zeros(size, size, CV_8UC1);

    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel(0.98);
    rmatcher.setMinDistanceToEpipolar(2.0);
    rmatcher.setRatio(1.00f);
    rmatcher.setVerbose(false);
    for (int kf_idx_q = 0; kf_idx_q < size; ++kf_idx_q)
        for (int kf_idx_t = 0; kf_idx_t < size; ++kf_idx_t)
        {
            const RGBDKeyframe& keyframe_q = keyframes[kf_idx_q];
            const RGBDKeyframe& keyframe_t = keyframes[kf_idx_t];

            if (kf_idx_q == kf_idx_t)
            {
                // self-association
                //correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.keypoints.size();
                correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.n_valid_keypoints;

                association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_q) = 1;
            }
            else
            {
                // skip non-candidates
                if (candidate_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) != 0)
                {
                    std::vector<cv::DMatch> inlier_matches;

                    // perform ransac matching, b onto a

                    std::vector<cv::DMatch> matches_r;
                    std::vector<cv::KeyPoint> kp1,kp2;
                    kp1 = keyframe_q.keypoints;
                    kp2 = keyframe_t.keypoints;
                    cv::Mat fundemental = rmatcher.match(matches_r,kp1,kp2,keyframe_q.descriptors,keyframe_t.descriptors);
                    if(matches_r.size() < n_matches_accept)
                    {
                        association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;

                        if(verbose_)
                            printf("[%d to %d] Fail: Not enough matches [%d]\n", kf_idx_q,kf_idx_t,(int)matches_r.size());
                        continue;
                    }

                    for (u_int i = 0; i < matches_r.size(); i++)
                    {
                        if (keyframe_q.kp_valid[matches_r[i].queryIdx]==true && keyframe_t.kp_valid[matches_r[i].trainIdx]==true )
                            inlier_matches.push_back(matches_r[i]);
                    }
                    correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_t) = inlier_matches.size();
                    if (inlier_matches.size() >= sac_min_inliers_)
                    {
                        // mark the association matrix
                        association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;
                        Eigen::Matrix4f transformation;
                        transformation.setIdentity();
                        // add an association

                        KeyframeAssociation association;
                        association.type = KeyframeAssociation::LANDMARKS;
                        association.kf_idx_a = kf_idx_t;
                        association.kf_idx_b = kf_idx_q;
                        association.matches  = inlier_matches;
                        association.a2b = transformation;
                        associations.push_back(association);

                        // output the results to screen
                        if(verbose_)
                            printf("[%d to %d] Pass: Matches [%d]\n", kf_idx_q,kf_idx_t, (int)matches_r.size());

                    }



                }
            }
        }
}

void KeyframeGraphDetector::buildCorrespondenceMatrix_mine(
        const KeyframeVector& keyframes,
        KeyframeAssociationVector& associations)
{
    // check for square matrix
    assert(candidate_matrix_.rows == candidate_matrix_.cols);
    int size = candidate_matrix_.rows;

    // initialize correspondence matrix
    correspondence_matrix_ = cv::Mat::zeros(size, size, CV_16UC1);
    association_matrix_    = cv::Mat::zeros(size, size, CV_8UC1);

    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel(0.98);
    rmatcher.setMinDistanceToEpipolar(2.0);
    rmatcher.setRatio(1.00f);
    rmatcher.setVerbose(false);
    for (int kf_idx_q = 0; kf_idx_q < size; ++kf_idx_q)
        for (int kf_idx_t = 0; kf_idx_t < size; ++kf_idx_t)
        {
            const RGBDKeyframe& keyframe_q = keyframes[kf_idx_q];
            const RGBDKeyframe& keyframe_t = keyframes[kf_idx_t];

            if (kf_idx_q == kf_idx_t)
            {
                // self-association
                //correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.keypoints.size();
                correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.n_valid_keypoints;

                association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_q) = 1;
            }
            else
            {
                // skip non-candidates
                if (candidate_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) != 0)
                {
                    if(verbose_) printf("[RANSAC %d to %d]: ", kf_idx_q, kf_idx_t);

                    std::vector<cv::DMatch> inlier_matches;

                    // perform ransac matching, b onto a
                    Eigen::Matrix4f transformation;

                    // query, train
                    //        int iterations = pairwiseMatching(
                    //          kf_idx_q, kf_idx_t, keyframes, inlier_matches, transformation);

                    std::vector<cv::DMatch> matches_r;
                    std::vector<cv::KeyPoint> kp1,kp2;
                    kp1 = keyframe_q.keypoints;
                    kp2 = keyframe_t.keypoints;
                    cv::Mat fundemental = rmatcher.match(matches_r,kp1,kp2,keyframe_q.descriptors,keyframe_t.descriptors);
                    if(matches_r.size() < n_matches_accept)
                    {
                        association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;
                        //            association_matrix_.at<uint8_t>(kf_idx_t,kf_idx_q) = 0;

                        if(verbose_)
                            printf("fail: Not enough matches [%d]\n", (int)matches_r.size());
                        if(matches_r.size()  == 0)
                            continue;
                        if (sac_save_results_)
                        {
                            std::vector<cv::Point2f> points1, points2;
                            cv::Mat image1;
                            keyframe_q.rgb_img.copyTo(image1);
                            cv::Mat image2;
                            keyframe_t.rgb_img.copyTo(image2);

                            for (std::vector<cv::DMatch>::const_iterator it= matches_r.begin();
                                 it!= matches_r.end(); ++it) {

                                // Get the position of left keypoints
                                float x= kp1[it->queryIdx].pt.x;
                                float y= kp1[it->queryIdx].pt.y;
                                points1.push_back(cv::Point2f(x,y));
                                cv::circle(image1,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
                                // Get the position of right keypoints
                                x= kp2[it->trainIdx].pt.x;
                                y= kp2[it->trainIdx].pt.y;
                                cv::circle(image2,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
                                points2.push_back(cv::Point2f(x,y));
                            }

                            // Draw the epipolar lines
                            std::vector<cv::Vec3f> lines1;
                            cv::computeCorrespondEpilines(cv::Mat(points1),1,fundemental,lines1);

                            for (std::vector<cv::Vec3f>::const_iterator it= lines1.begin();
                                 it!=lines1.end(); ++it) {

                                cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
                                        cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
                                        cv::Scalar(255,255,255));
                            }

                            std::vector<cv::Vec3f> lines2;
                            cv::computeCorrespondEpilines(cv::Mat(points2),2,fundemental,lines2);

                            for (std::vector<cv::Vec3f>::const_iterator it= lines2.begin();
                                 it!=lines2.end(); ++it) {

                                cv::line(image1,cv::Point(0,-(*it)[2]/(*it)[1]),
                                        cv::Point(image1.cols,-((*it)[2]+(*it)[0]*image1.cols)/(*it)[1]),
                                        cv::Scalar(255,255,255));
                            }
                            cv::Mat outImg,outImg1,outImg2;
                            cv::Size size( image1.cols + image2.cols, MAX(image1.rows, image2.rows) );
                            outImg.create( size, CV_MAKETYPE(image1.depth(), 3) );
                            outImg1 = outImg( cv::Rect(0, 0, image1.cols, image1.rows) );
                            outImg2 = outImg( cv::Rect(image1.cols, 0, image2.cols, image2.rows) );
                            image1.copyTo( outImg1 );
                            image2.copyTo( outImg2 );
                            std::stringstream ss1;
                            ss1 << kf_idx_q << "_to_" << kf_idx_t << "_fail_matches[" << matches_r.size() << "].png";
                            cv::imwrite(output_path_ + "/" + ss1.str() , outImg);
                        }
                        continue;
                    }
                    DMatchVector best_inlier_matches;
                    //        int iterations = pairwiseMatchingRANSAC(
                    //                    kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, transformation,matches_r);
                    int iterations = pairwiseMatchingRANSAC(
                                kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, transformation);
                    inlier_matches = best_inlier_matches;
                    correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_t) = inlier_matches.size();
                    //        correspondence_matrix_.at<uint16_t>( kf_idx_t,kf_idx_q) = inlier_matches.size();

                    //        if ( inlier_matches.size() > 20)
                    //        {
                    //            std::ostringstream oss;
                    //            oss << "Matched " << kf_idx_q << " to " << kf_idx_t << " with " << inlier_matches.size() << " inliers.";
                    //            std::cout << oss.str() << std::endl;
                    //        }
                    if (inlier_matches.size() >= sac_min_inliers_)
                    {
                        // mark the association matrix
                        association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;
                        //          association_matrix_.at<uint8_t>(kf_idx_t,kf_idx_q) = 1;

                        // add an association
                        KeyframeAssociation association;
                        association.type = KeyframeAssociation::RANSAC;
                        association.kf_idx_a = kf_idx_t;
                        association.kf_idx_b = kf_idx_q;
                        association.matches  = inlier_matches;
                        association.a2b = transformation;
                        associations.push_back(association);

                        // output the results to screen
                        if(verbose_)
                            printf("pass [%d][%d]\n", iterations, (int)inlier_matches.size());

                        // save the results to file
                        if (sac_save_results_)
                        {
                            std::vector<cv::Point2f> points1, points2;
                            cv::Mat image1;
                            keyframe_q.rgb_img.copyTo(image1);
                            cv::Mat image2;
                            keyframe_t.rgb_img.copyTo(image2);

                            for (std::vector<cv::DMatch>::const_iterator it= inlier_matches.begin();
                                 it!= inlier_matches.end(); ++it) {

                                // Get the position of left keypoints
                                float x= kp1[it->queryIdx].pt.x;
                                float y= kp1[it->queryIdx].pt.y;
                                points1.push_back(cv::Point2f(x,y));
                                cv::circle(image1,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
                                // Get the position of right keypoints
                                x= kp2[it->trainIdx].pt.x;
                                y= kp2[it->trainIdx].pt.y;
                                cv::circle(image2,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
                                points2.push_back(cv::Point2f(x,y));
                            }

                            // Draw the epipolar lines
                            std::vector<cv::Vec3f> lines1;
                            cv::computeCorrespondEpilines(cv::Mat(points1),1,fundemental,lines1);

                            for (std::vector<cv::Vec3f>::const_iterator it= lines1.begin();
                                 it!=lines1.end(); ++it) {

                                cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
                                        cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
                                        cv::Scalar(255,255,255));
                            }

                            std::vector<cv::Vec3f> lines2;
                            cv::computeCorrespondEpilines(cv::Mat(points2),2,fundemental,lines2);

                            for (std::vector<cv::Vec3f>::const_iterator it= lines2.begin();
                                 it!=lines2.end(); ++it) {

                                cv::line(image1,cv::Point(0,-(*it)[2]/(*it)[1]),
                                        cv::Point(image1.cols,-((*it)[2]+(*it)[0]*image1.cols)/(*it)[1]),
                                        cv::Scalar(255,255,255));
                            }
                            cv::Mat outImg,outImg1,outImg2;
                            cv::Size size( image1.cols + image2.cols, MAX(image1.rows, image2.rows) );
                            outImg.create( size, CV_MAKETYPE(image1.depth(), 3) );
                            outImg1 = outImg( cv::Rect(0, 0, image1.cols, image1.rows) );
                            outImg2 = outImg( cv::Rect(image1.cols, 0, image2.cols, image2.rows) );
                            image1.copyTo( outImg1 );
                            image2.copyTo( outImg2 );
                            std::stringstream ss1;
                            ss1 << kf_idx_q << "_to_" << kf_idx_t << "_OK["  << inlier_matches.size() << "].png";
                            cv::imwrite(output_path_ + "/" + ss1.str() , outImg);
                        }
                    }
                    else
                    {
                        if(matches_r.size() >= n_matches_accept)
                        {
                            std::vector<cv::DMatch> final_matches;
                            for (u_int i=0; i<matches_r.size();i++)
                            {
                                if (keyframe_t.kp_valid[matches_r[i].trainIdx] && keyframe_q.kp_valid[matches_r[i].queryIdx])
                                    final_matches.push_back(matches_r[i]);
                            }
                            if(final_matches.size()==0)
                                continue;
                            association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;
                            KeyframeAssociation association;
                            association.type = KeyframeAssociation::LANDMARKS;
                            association.kf_idx_a = kf_idx_t;
                            association.kf_idx_b = kf_idx_q;
                            association.matches  = final_matches;
                            association.a2b = transformation;
                            associations.push_back(association);

                            if(verbose_)
                                printf("fail [%d][%d] - Accepted\n", iterations, (int)final_matches.size());
                        }
                        else
                        {
                            if(verbose_)
                                printf("fail [%d][%d]\n", iterations, (int)inlier_matches.size());
                        }
                        if (sac_save_results_)
                        {
                            //              DMatchVector candidate_matches;
                            //              cv::FlannBasedMatcher& matcher = matchers_[kf_idx_t];
                            //              getCandidateMatches(keyframe_q, keyframe_t, matcher, candidate_matches);
                            cv::Mat img_matches;
                            std::vector<cv::KeyPoint> q_pt;
                            std::vector<cv::KeyPoint> t_pt;
                            std::vector<int> q_,t_;
                            int ct=0,cq=0;
                            if(keyframe_q.keypoints.size() != keyframe_t.keypoints.size())
                                continue;
                            for(u_int k = 0; k<keyframe_q.keypoints.size();k++)
                            {
                                if(keyframe_q.kp_valid[k]==true)
                                {
                                    q_pt.push_back(keyframe_q.keypoints[k]);
                                    q_.push_back(k-cq);
                                }
                                else
                                {
                                    cq++;
                                    q_.push_back(-1);
                                }
                                if(keyframe_t.kp_valid[k]==true)
                                {
                                    t_pt.push_back(keyframe_t.keypoints[k]);
                                    t_.push_back(k-ct);
                                }
                                else
                                {
                                    ct++;
                                    t_.push_back(-1);
                                }
                            }
                            for(u_int k=0;k<inlier_matches.size();k++)
                            {
                                if(q_[inlier_matches[k].queryIdx]==-1)
                                {
                                    printf("OOOOOOOOO");
                                }
                                else
                                {
                                    inlier_matches[k].queryIdx = q_[inlier_matches[k].queryIdx];
                                }
                                if(t_[inlier_matches[k].trainIdx]==-1)
                                {
                                    printf("OOOOOOOOO");
                                }
                                else
                                {
                                    inlier_matches[k].trainIdx = t_[inlier_matches[k].trainIdx];
                                }
                            }

                            cv::drawMatches(keyframe_q.rgb_img, q_pt,
                                            keyframe_t.rgb_img, t_pt,
                                            inlier_matches, img_matches);

                            std::stringstream ss1;
                            ss1 << kf_idx_q << "_to_" << kf_idx_t << "_fail_RANSAC[" << inlier_matches.size() << "].png";
                            cv::imwrite(output_path_ + "/" + ss1.str(), img_matches);
                        }

                    }
                }
            }
        }
}
void KeyframeGraphDetector::buildCorrespondenceMatrix_rgb_Iterative(u_int kf_idx_q,
                                                                    const KeyframeVector& keyframes,
                                                                    KeyframeAssociationVector& associations)
{

    assert(candidate_matrix_.rows == candidate_matrix_.cols);
    int size = candidate_matrix_.rows;
    if(size > correspondence_matrix_.cols)
    {
        if(correspondence_matrix_.cols == 0)
        {
            // initialize correspondence matrix
            correspondence_matrix_ = cv::Mat::zeros(1000, 1000, CV_16UC1);
            association_matrix_    = cv::Mat::zeros(1000, 1000, CV_8UC1);
        }
        else
        {
            std::cout << "Max number of keyframes reached." << std::endl;
            return;
        }
    }
    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel(0.95);
    rmatcher.setMinDistanceToEpipolar(2.0);
    rmatcher.setRatio(1.00f);
    rmatcher.setVerbose(false);
    for (int kf_idx_t = 0; kf_idx_t < size; ++kf_idx_t)
    {
        RGBDKeyframe keyframe_q = keyframes[kf_idx_t]; // switched on purpose << -- map
        RGBDKeyframe keyframe_t = keyframes[kf_idx_q]; // switched on purpose << -- localization

        if (kf_idx_q == kf_idx_t)
        {
            // self-association
            //correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.keypoints.size();
            correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.n_valid_keypoints;

            association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_q) = 1;
        }
        else
        {
            // skip non-candidates
            if (candidate_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) != 0)
            {
                if(verbose_) printf("[RANSAC %d to %d]: ", kf_idx_q, kf_idx_t);

                // perform ransac matching, b onto a
                Eigen::Matrix4f transformation;

                std::vector<cv::DMatch> matches_r;
                std::vector<cv::KeyPoint> kp1,kp2;
                kp1 = keyframe_q.keypoints;
                kp2 = keyframe_t.keypoints;
                cv::Mat fundemental = rmatcher.match(matches_r,kp1,kp2,keyframe_q.descriptors,keyframe_t.descriptors);
                if(matches_r.size() < n_matches_accept)
                {
                    if(verbose_)
                        printf("fail: Not enough matches [%d]\n", (int)matches_r.size());
                    continue;
                }
                std::vector<cv::Point3f> points3d,inliers_3d;
                std::vector<cv::Point2f> points2d_q,points2d_t,inliers_2d;
                cv::Mat intr = keyframe_t.intr;
                cv::Mat dist = keyframe_t.dist;
                cv::Mat rvec,tvec;
                std::vector<std::vector<float> > dists;

                for (u_int i = 0; i < matches_r.size(); i++)
                {
                    cv::DMatch match = matches_r[i];
                    cv::KeyPoint p_q,p_t;
                    p_q = kp1[match.queryIdx];
                    p_t = kp2[match.trainIdx];
                    cv::Point2f pq,pt;
                    pq = p_q.pt;
                    pt = p_t.pt;
                    if(pq.x < 0 || pt.x < 0 || pq.x >= 640 || pt.x >= 640 || pt.y < 0 || pq.y < 0 || pt.y >= 480 || pq.y >= 480)
                        continue;
                    if(!keyframe_q.kp_valid[match.queryIdx])
                        continue;
                    points2d_q.push_back(pq);
                    points2d_t.push_back(pt);
                    std::vector<float> t_dist;
                    t_dist.push_back(keyframe_q.kp_means[match.queryIdx][0]);
                    t_dist.push_back(keyframe_q.kp_means[match.queryIdx][1]);
                    t_dist.push_back(keyframe_q.kp_means[match.queryIdx][2]);

                    dists.push_back(t_dist);
                }

                for(u_int i = 0; i < dists.size(); i++)
                {
                    cv::Point3f pt;
                    pt.x = dists[i][0];
                    pt.y = dists[i][1];
                    pt.z = dists[i][2];
                    points3d.push_back(pt);
                }

                assert(points3d.size() == points2d_t.size());
                if(points3d.size() < 30)
                {
                    association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;

                    if(verbose_)
                        printf("fail: Not enough points [%d]\n", (int)points3d.size());
                    continue;
                }
                std::vector<int> inliers;

                cv::solvePnPRansac(points3d,points2d_t,intr,dist,rvec,tvec,false,200,4,100,inliers,CV_EPNP);
                for(u_int i = 0; i < inliers.size(); i++)
                {
                    inliers_3d.push_back(points3d[inliers[i]]);
                    inliers_2d.push_back(points2d_t[inliers[i]]);
                }

                if(tvec.at<double>(0,0) == 0 && tvec.at<double>(0,1) == 0 && tvec.at<double>(0,2) == 0)
                {
                    association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;

                    if(verbose_)
                        printf("fail: Bad estimate \n");
                    continue;
                }
                //                std::cout << rvec << " " << tvec << std::endl;
                cv::Mat rmat;
                cv::Rodrigues(rvec,rmat);

                std::vector<cv::Point2f> reproj_points;
                cv::projectPoints(inliers_3d,rvec,tvec,intr,dist,reproj_points);
                double error = 0;
                for(u_int i = 0; i < reproj_points.size(); i++)
                {
                    error += std::abs(reproj_points[i].x - inliers_2d[i].x);
                    error += std::abs(reproj_points[i].y - inliers_2d[i].y);
                }
                error /= reproj_points.size();
                if (error > 3)
                {
                    association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;

                    if(verbose_)
                        printf("fail: Reprojection Error above threshold [%d]\n", (int)error);
                    continue;
                }
                std::cout << error << std::endl;
                rmat = rmat.t();
                tvec = rmat*-tvec;

                transformation(0,0) = rmat.at<double>(0,0);
                transformation(1,0) = rmat.at<double>(1,0);
                transformation(2,0) = rmat.at<double>(2,0);
                transformation(0,1) = rmat.at<double>(0,1);
                transformation(1,1) = rmat.at<double>(1,1);
                transformation(2,1) = rmat.at<double>(2,1);
                transformation(0,2) = rmat.at<double>(0,2);
                transformation(1,2) = rmat.at<double>(1,2);
                transformation(2,2) = rmat.at<double>(2,2);
                transformation(3,0) = transformation(3,1) = transformation(3,2) = 0;
                transformation(3,3) = 1;
                transformation(0,3) = tvec.at<double>(0,0);
                transformation(1,3) = tvec.at<double>(0,1);
                transformation(2,3) = tvec.at<double>(0,2);
                association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;

                // add an association
                KeyframeAssociation association;
                association.type = KeyframeAssociation::RANSAC;
                association.kf_idx_a = kf_idx_t;
                association.kf_idx_b = kf_idx_q;
                association.matches  = matches_r;
                association.a2b = transformation;
                associations.push_back(association);

                // output the results to screen
                if(verbose_)
                    printf("pass [-][%d]\n", (int)matches_r.size());

            }
        }
    }
}

void KeyframeGraphDetector::buildCorrespondenceMatrix_mine_Iterative(u_int kf_idx_q,
                                                                     const KeyframeVector& keyframes,
                                                                     KeyframeAssociationVector& associations)
{
    // check for square matrix
    assert(candidate_matrix_.rows == candidate_matrix_.cols);
    int size = candidate_matrix_.rows;
    if(size > correspondence_matrix_.cols)
    {
        if(correspondence_matrix_.cols == 0)
        {
            // initialize correspondence matrix
            correspondence_matrix_ = cv::Mat::zeros(1000, 1000, CV_16UC1);
            association_matrix_    = cv::Mat::zeros(1000, 1000, CV_8UC1);
        }
        else
        {
            std::cout << "Max number of keyframes reached." << std::endl;
            return;
        }
    }
    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel(0.98);
    rmatcher.setMinDistanceToEpipolar(2.0);
    rmatcher.setRatio(1.00f);
    rmatcher.setVerbose(false);
    for (int kf_idx_t = 0; kf_idx_t < size; ++kf_idx_t)
    {
        const RGBDKeyframe& keyframe_q = keyframes[kf_idx_q];
        const RGBDKeyframe& keyframe_t = keyframes[kf_idx_t];

        if (kf_idx_q == kf_idx_t)
        {
            // self-association
            //correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.keypoints.size();
            correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.n_valid_keypoints;

            association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_q) = 1;
        }
        else
        {
            // skip non-candidates
            if (candidate_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) != 0)
            {
                if(verbose_) printf("[RANSAC %d to %d]: ", kf_idx_q, kf_idx_t);

                std::vector<cv::DMatch> inlier_matches;

                // perform ransac matching, b onto a
                Eigen::Matrix4f transformation;

                std::vector<cv::DMatch> matches_r;
                std::vector<cv::KeyPoint> kp1,kp2;
                kp1 = keyframe_q.keypoints;
                kp2 = keyframe_t.keypoints;
                cv::Mat fundemental = rmatcher.match(matches_r,kp1,kp2,keyframe_q.descriptors,keyframe_t.descriptors);
                if(matches_r.size() < n_matches_accept)
                {
                    association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;

                    if(verbose_)
                        printf("fail: Not enough matches [%d]\n", (int)matches_r.size());
                    continue;
                }
                DMatchVector best_inlier_matches;
                int iterations = pairwiseMatchingRANSAC(
                            kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, transformation,matches_r);
                inlier_matches = best_inlier_matches;
                correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_t) = inlier_matches.size();

                if (inlier_matches.size() >= sac_min_inliers_)
                {
                    // mark the association matrix
                    association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;
                    //          association_matrix_.at<uint8_t>(kf_idx_t,kf_idx_q) = 1;

                    // add an association
                    KeyframeAssociation association;
                    association.type = KeyframeAssociation::RANSAC;
                    association.kf_idx_a = kf_idx_t;
                    association.kf_idx_b = kf_idx_q;
                    association.matches  = inlier_matches;
                    association.a2b = transformation;
                    associations.push_back(association);

                    // output the results to screen
                    if(verbose_)
                        printf("pass [%d][%d]\n", iterations, (int)inlier_matches.size());
                }
                else
                {
                    if(verbose_)
                        printf("fail: Not enough matches [%d]\n", (int)inlier_matches.size());
                }

            }
        }
    }
}

// frame_a = train, frame_b = query
void KeyframeGraphDetector::getCandidateMatches(
        const RGBDFrame& frame_q, const RGBDFrame& frame_t,
        cv::FlannBasedMatcher& matcher,
        DMatchVector& candidate_matches)
{
    // **** build candidate matches ***********************************
    // assumes detectors and distributions are computed
    // establish all matches from b to a

    if (matcher_use_desc_ratio_test_)
    {
        std::vector<DMatchVector> all_matches2;

        matcher.knnMatch(
                    frame_q.descriptors, all_matches2, 2);

        for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
        {
            const cv::DMatch& match1 = all_matches2[m_idx][0];
            const cv::DMatch& match2 = all_matches2[m_idx][1];

            double ratio =  match1.distance / match2.distance;

            // remove bad matches - ratio test, valid keypoints
            if (ratio < matcher_max_desc_ratio_)
            {
                int idx_q = match1.queryIdx;
                int idx_t = match1.trainIdx;

                if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
                    candidate_matches.push_back(match1);
            }
        }
    }
    else
    {
        DMatchVector all_matches;

        matcher.match(
                    frame_q.descriptors, all_matches);

        for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
        {
            const cv::DMatch& match = all_matches[m_idx];

            // remove bad matches - descriptor distance, valid keypoints
            if (match.distance < matcher_max_desc_dist_)
            {
                int idx_q = match.queryIdx;
                int idx_t = match.trainIdx;

                if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
                    candidate_matches.push_back(match);
            }
        }
    }
}

// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatching(
        int kf_idx_q, int kf_idx_t,
        const KeyframeVector& keyframes,
        DMatchVector& best_inlier_matches,
        Eigen::Matrix4f& best_transformation)
{
    int iterations;

    if (pairwise_matching_method_ == PAIRWISE_MATCHING_RANSAC)
    {
        iterations = pairwiseMatchingRANSAC(
                    kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, best_transformation);
    }
    else if (pairwise_matching_method_ == PAIRWISE_MATCHING_BFSAC)
    {
        iterations = pairwiseMatchingBFSAC(
                    kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, best_transformation);
    }

    return iterations;
}

// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingBFSAC(
        int kf_idx_q, int kf_idx_t,
        const KeyframeVector& keyframes,
        DMatchVector& best_inlier_matches,
        Eigen::Matrix4f& best_transformation)
{
    // constants
    int min_sample_size = 3;

    const RGBDFrame& frame_t = keyframes[kf_idx_t];
    const RGBDFrame& frame_q = keyframes[kf_idx_q];
    cv::FlannBasedMatcher& matcher = matchers_[kf_idx_t];

    // find candidate matches
    DMatchVector candidate_matches;
    getCandidateMatches(frame_q, frame_t, matcher, candidate_matches);
    if (candidate_matches.size() < min_sample_size) return 0;

    // **** build 3D features for SVD ********************************

    PointCloudFeature features_t, features_q;

    features_t.resize(candidate_matches.size());
    features_q.resize(candidate_matches.size());

    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
        const cv::DMatch& match = candidate_matches[m_idx];
        int idx_q = match.queryIdx;
        int idx_t = match.trainIdx;

        PointFeature& p_t = features_t[m_idx];
        p_t.x = frame_t.kp_means[idx_t](0,0);
        p_t.y = frame_t.kp_means[idx_t](1,0);
        p_t.z = frame_t.kp_means[idx_t](2,0);

        PointFeature& p_q = features_q[m_idx];
        p_q.x = frame_q.kp_means[idx_q](0,0);
        p_q.y = frame_q.kp_means[idx_q](1,0);
        p_q.z = frame_q.kp_means[idx_q](2,0);
    }

    // **** main BFSAC loop ****************************************

    TransformationEstimationSVD svd;
    Eigen::Matrix4f transformation; // transformation used inside loop
    best_inlier_matches.clear();
    int iterations = 0;

    for (int i = 0;   i < candidate_matches.size(); ++i)
        for (int j = i+1; j < candidate_matches.size(); ++j)
            for (int k = j+1; k < candidate_matches.size(); ++k)
            {
                // build the Minimum Sample Set of 3 matches (index vector)
                IntVector inlier_idx;
                inlier_idx.push_back(i);
                inlier_idx.push_back(j);
                inlier_idx.push_back(k);

                // build the Minimum Sample Set of 3 matches (actual matches)
                std::vector<cv::DMatch> inlier_matches;
                inlier_matches.push_back(candidate_matches[i]);
                inlier_matches.push_back(candidate_matches[j]);
                inlier_matches.push_back(candidate_matches[k]);

                // estimate transformation from minimum set of random samples
                svd.estimateRigidTransformation(
                            features_q, inlier_idx,
                            features_t, inlier_idx,
                            transformation);

                // evaluate transformation fitness by checking distance to all points
                PointCloudFeature features_q_tf;
                pcl::transformPointCloud(features_q, features_q_tf, transformation);

                for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
                {
                    // euclidedan distance test
                    const PointFeature& p_t = features_t[m_idx];
                    const PointFeature& p_q = features_q_tf[m_idx];
                    float eucl_dist_sq = distEuclideanSq(p_t, p_q);

                    if (eucl_dist_sq < sac_max_eucl_dist_sq_)
                    {
                        inlier_idx.push_back(m_idx);
                        inlier_matches.push_back(candidate_matches[m_idx]);

                        // reestimate transformation from all inliers
                        if (sac_reestimate_tf_)
                        {
                            svd.estimateRigidTransformation(
                                        features_q, inlier_idx,
                                        features_t, inlier_idx,
                                        transformation);
                            pcl::transformPointCloud(features_q, features_q_tf, transformation);
                        }
                    }
                }

                // check if inliers are better than the best model so far
                if (inlier_matches.size() > best_inlier_matches.size())
                {
                    svd.estimateRigidTransformation(
                                features_q, inlier_idx,
                                features_t, inlier_idx,
                                transformation);

                    best_transformation = transformation;
                    best_inlier_matches = inlier_matches;
                }

                iterations++;
            }

    return iterations;
}  

// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingRANSAC(
        int kf_idx_q, int kf_idx_t,
        const KeyframeVector& keyframes,
        DMatchVector& best_inlier_matches,
        Eigen::Matrix4f& best_transformation)
{
    // constants
    int min_sample_size = 3;

    // find candidate matches
    const RGBDFrame& frame_t = keyframes[kf_idx_t];
    const RGBDFrame& frame_q = keyframes[kf_idx_q];
    cv::FlannBasedMatcher& matcher = matchers_[kf_idx_t];

    DMatchVector candidate_matches;
    getCandidateMatches(frame_q, frame_t, matcher, candidate_matches);

    // check if enough matches are present
    if (candidate_matches.size() < min_sample_size)  return 0;
    if (candidate_matches.size() < sac_min_inliers_) return 0;

    // **** build 3D features for SVD ********************************

    PointCloudFeature features_t, features_q;

    features_t.resize(candidate_matches.size());
    features_q.resize(candidate_matches.size());

    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
        const cv::DMatch& match = candidate_matches[m_idx];
        int idx_q = match.queryIdx;
        int idx_t = match.trainIdx;

        PointFeature& p_t = features_t[m_idx];
        p_t.x = frame_t.kp_means[idx_t](0,0);
        p_t.y = frame_t.kp_means[idx_t](1,0);
        p_t.z = frame_t.kp_means[idx_t](2,0);

        PointFeature& p_q = features_q[m_idx];
        p_q.x = frame_q.kp_means[idx_q](0,0);
        p_q.y = frame_q.kp_means[idx_q](1,0);
        p_q.z = frame_q.kp_means[idx_q](2,0);
    }

    // **** main RANSAC loop ****************************************

    TransformationEstimationSVD svd;
    Eigen::Matrix4f transformation; // transformation used inside loop
    best_inlier_matches.clear();
    int iteration = 0;

    std::set<int> mask;

    while(true)
        //for (iteration = 0; iteration < ransac_max_iterations_; ++iteration)
    {
        // generate random indices
        IntVector sample_idx;
        get3RandomIndices(candidate_matches.size(), mask, sample_idx);

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
                    features_q, inlier_idx,
                    features_t, inlier_idx,
                    transformation);

        // evaluate transformation fitness by checking distance to all points
        PointCloudFeature features_q_tf;
        pcl::transformPointCloud(features_q, features_q_tf, transformation);

        for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
        {
            // euclidedan distance test
            const PointFeature& p_t = features_t[m_idx];
            const PointFeature& p_q = features_q_tf[m_idx];
            float eucl_dist_sq = distEuclideanSq(p_t, p_q);

            if (eucl_dist_sq < sac_max_eucl_dist_sq_)
            {
                inlier_idx.push_back(m_idx);
                inlier_matches.push_back(candidate_matches[m_idx]);

                // reestimate transformation from all inliers
                if (sac_reestimate_tf_)
                {
                    svd.estimateRigidTransformation(
                                features_q, inlier_idx,
                                features_t, inlier_idx,
                                transformation);
                    pcl::transformPointCloud(features_q, features_q_tf, transformation);
                }
            }
        }

        // check if inliers are better than the best model so far
        if (inlier_matches.size() > best_inlier_matches.size())
        {
            svd.estimateRigidTransformation(
                        features_q, inlier_idx,
                        features_t, inlier_idx,
                        transformation);

            best_transformation = transformation;
            best_inlier_matches = inlier_matches;
        }

        double best_inlier_ratio = (double) best_inlier_matches.size() /
                (double) candidate_matches.size();

        // **** termination: iterations + inlier ratio
        if(best_inlier_matches.size() < sac_min_inliers_)
        {
            if (iteration >= ransac_max_iterations_) break;
        }
        // **** termination: confidence ratio test
        else
        {
            double h = log_one_minus_ransac_confidence_ /
                    log(1.0 - pow(best_inlier_ratio, min_sample_size));

            if (iteration > (int)(h+1)) break;
            if (iteration >= 2*ransac_max_iterations_) break; //I added
        }

        iteration++;
    }

    return iteration;
}
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingRANSAC(
        int kf_idx_q, int kf_idx_t,
        const KeyframeVector& keyframes,
        DMatchVector& best_inlier_matches,
        Eigen::Matrix4f& best_transformation,std::vector<cv::DMatch> matches)
{
    // constants
    int min_sample_size = 3;

    // find candidate matches
    const RGBDFrame& frame_t = keyframes[kf_idx_t];
    const RGBDFrame& frame_q = keyframes[kf_idx_q];

    DMatchVector candidate_matches;

    for (u_int i = 0; i< matches.size();i++) {
        int idx_q = matches[i].queryIdx;
        int idx_t = matches[i].trainIdx;
        if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
            candidate_matches.push_back(matches[i]);
    }
    // check if enough matches are present
    if (candidate_matches.size() < sac_min_inliers_)
        return 0;
    if (candidate_matches.size() < min_sample_size)
        return 0;


    // **** build 3D features for SVD ********************************

    PointCloudFeature features_t, features_q;

    features_t.resize(candidate_matches.size());
    features_q.resize(candidate_matches.size());

    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
        const cv::DMatch& match = candidate_matches[m_idx];
        int idx_q = match.queryIdx;
        int idx_t = match.trainIdx;

        PointFeature& p_t = features_t[m_idx];
        p_t.x = frame_t.kp_means[idx_t](0,0);
        p_t.y = frame_t.kp_means[idx_t](1,0);
        p_t.z = frame_t.kp_means[idx_t](2,0);

        PointFeature& p_q = features_q[m_idx];
        p_q.x = frame_q.kp_means[idx_q](0,0);
        p_q.y = frame_q.kp_means[idx_q](1,0);
        p_q.z = frame_q.kp_means[idx_q](2,0);
    }

    // **** main RANSAC loop ****************************************

    TransformationEstimationSVD svd;
    Eigen::Matrix4f transformation; // transformation used inside loop
    best_inlier_matches.clear();
    int iteration = 0;

    std::set<int> mask;

    while(true)
        //for (iteration = 0; iteration < ransac_max_iterations_; ++iteration)
    {
        // generate random indices
        IntVector sample_idx;
        get3RandomIndices(candidate_matches.size(), mask, sample_idx);

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
                    features_q, inlier_idx,
                    features_t, inlier_idx,
                    transformation);

        // evaluate transformation fitness by checking distance to all points
        PointCloudFeature features_q_tf;
        pcl::transformPointCloud(features_q, features_q_tf, transformation);

        for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
        {
            // euclidedan distance test
            const PointFeature& p_t = features_t[m_idx];
            const PointFeature& p_q = features_q_tf[m_idx];
            float eucl_dist_sq = distEuclideanSq(p_t, p_q);

            if (eucl_dist_sq < sac_max_eucl_dist_sq_)
            {
                inlier_idx.push_back(m_idx);
                inlier_matches.push_back(candidate_matches[m_idx]);

                // reestimate transformation from all inliers
                if (sac_reestimate_tf_)
                {
                    svd.estimateRigidTransformation(
                                features_q, inlier_idx,
                                features_t, inlier_idx,
                                transformation);
                    pcl::transformPointCloud(features_q, features_q_tf, transformation);
                }
            }
        }

        // check if inliers are better than the best model so far
        if (inlier_matches.size() > best_inlier_matches.size())
        {
            svd.estimateRigidTransformation(
                        features_q, inlier_idx,
                        features_t, inlier_idx,
                        transformation);

            best_transformation = transformation;
            best_inlier_matches = inlier_matches;
        }

        double best_inlier_ratio = (double) best_inlier_matches.size() /
                (double) candidate_matches.size();

        // **** termination: iterations + inlier ratio
        if(best_inlier_matches.size() < sac_min_inliers_)
        {
            if (iteration >= ransac_max_iterations_) break;
        }
        // **** termination: confidence ratio test
        else
        {
            double h = log_one_minus_ransac_confidence_ /
                    log(1.0 - pow(best_inlier_ratio, min_sample_size));

            if (iteration > (int)(h+1)) break;
            if (iteration >= 2*ransac_max_iterations_) break; //I added

        }

        iteration++;
    }

    return iteration;
}

} // namespace rgbdtools

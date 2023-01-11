#ifndef MONO_SLAM_FEATURE_DETECTOR_H
#define MONO_SLAM_FEATURE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <variant>
#include "frame.h"

namespace mono
{
    // tune ORB params
    struct ORBDetectorParams
    {
        float scale_factor            = 1.2f; // scale images 
        int n_levels                  = 8;
        int edge_threshold            = 0;  // Since we are passing patches border not required
        int first_level               = 0;
        int WTA_K                     = 0;  // No BRIEF descriptors for ORB features
        cv::ORB::ScoreType score_type = cv::ORB::HARRIS_SCORE;
        int brief_patch_size          = 20;
        int fast_threshold            = 10;
    };

    // tune FAST params
    struct FASTDetectorParams
    {
        int fast_threshold                         = 10; 
        bool non_maximal_suppresion                = true;
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
    };

      struct DetectorParams
    {
        std::variant<ORBDetectorParams, FASTDetectorParams> params;
        //! Max number of features per image = 10 * 100 = 1000
        //! Atleast 5 pixels width between consecutive features detected
        unsigned int max_feats_per_grid     = 10;
        cv::Size grid_size                  = cv::Size(10, 10);
        int min_dist_bewteen_curr_new_feats = 5;
    };

    class FeatureDetector
    {
       public:
        FeatureDetector(const DetectorParams &detector_params);
        // FeatureDetector(FeatureDetector &&)      = default;
        // FeatureDetector(const FeatureDetector &) = default;
        // FeatureDetector &operator=(FeatureDetector &&) = delete;
        // FeatureDetector &operator=(const FeatureDetector &) = delete;
        // ~FeatureDetector()                                  = default;

        void detect(Frame &frame, const cv::Mat &mask);

        DetectorParams detector_params_;

        std::shared_ptr<cv::Feature2D> feature_detector_;
    };
}
    
#endif
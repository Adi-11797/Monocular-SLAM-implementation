#ifndef MONO_SLAM_FRAME_H
#define MONO_SLAM_FRAME_H

#include <gtsam/geometry/Pose3.h>
#include <utils.h>
#include <opencv2/opencv.hpp>

namespace mono
{

    // detected feature initialized with invalid Track ID = -1
    // on successful tracking, assign incrementally generated track ID to the feature
    using TrackId  = long long int; 
    using Keypoint = cv::Point2f; // get features for tracking
    using Feature  = std::pair<Keypoint, TrackId>;
    using Features = std::vector<Feature>;
    // create a status log for features 
    enum class FeatureStatus
    {
        INVALID = -1,
        TRACKING = 0,
        TERMINATED = 1
    };
    using FeatureStatuses = std::vector<FeatureStatus>;

    // for single frame from image
    class Frame
    {
        public:
        Frame(){};
        Frame(Frame &&)      = default;
        Frame(const Frame &) = default;
        Frame &operator=(Frame &&) = default;
        Frame &operator=(const Frame &) = default;
        ~Frame()                        = default;
        // details of image frame
        utils::Index index;
        utils::Timestamp timestamp;
        cv::Mat image;
        // details on features 
        FeatureStatuses feature_statuses;
        Features features;
        std::vector<size_t> feature_ages;
        // get initial estimate of frame
        gtsam::Pose3 frame_T_g;
    };
}

#endif
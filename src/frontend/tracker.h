#ifndef MONO_SLAM_TRACKER_H
#define MONO_SLAM_TRACKER_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <optional>
#include "frame.h"


namespace mono
{
    struct TrackerParams
    {   // Information used during tracking
        //image size
        cv::Size image_size;
        // time for which a feature will be tracked
        size_t max_feature_age;
        // rotation tolerance
        double rotation_tolerance;
        // Window size of the Kanade Lucas Tomasi Tracker
        cv::Size klt_window_size;
        // KLT optimisation parameters
        int klt_max_iter;
        int klt_max_level;
        double klt_convergence_eps;
        double klt_min_eig_thresh;
        // Ransac Parameters
        double outlier_ransac_prob;
        double outlier_ransac_threshold;
    };
    class FeatureTracker
    {
       public:
        // Pass the camera calibration matrix and the Tracker Parameters
        FeatureTracker(const Eigen::Matrix3d &K, const TrackerParams &params);
        
        gtsam::Pose3 track(Frame &prev_frame, Frame &curr_frame, std::optional<gtsam::Rot3> camera2_R_camera1);

        cv::Mat visualizeTracks(const Frame &prev_frame, const Frame &curr_frame);

       private:
        std::vector<Keypoint> predictOpticalFlow(const std::vector<Keypoint> &prev_keypoints, const gtsam::Rot3 &camera2_R_camera1);

        cv::Matx33d rejectOutliers(const std::vector<Keypoint> &curr_keypoints, const std::vector<Keypoint> &prev_keypoints, std::vector<uchar> &status);

        Eigen::Matrix3d K_;
        Eigen::Matrix3d K_inverse_;
        cv::Matx33d K_cv_;
        cv::Matx33d K_inverse_cv_;
        TrackerParams params_;
    };
}

#endif

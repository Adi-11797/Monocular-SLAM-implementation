#include "tracker.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include "frame.h"

namespace mono{
    // Feature Tracker constructor
    FeatureTracker::FeatureTracker(const Eigen::Matrix3d& K, const TrackerParams& params) : K_(K), K_inverse_(K.inverse()), params_(params)
    {
        //convert K from Eigen Tensors to OpenCV matrices
        cv::eigen2cv(K_, K_cv_);
        cv::eigen2cv(K_inverse_, K_inverse_cv_);
    }
    // Optical Flow prediction: Take in previous keypoints and rotation matrix
    std::vector<Keypoint> FeatureTracker::predictOpticalFlow(const std::vector<Keypoint>& prev_keypoints, const gtsam::Rot3& camera2_R_camera1){
        // get axis angle
        const auto& camera1_R_camera2 = camera2_R_camera1.inverse();
        std::pair<gtsam::Unit3, double> axis_angle = camera1_R_camera2.axisAngle();

        if(axis_angle.second < params_.rotation_tolerance){
            return prev_keypoints;
        }
        // Calculate Homography
        Eigen::Matrix3d H = K_ * camera1_R_camera2.matrix() * K_inverse_;

        cv::Matx33d H_cv;
        cv::eigen2cv(H,H_cv);

        // Use new object in case next keypoints are pointing to previous keypoints
        std::vector<Keypoint> new_keypoints;
        new_keypoints.reserve(prev_keypoints.size());

        for(size_t i =0; i < prev_keypoints.size(); i++){
            const Keypoint& kp = prev_keypoints[i];
            // create homogeneous features
            cv::Vec3f p1(kp.x, kp.y, 1.0f);
            // Project into new image coordinates
            cv::Vec3f p2 = cv::Matx33f(H_cv) * p1;

            //Project predicted bearing vectors to 2D again and re-homogenize.
            if(p2[2] > 0.0f){
                //get new keypoint coordinates
                Keypoint new_kp = Keypoint(p2[0] / p2[2], p2[1]/p2[2]);
                if (new_kp.x >= 0 && new_kp.x <= static_cast<float>(params_.image_size.width) && new_kp.y >= 0 &&
                    new_kp.y <= static_cast<float>(params_.image_size.height))
                {
                    new_keypoints.emplace_back(new_kp);
                }
                else
                {
                    new_keypoints.push_back(kp);
                }
            }
        }
        return new_keypoints;
    }


    cv::Matx33d FeatureTracker::rejectOutliers(const std::vector<Keypoint> &curr_keypoints, const std::vector<Keypoint>& prev_keypoints, std::vector<uchar>& status){
        if(prev_keypoints.size()<5 || curr_keypoints.size() < 5){
            status.resize(prev_keypoints.size(), static_cast<uchar>(0));
        }
        std::vector<Keypoint> tracked_prev_keypoints;
        std::vector<Keypoint> tracked_curr_keypoints;
        std::vector<size_t> tracked_status_indices;
        for(size_t i = 0; i< status.size(); i++){
            if(status[i]){
                tracked_prev_keypoints.push_back(prev_keypoints[i]);
                tracked_curr_keypoints.push_back(curr_keypoints[i]);
                tracked_status_indices.push_back(i);
            }
        }

        std::vector<uchar> inliers;
        // given the tracked points, the calibration matrix, find the essential matrix
        cv::Matx33d essential_mat = cv::findEssentialMat(tracked_curr_keypoints, tracked_prev_keypoints, K_cv_, cv::RANSAC, params_.outlier_ransac_prob, params_.outlier_ransac_threshold, inliers);
        
        for (size_t i = 0; i < inliers.size(); ++i)
        {
            status[tracked_status_indices[i]] = inliers[i];
        }
        return essential_mat;
    }
    // std::optional
    gtsam::Pose3 FeatureTracker::track(Frame &prev_frame, Frame& curr_frame, std::optional<gtsam::Rot3> camera2_R_camera1){
        if(prev_frame.features.size()<= 0){
            return gtsam::Pose3();
        }
        std::vector<Keypoint> valid_prev_keypoints;
        std::vector<size_t> valid_prev_feature_ages;
        std::vector<size_t> valid_feature_ids;

        valid_prev_keypoints.reserve(prev_frame.features.size());
        valid_prev_feature_ages.reserve(prev_frame.features.size());
        valid_feature_ids.reserve(prev_frame.features.size());

        for(size_t i = 0; i< prev_frame.features.size(); i++){
            if(prev_frame.feature_statuses[i] == FeatureStatus::TRACKING){
                valid_feature_ids.push_back(i);
                valid_prev_keypoints.push_back(prev_frame.features[i].first);
                valid_prev_feature_ages.push_back(prev_frame.feature_ages[i]);
            }
        }
        
        // Predict initialization and store in curr_features for KLT tracking
        std::vector<Keypoint> curr_keypoints;
        if(camera2_R_camera1.has_value()){
            curr_keypoints = predictOpticalFlow(valid_prev_keypoints, camera2_R_camera1.value());
        }
        else{
            curr_keypoints = valid_prev_keypoints;
        }

        //KLT Tracking
        std::vector<uchar> status;
        std::vector<float> error;
        cv::calcOpticalFlowPyrLK(
            prev_frame.image,
            curr_frame.image,
            valid_prev_keypoints,
            curr_keypoints,
            status,
            error,
            params_.klt_window_size,
            params_.klt_max_level,
            cv::TermCriteria(
                cv::TermCriteria::COUNT + cv::TermCriteria::EPS, params_.klt_max_iter, params_.klt_convergence_eps),
            cv::OPTFLOW_USE_INITIAL_FLOW,
            params_.klt_min_eig_thresh);
        
        // // Recover relative pose from essential matrix and inliers of tracked freatures using Ransac
        // cv::Matx33d rotation_cv;
        // cv::Vec3d translation_cv;
        // cv::Matx33d essential_mat_cv = rejectOutliers(curr_keypoints, valid_prev_keypoints, status);
        // cv::recoverPose(essential_mat_cv, valid_prev_keypoints, curr_keypoints, K_cv_, rotation_cv, translation_cv);

        // //////////////////////////////
        cv::Matx33d rotation_cv;
        cv::Vec3d translation_cv;
        cv::Matx33d essential_mat_cv = rejectOutliers(curr_keypoints, valid_prev_keypoints, status);
        cv::recoverPose(essential_mat_cv, valid_prev_keypoints, curr_keypoints, K_cv_, rotation_cv, translation_cv);

        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        cv::cv2eigen(rotation_cv, rotation);
        cv::cv2eigen(translation_cv, translation);

        gtsam::Pose3 camera2_T_camera1(gtsam::Rot3(rotation), translation);

        //! Find maximum track_id to start with for this frame
        auto it          = std::max_element(prev_frame.features.begin(),
                                   prev_frame.features.end(),
                                   [](const Feature& a, const Feature& b) { return a.second < b.second; });
        TrackId track_id = it->second;

        //! Store successfully tracked features in curr frame
        //! Write feature status in prev frame if terminated
        for (size_t i = 0; i < status.size(); ++i)
        {
            size_t prev_frame_feature_id = valid_feature_ids[i];
            const Feature& prev_feature  = prev_frame.features[prev_frame_feature_id];

            if (!status[i] || valid_prev_feature_ages[i] >= params_.max_feature_age)
            {
                prev_frame.feature_statuses[prev_frame_feature_id] = FeatureStatus::TERMINATED;
                continue;
            }

            if (prev_feature.second != -1)
            {
                //! Use the same track ID as the prev frame feature if ID already exists
                curr_frame.features.emplace_back(curr_keypoints[i], prev_feature.second);
            }
            else
            {
                //! Increment a new track ID if newly tracked
                curr_frame.features.emplace_back(curr_keypoints[i], ++track_id);
            }
            curr_frame.feature_statuses.push_back(FeatureStatus::TRACKING);
            curr_frame.feature_ages.push_back(++valid_prev_feature_ages[i]);
        }
        return camera2_T_camera1;
    }


    cv::Mat FeatureTracker::visualizeTracks(const Frame& ref_frame, const Frame& curr_frame)
    {
        cv::Scalar red(0, 0, 255);
        cv::Scalar blue(255, 0, 0);
        cv::Scalar green(0, 255, 0);

        cv::Mat tracker_image = curr_frame.image.clone();

        for (size_t i = 0; i < curr_frame.features.size(); ++i)
        {
            const auto& curr_feature = curr_frame.features.at(i);
            if (curr_frame.feature_statuses.at(i) != FeatureStatus::TRACKING)
            {
                cv::circle(tracker_image, curr_feature.first, 2, red, 1);
                continue;
            }

            //! This iterates over the entire reference frame feature track ids
            const auto& it = std::find_if(ref_frame.features.begin(),
                                          ref_frame.features.end(),
                                          [&curr_feature](const Feature& a) { return curr_feature.second == a.second; });
            if (it == ref_frame.features.end() || it->second == -1)
                continue;

            cv::circle(tracker_image, curr_feature.first, 4, blue, 1);
            cv::arrowedLine(tracker_image, it->first, curr_feature.first, green);
        }
        return tracker_image;
    }
}
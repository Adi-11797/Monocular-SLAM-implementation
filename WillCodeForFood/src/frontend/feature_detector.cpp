#include "feature_detector.h"
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

namespace mono
{
    // init feature detector
    FeatureDetector::FeatureDetector(const DetectorParams& detector_params) : detector_params_(detector_params)
    {
        // orb mode
        if (std::holds_alternative<ORBDetectorParams>(detector_params_.params))
        {
            // get orb params
            auto orb_params   = std::get<ORBDetectorParams>(detector_params_.params);
            // init 
            feature_detector_ = cv::ORB::create(static_cast<int>(detector_params_.max_feats_per_grid),
                                                orb_params.scale_factor,
                                                orb_params.n_levels,
                                                orb_params.edge_threshold,
                                                orb_params.first_level,
                                                orb_params.WTA_K,
                                                orb_params.score_type,
                                                orb_params.brief_patch_size,
                                                orb_params.fast_threshold);
        }
        // fast mode
        else if (std::holds_alternative<FASTDetectorParams>(detector_params_.params))
        {
            // get fast params
            auto fast_params  = std::get<FASTDetectorParams>(detector_params_.params);
            // init 
            feature_detector_ = cv::FastFeatureDetector::create(fast_params.fast_threshold, fast_params.non_maximal_suppresion, fast_params.type);
        }
    }

    // detect features
    void FeatureDetector::detect(Frame& frame, const cv::Mat& mask)
    {
        // init states
        auto& image            = frame.image;
        auto& current_features = frame.features;
        auto& current_statuses = frame.feature_statuses;
        auto& current_ages     = frame.feature_ages;
        Features features;

        // init mask
        cv::Mat feature_mask;
        if (mask.empty())
        {
            feature_mask = cv::Mat(frame.image.size(), CV_8UC1, cv::Scalar(255));
        }
        else
        {
            feature_mask = mask.clone();
        }

        // specify image size
        cv::Size patch_size(image.cols / detector_params_.grid_size.width, image.rows / detector_params_.grid_size.height);

        int x_start = (image.cols % patch_size.width) / 2;
        int x_stop  = image.cols - patch_size.width;
        int y_start = (image.rows % patch_size.height) / 2;
        int y_stop  = image.rows - patch_size.height;

        cv::Mat cells(detector_params_.grid_size, CV_32S, cv::Scalar(0));

        // create mask to ignore features in areas where features already exist
        for (const auto& feat : current_features)
        {
            const auto& kp = feat.first;
            auto x_startf  = static_cast<float>(x_start);
            auto y_startf  = static_cast<float>(y_start);
            if (kp.x >= x_startf && kp.y >= y_startf)
            {
                int x = static_cast<int>((kp.x - x_startf) / static_cast<float>(patch_size.width));
                int y = static_cast<int>((kp.y - y_startf) / static_cast<float>(patch_size.height));
                
                // draw keypoint over mask
                cv::circle(feature_mask, kp, detector_params_.min_dist_bewteen_curr_new_feats, cv::Scalar(0), -1);

                if ((x >= 0) && (x < detector_params_.grid_size.width) && (y >= 0) &&
                    (y < detector_params_.grid_size.height))
                {
                    cells.at<int>(y, x) += 1;
                }
            }
        }



        // divide the image into a grid and detect features individually in each patch
        // do not parallelize, as we need to compact the vector after filling it.
        Features new_features;
        for (int x = x_start; x <= x_stop; x += patch_size.width)
        {
            for (int y = y_start; y <= y_stop; y += patch_size.height)
            {
                // create rect patch on image for tracking
                cv::Rect rect(x, y, patch_size.width, patch_size.height);
                cv::Mat image_crop = image(rect);

                // split into grid cells
                int cell_y = (y - y_start) / patch_size.height;
                int cell_x = (x - x_start) / patch_size.width;

                int threshold = patch_size.width * patch_size.width / 10;

                while (cells.at<uint>(cell_y, cell_x) < detector_params_.max_feats_per_grid && threshold >= 10)
                {
                    // init detected keypoints
                    std::vector<cv::KeyPoint> detected_keypoints;
                    // detect keypoints from image section
                    feature_detector_->detect(image_crop, detected_keypoints);
                    // sort keypoints by response
                    std::sort(detected_keypoints.begin(),
                              detected_keypoints.end(),
                              [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool { return a.response > b.response; });

                    for (size_t i = 0; i < detected_keypoints.size() &&
                                       cells.at<unsigned int>(cell_y, cell_x) < detector_params_.max_feats_per_grid;
                         i++)
                    {
                        // get global pos of detected keypoints
                        float x_global = float(x) + detected_keypoints[i].pt.x;
                        float y_global = float(y) + detected_keypoints[i].pt.y;

                        if (feature_mask.at<uint>(int(y_global), int(x_global)) > 0)
                        {
                            Keypoint kp(x_global, y_global);
                            // draw keypoints
                            cv::circle(
                                feature_mask, kp, detector_params_.min_dist_bewteen_curr_new_feats, cv::Scalar(0), -1);
                            // add a new feature with keypoint kp, and invalid track ID
                            new_features.emplace_back(Feature(kp, -1));
                            cells.at<int>(cell_y, cell_x)++;
                        }
                    }
                    threshold /= 2;
                }
            }
        }

        // store features
        FeatureStatuses new_statuses(new_features.size(), FeatureStatus::TRACKING);
        std::vector<size_t> new_ages(new_features.size(), 0);
        current_features.insert(std::end(current_features), std::begin(new_features), std::end(new_features));
        current_statuses.insert(std::end(current_statuses), std::begin(new_statuses), std::end(new_statuses));
        current_ages.insert(std::end(current_ages), std::begin(new_ages), std::end(new_ages));
    }
}  // namespace mono
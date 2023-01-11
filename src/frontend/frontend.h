#ifndef MONO_SLAM_FRONTEND_H
#define MONO_SLAM_FRONTEND_H

#include <tbb/tbb.h>
#include <thread>
#include <unordered_set>
#include "../backend/backend.h"
#include "feature_track.h"
#include "feature_detector.h"
#include "frame.h"
#include "tracker.h"

namespace mono
{
    struct FrontendParams{
        DetectorParams detector_params;
        TrackerParams tracker_params;
        Eigen::Matrix3d K;
        double keyframe_unique_quality;
        double keyframe_match_quality;
    };//Test-case here

    class Frontend
    {           
        public:
            // Check copy and move constructors to be completely sure
            // also rvalue operators
            // Stores the params
            FrontendParams params_;
           // initialised as a thread to run the frontend class
            std::unique_ptr<std::thread> process_thread_;
            std::pair<size_t, size_t> searchFeaturesInTracks(const Frame &frame);
            // Pointers to the objects of FeatureDetector class and FeatureTracker class
            std::unique_ptr<FeatureDetector> feature_detector_;
            std::unique_ptr<FeatureTracker> feature_tracker_;
            // Pointer to the previous frame
            std::shared_ptr<Frame> prev_frame;
            // set of all time_stamps
            std::unordered_set<utils::Timestamp> keyframe_timestamps_;
            // Tracked Features over scenes
            FeatureTracks feature_tracks_;
            // Queue that contains pointers to the input frames
            tbb::concurrent_bounded_queue<std::shared_ptr<Frame>> input_queue_;
            // Queue that contains the pointer to the output images to be sent to pangolin
            tbb::concurrent_bounded_queue<std::shared_ptr<cv::Mat>> output_vis_queue_;
            // Queue that contains the information to be sent to the backend
            tbb::concurrent_bounded_queue<std::shared_ptr<BackendInput>> *backend_queue_ = nullptr;
            
            Frontend(const FrontendParams &params);
            // initialise the process thread
            inline void init(){
                process_thread_ = std::make_unique<std::thread>(&Frontend::run, this);    
            }
            // joining the frontend thread
            inline void join(){
                if(process_thread_->joinable()){
                    process_thread_->join();
                }
            }
            void run();
            void processFrame(const std::shared_ptr<Frame> &frame);
            
            bool checkFrameQuality(const Frame &frame);
            void addTrackedFeatures(const Frame &frame);
    };
}

#endif
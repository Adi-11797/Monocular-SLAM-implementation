
#include "frontend.h"

namespace mono{

    Frontend::Frontend(const FrontendParams& params) : params_(params)
    {
        // Initialise the object of class FeatureDetector and FeatureTracker
        feature_detector_ = std::make_unique<FeatureDetector>(params.detector_params);
        feature_tracker_ = std::make_unique<FeatureTracker>(params.K, params.tracker_params);
    }

    void Frontend::run(){
        while(true){
            // create a shared pointer to the Frame class
            std::shared_ptr<Frame> frame;
            // pop the first frame in the input queue in to the Frame object created above
            input_queue_.pop(frame);
            if(frame.get()){
                // While the frame object has information process it
                this->processFrame(frame);
            }
            else{
                break;
            }
        }
        output_vis_queue_.push(nullptr);
        backend_queue_->push(nullptr);
    }

    //Processing each frame
    void Frontend::processFrame(const std::shared_ptr<Frame>& frame){
        // Get a reference to the current frame
        Frame& curr_frame = *frame;
        
        gtsam::Pose3& currframe_T_g = curr_frame.frame_T_g;

        // Pose estimate of the previous frame       
        gtsam::Pose3 currframe_T_prevframe_estimate;
        // prev_frame class member
        if(prev_frame.get()){
            // Track
            gtsam::Pose3 currframe_T_prevframe = prev_frame->frame_T_g.inverse() * currframe_T_g;
            currframe_T_prevframe_estimate = feature_tracker_->track(*prev_frame, curr_frame, currframe_T_prevframe.rotation());
            //! Update pose estimate for current frame
            /* currframe_T_g = prev_frame->frame_T_g * currframe_T_prevframe_estimate; */

            auto visualize_image = feature_tracker_->visualizeTracks(*prev_frame, curr_frame);
            output_vis_queue_.push(std::make_shared<cv::Mat>(visualize_image));
        }

        //! Modifies curr_frame
        feature_detector_->detect(curr_frame, cv::Mat());

        bool is_keyframe = checkFrameQuality(curr_frame);
        if (is_keyframe)
        {
            keyframe_timestamps_.insert(curr_frame.timestamp);
        }

        //! Add tracked features to FeatureTracks
        addTrackedFeatures(curr_frame);

        //! If keyframe and backend exists send estimate to backend for triangulation + graph optimization
        // TODO: Work with only keyframes
        if (/* is_keyframe && */ backend_queue_){

            std::shared_ptr<BackendInput> backend_input = std::make_shared<BackendInput>();

            //! TODO: We send a full copy to the backend, change to sending a reference
            //! We want the updated pose to reflect in the frame for better initialization
            backend_input->keyframe               = frame;
            FeatureTracks& feature_tracks_to_send = backend_input->feature_tracks;
            backend_input->relative_pose          = currframe_T_prevframe_estimate;

            for (auto& feature_track : feature_tracks_)
            {
                auto& track_id             = feature_track.first;
                auto& feature_observations = feature_track.second;
                //! Send only informative features that can actually be triangulated
                if (feature_observations.features.size() >= 2)
                {
                    //! Move the feature observations from the track ID completely to avoid duplicate measurements
                    feature_tracks_to_send.insert(std::make_pair(track_id, std::move(feature_observations)));
                    feature_observations = FeatureTrack(track_id);
                }
            }
            backend_queue_->push(backend_input);
        }
        //! Store current frame in prev_frame
        prev_frame = frame;
    }

    bool Frontend::checkFrameQuality(const Frame& frame){
        if (frame.index == 0)
        {
            return true;
        }
        bool is_keyframe = false;
        size_t keyframe_matches, frame_matches;
        std::tie(keyframe_matches, frame_matches) = searchFeaturesInTracks(frame);

        //! Check ratio of matched
        //! TODO: Other ways of checking frame quality?
        auto total_matches = keyframe_matches + frame_matches;
        if (total_matches != 0 && feature_tracks_.size() != 0){
            auto keyframe_match_ratio = static_cast<double>(keyframe_matches) / static_cast<double>(total_matches);
            auto total_match_ratio    = static_cast<double>(total_matches) / static_cast<double>(feature_tracks_.size());
			
            if (keyframe_match_ratio <= params_.keyframe_unique_quality &&
                total_match_ratio >= params_.keyframe_match_quality)
                is_keyframe = true;
        }

        //! TODO: Check baseline of frame

        return is_keyframe;
    }

    void Frontend::addTrackedFeatures(const Frame& frame){
        //! TODO: Add to feature-track only if frame is a keyframe?
        for (const auto& feature : frame.features){
            const TrackId& track_id = feature.second;
            if (track_id == -1)
            {
                continue;
            }
            auto it = feature_tracks_.find(track_id);
            //! If Tracked feature was not found in FeatureTracks, add it
            gtsam::Vector2 pix_observation(feature.first.x, feature.first.y);
            if (it == feature_tracks_.end())
            {
                auto feature_track = FeatureTrack(feature.second, frame.index, pix_observation);
                feature_tracks_.insert(std::make_pair(feature.second, feature_track));
            }
            else
            {
                FeatureTrack& feature_track = it->second;
                feature_track.emplace_back(frame.index, pix_observation);
            }
        }
    }

    std::pair<size_t, size_t> Frontend::searchFeaturesInTracks(const Frame& frame){
        size_t keyframe_matches = 0;
        size_t frame_matches    = 0;
        for (const auto& feature : frame.features){
            /* const Keypoint& keypoint = feature.first; */
            const TrackId& track_id = feature.second;
            if (track_id == -1)
            {
                continue;
            }
            auto it = feature_tracks_.find(track_id);
            //! If we found an existing track with same track ID
            if (it != feature_tracks_.end()){
                const FeatureTrack& feature_track = it->second;
                for (const auto& tracked_feature : feature_track.features){
                    //! One or more of the tracked features in current frame is from a keyframe
                    if (keyframe_timestamps_.count(tracked_feature.first))
                    {
                        keyframe_matches++;
                    }
                    else
                    {
                        frame_matches++;
                    }
                }
            }
        }
        return std::make_pair(keyframe_matches, frame_matches);
    }

}

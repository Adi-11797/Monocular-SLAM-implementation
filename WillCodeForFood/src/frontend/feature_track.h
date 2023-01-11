#ifndef MONO_SLAM_FEATURE_TRACK_H
#define MONO_SLAM_FEATURE_TRACK_H

#include "tracker.h"
#include "utils.h"

namespace mono{



    struct FeatureTrack
    {
        // A Feature is a 2D calibrated pixel observation in a camera with timestamp
        using FeatureMeasurement  = std::pair<utils::Index, gtsam::Vector2>;
        using FeatureMeasurements = std::vector<FeatureMeasurement>;

        FeatureTrack(TrackId track_id) : track_id(track_id) {}
        FeatureTrack(TrackId track_id, utils::Index _frame_id, const gtsam::Vector2 &_pix_observation) : track_id(track_id)
        {
            features.emplace_back(_frame_id, _pix_observation);
        }

        inline void emplace_back(utils::Index _frame_id, const gtsam::Vector2 &_pix_observation)
        {
            features.emplace_back(_frame_id, _pix_observation);
        }

        inline size_t length() { return features.size(); }

        TrackId track_id;
        FeatureMeasurements features;
    };

    using FeatureTracks = std::unordered_map<TrackId, FeatureTrack>;

}

#endif
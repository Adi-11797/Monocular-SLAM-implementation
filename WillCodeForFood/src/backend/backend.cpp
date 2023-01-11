#include "backend.h"
#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>

namespace mono{
    Backend::Backend(const BackendParams &backend_params) : backend_state_(utils::BackendState::INITIALIZING), backend_params_(backend_params)
    {
        isam2_ = std::make_unique<gtsam::ISAM2>(backend_params.isam2_params);
        // initialise the noise in the pose
        gtsam::Vector6 pose_sigmas;
        pose_sigmas.head<3>().setConstant(backend_params_.init_rotation_sigma);
        pose_sigmas.tail<3>().setConstant(backend_params_.init_translation_sigma);
        pose_init_noise_ = gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);

        camera_measure_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, backend_params_.smart_projection_sigma);
        
        gtsam::Vector6 between_pose_sigmas;
        between_pose_sigmas.head<3>().setConstant(1e-2);
        between_pose_sigmas.tail<3>().setConstant(1e-2);
        // noise for the between factors
        between_measure_noise_ = gtsam::noiseModel::Diagonal::Sigmas(between_pose_sigmas);
    }

    void Backend::run(){
        // pass each frontend output(frames) one by one to the processInput function
        while(true){
            std::shared_ptr<BackendInput> backend_input;
            input_queue_.pop(backend_input);
            if(backend_input.get()){
                // pass to the processInput function
                this->processInput(backend_input);
            }
            else{
                break;
            }
        }
    }
    // take the backend input, call functions to get the ouput and assign them to the backend output
    // add the output to the output queue
    void Backend::processInput(const std::shared_ptr<BackendInput>& backend_input){
        const BackendInput& curr_packet = *backend_input;

        if(backend_state_ == utils::BackendState::INITIALIZING){
            initialize(backend_input);
        }
        else if(backend_state_ == utils::BackendState::RUNNING){
            update(backend_input);
        }
        BackendOutput backend_output;
        //get point map
        backend_output.point_map = getPointMap(isam2_->getFactorsUnsafe());
        // get trajectory
        backend_output.trajectory = getTrajectory();
        // add the backend_output to the output queue
        output_queue_.push(std::make_shared<BackendOutput>(backend_output));

    }

    // intialize the graph
    void Backend::initialize(const std::shared_ptr<BackendInput>& backend_input){
        new_values_.clear();
        // intialize the keyframe id
        keyframe_id_ = backend_input->keyframe->index;
        // get the time stamp from the keyframe
        timestamp_keyframe_ = backend_input->keyframe->timestamp;
        // get the pose
        keyframe_T_world_ = backend_input->keyframe->frame_T_g;

        // initialise the graph
        gtsam::NonlinearFactorGraph new_factors;
        new_factors.push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>( gtsam::Symbol(POSE_CHAR,keyframe_id_), keyframe_T_world_, pose_init_noise_));

        //add new values
        new_values_.insert(gtsam::Symbol(POSE_CHAR,keyframe_id_), keyframe_T_world_);

        // compute result from isam2
        gtsam::ISAM2Result result = isam2_->update(new_factors, new_values_);

        // Clear new values
        new_values_.clear();

        backend_state_ = utils::BackendState::RUNNING;

    }

    void Backend::update(const std::shared_ptr<BackendInput>& backend_input){
        // take each backend input
        const BackendInput& curr_packet = *backend_input;
        // keyframe shared pointer
        const std::shared_ptr<Frame>& keyframe = curr_packet.keyframe;
        // feature tracks in the new backend input
        const FeatureTracks& incoming_feature_tracks = curr_packet.feature_tracks;

        gtsam::NonlinearFactorGraph new_factors;
        gtsam::FactorIndices delete_factor_indices;
        
        // take feature track in the incoming feature tracks
        for(const auto & feature_track : incoming_feature_tracks){
            // get track_id of the feature
            TrackId track_id = feature_track.first;
            // get the measurements of the feature
            FeatureTrack::FeatureMeasurements observations = feature_track.second.features;
            // if the feature is not present as a landmark
            if(!isLandmarkInGraph(track_id)){
                // create new smart factor, pass the noise, calibration and backend parameters, smart projection parameters
                SmartPinholeProjFactor::shared_ptr new_smart_factor = boost::make_shared<SmartPinholeProjFactor>(camera_measure_noise_,
                boost::make_shared<gtsam::Cal3_S2>(backend_params_.K), backend_params_.smart_projection_params);

                // Now add the observations in the feature to the new_feature
                for(const auto& observation : observations){
                    // add the observation for the feature to the landmark and add the symbol
                    new_smart_factor->add(observation.second, gtsam::symbol(POSE_CHAR, observation.first));
                }
                // add the new_smart_factor to the new_factor
                new_factors.push_back(new_smart_factor);
                // Maintain a reference to the inserted smart factor with invalid factor index to be updated later from
                // ISAM2Result
                smart_factor_map_[track_id] = std::make_pair(new_smart_factor, INVALID_FACTOR_INDEX);
                setLandmarkInGraph(track_id);
            }
            else{
                // if the track_id ie the feature is present as a landmark
                // using SmartFactorMap = std::unordered_map<TrackId, std::pair<SmartPinholeProjFactor::shared_ptr, IndexInGraph>>; 
                // it is a pair of sppfactor and index
                auto& it = smart_factor_map_[track_id];
                // add factor indices to be deleted
                delete_factor_indices.push_back(gtsam::FactorIndex(it.second));
                // make a new smart factor with the information fromt the previous it
                SmartPinholeProjFactor::shared_ptr new_smart_factor = boost::make_shared<SmartPinholeProjFactor>(*it.first);
                // for all the observations in the feature observation
                // and add it to the new_smart_factor
                for(const auto& observation : observations){
                    new_smart_factor ->add(observation.second, gtsam::Symbol(POSE_CHAR, observation.first));
                }
                // add the new_smart_factor to new_factors
                new_factors.push_back(new_smart_factor);
                it.first = new_smart_factor;
                it.second = INVALID_FACTOR_INDEX;
            }
        }
        // add between factors between the two keyframes with the relative pose and the relative measurement
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::symbol(POSE_CHAR, keyframe->index-1),
                                                                       gtsam::symbol(POSE_CHAR, keyframe->index),
                                                                       backend_input->relative_pose,
                                                                       between_measure_noise_ );
        new_values_.insert(gtsam::Symbol(POSE_CHAR,keyframe->index), keyframe->frame_T_g);
        // deleting ?
        for(const auto&index : delete_factor_indices){

        }
        ///////////
        // compute result from isam2
        gtsam::ISAM2Result result = isam2_->update(new_factors, new_values_, delete_factor_indices);

        updateSmartFactorIndices(result);
        state_ = isam2_->calculateBestEstimate();

        // clear new values
        new_values_.clear();
    }

    void Backend::updateSmartFactorIndices(const gtsam::ISAM2Result& result){
        //get the new factor indices from the results
        const gtsam::FactorIndices& new_factor_indices = result.newFactorsIndices;
        // iterate over all newly added factors
        for(const auto& index : new_factor_indices){
            //get the factor
            const auto& factor = isam2_->getFactorsUnsafe().at(index);
            // 
            auto smart_factor = boost::dynamic_pointer_cast<SmartPinholeProjFactor>(factor);
            if (smart_factor == nullptr)
                continue;

            //! Find the map value associated with this smart factor pointer and then change the index
            auto it = std::find_if(smart_factor_map_.begin(),
                                   smart_factor_map_.end(),
                                   [&smart_factor](const SmartFactorMap::value_type& iter)
                                   { return iter.second.first == smart_factor; });
            if (it != smart_factor_map_.end())
            {
                it->second.second = static_cast<IndexInGraph>(index);
            }
        }
    }

    PointMap Backend::getPointMap(const gtsam::NonlinearFactorGraph& graph)
    {
        PointMap point_map;
        //! Iterate over all landmark IDs in the active landmark set and then update 3D points for the same from the graph
        for (const auto& track_id : active_landmark_set_)
        {
            const auto& it = smart_factor_map_.find(track_id);
            if (it == smart_factor_map_.end())
                continue;
            IndexInGraph id_in_graph = it->second.second;

            //! If the track_id does not have a corresponding factor in the isam2 factor graph.
            //! TODO: should this happen at all?
            //! TODO: We need to remove some points from the graph when they are not optimized at all, etc.
            if (id_in_graph == -1)
                continue;
            const auto id = static_cast<gtsam::FactorIndex>(id_in_graph);
            //! This active landmark has no smart factors in the isam2 graph
            if (!graph.exists(id))
                continue;

            const SmartPinholeProjFactor::shared_ptr& factor = it->second.first;
            //! The factor pointers in the isam2 map and local smart factor map do not match
            if (factor != graph.at(id))
                continue;

            gtsam::TriangulationResult result = factor->point();
            if (result.valid())
            {
                //! We add 3D points triangulated with measurements in at least 3 frames
                //! TODO: Change this to a parameter
                if (factor->measured().size() >= 3)
                    point_map.emplace(std::make_pair(track_id, *result));
            }
        }
        return point_map;
    }

    Trajectory Backend::getTrajectory()
    {
        Trajectory trajectory;
        gtsam::Values::Filtered<gtsam::Value> pose_values = state_.filter(gtsam::Symbol::ChrTest(POSE_CHAR));
        for (const auto& it : pose_values)
        {
            const gtsam::Symbol& symbol = it.key;
            const gtsam::Pose3& value   = it.value.cast<gtsam::Pose3>();
            utils::Index id             = static_cast<utils::Index>(symbol.index());
            trajectory.emplace_back(std::make_pair(id, value));
        }
        return trajectory;
    }

}
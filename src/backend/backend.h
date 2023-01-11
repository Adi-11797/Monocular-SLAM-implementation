#ifndef MONO_SLAM_BACKEND_H
#define MONO_SLAM_BACKEND_H

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <tbb/tbb.h>
#include <memory>
#include <thread>
#include <unordered_set>

#include "../frontend/feature_track.h"


namespace mono{
    //Pose at each timestep
    using StampedPose3           = std::pair<utils::Timestamp, gtsam::Pose3>;
    // Factor created from projection
    using SmartPinholeProjFactor = gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>;
    using IndexInGraph           = long int;
    // Map between each track Id and a pair of a shared ptr with shared_ptr and its index
    using SmartFactorMap         = std::unordered_map<TrackId, std::pair<SmartPinholeProjFactor::shared_ptr, IndexInGraph>>;
    // Point map between track id and 3 vector
    using PointMap               = std::unordered_map<TrackId, gtsam::Vector3>;
    // vector containing pair of index and corresponding pose along the trajectory
    using Trajectory             = std::vector<std::pair<utils::Index, gtsam::Pose3>>;

    // constexpr is like const but can be used in other cases as well such as functions
    // Character used in the nonlinear factorgraph for the pose
    static constexpr char POSE_CHAR = 'x';
    // Character used in the factor graph for the object
    static constexpr char OBJECT_CHAR  = 'o';
    static constexpr IndexInGraph INVALID_FACTOR_INDEX = -1;
    
    struct BackendParams{
        // We are using isam2 for the optimisation
        gtsam::ISAM2Params isam2_params;

        void setDefaultISAM2Params(){
            // gaussian parameters for ISAM2
            gtsam::ISAM2GaussNewtonParams gauss_newton_params;
            //Update the Newton's method step point, using wildfire
            gauss_newton_params.wildfireThreshold = 0.001;
            isam2_params.optimizationParams = gauss_newton_params;

            isam2_params.setCacheLinearizedFactors(true);
            // checkRelinearize will find set of variables to be relinearized according to relinearizeThreshold
            isam2_params.relinearizeThreshold = 0.1;
            isam2_params.relinearizeSkip = 1;
            isam2_params.findUnusedFactorSlots = true;
            isam2_params.setEvaluateNonlinearError(false);
            isam2_params.enableDetailedResults = false;
            // Choosing cholesky for factorisation
            isam2_params.factorization = gtsam::ISAM2Params::CHOLESKY;
        }

        // initialising the noise
        double init_rotation_sigma;
        double init_translation_sigma;
        double smart_projection_sigma;

        // Smart Projection only optimises over the pose not the camera matrix
        gtsam::SmartProjectionParams smart_projection_params;

        // GTSAM Camera calibration parameters of the monocular camera considered
        gtsam::Cal3_S2 K;
    };
    
    struct BackendInput
    {
        // Input from the frontend - Frame 
        std::shared_ptr<Frame> keyframe;
        // Tracked features
        FeatureTracks feature_tracks;
        //! TODO: Temporary
        gtsam::Pose3 relative_pose;
    };

    struct BackendOutput
    {   // Final output
        // Trajectory
        Trajectory trajectory;
        // map between track_id and vector3
        PointMap point_map;
    };


    class Backend{
        public:
            // parameters for backend
            BackendParams backend_params_;
            utils::Index keyframe_id_;
            utils::Timestamp timestamp_keyframe_;
            // Keyframe poses
            gtsam::Pose3 keyframe_T_world_;
            
            // Noise models
            gtsam::SharedNoiseModel pose_init_noise_;
            gtsam::SharedNoiseModel camera_measure_noise_;
            gtsam::SharedNoiseModel between_measure_noise_;

            // check if a particular track_id is already in the factor graph
            std::unordered_set<TrackId> active_landmark_set_;
            // Track id to factor index in graph and pointer to factor
            SmartFactorMap smart_factor_map_;

            // Complete state of the SLAM system
            gtsam::Values state_;

            // New incrementally added values in the graph
            gtsam::Values new_values_;

            //ISAM2
            std::unique_ptr<gtsam::ISAM2> isam2_;
            std::unique_ptr<std::thread> process_thread_;
            utils::BackendState backend_state_;
            
            tbb::concurrent_bounded_queue<std::shared_ptr<BackendInput>> input_queue_;
            tbb::concurrent_bounded_queue<std::shared_ptr<BackendOutput>> output_queue_;

            Backend(const BackendParams &backend_params);
            Backend(Backend &&)      = default;
            Backend(const Backend &) = delete;
            Backend &operator=(Backend &&) = delete;
            Backend &operator=(const Backend &) = delete;
            virtual ~Backend()                  = default;

            // initialise threads
            inline void init(){
                process_thread_ = std::make_unique<std::thread>(&Backend::run, this);
            }
            // initialise join
            inline void join(){
                if(process_thread_->joinable())
                    process_thread_->join();
            }
            // check if the given track id corresponding to a landmark is in the graph
            inline bool isLandmarkInGraph(TrackId track_id){
                return active_landmark_set_.count(track_id) >= 1;
            }
            // add the landmark track id to the landmark set
            inline void setLandmarkInGraph(TrackId track_id){
                active_landmark_set_.insert(track_id);
            }
            
            void run();
            // process the output from the input that is in a queue
            void processInput(const std::shared_ptr<BackendInput> &backend_input);
            
            void initialize(const std::shared_ptr<BackendInput> &backend_input);

            void update(const std::shared_ptr<BackendInput> &backend_input);

            void updateSmartFactorIndices(const gtsam::ISAM2Result &result);
            
            PointMap getPointMap(const gtsam::NonlinearFactorGraph &graph);

            Trajectory getTrajectory(void);

            
        
    };

}

#endif

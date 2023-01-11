#include "controller.h"
// #include <fmt/chrono.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <chrono>
#include <memory>
#include "../data/tum_dataset.h"
#include "utils.h"

namespace mono{
    Controller::Controller(std::string dataset_path){
        // Figure out a pointer for the dataset
        dataset_ = std::make_shared<TumDataset>(dataset_path);
        //Get detector params
        frontend_params_.detector_params = DetectorParams();
        frontend_params_.detector_params.params = FASTDetectorParams();
        //get tracker params
        auto &tracker_params = frontend_params_.tracker_params;

        {
            //initialise the tracker params
            tracker_params.rotation_tolerance                                           = 0.5 * 3.14 / 180.0;
            // need to figure out dataset
            std::tie(tracker_params.image_size.height, tracker_params.image_size.width) = dataset_->getImageSize();
            tracker_params.klt_window_size                                              = cv::Size(25, 25);
            tracker_params.klt_convergence_eps                                          = 0.001;
            tracker_params.klt_min_eig_thresh                                           = 1e-4;
            tracker_params.klt_max_iter                                                 = 10;
            tracker_params.klt_max_level                                                = 3;
            tracker_params.max_feature_age                                              = 5;

            tracker_params.outlier_ransac_prob      = 0.999;
            tracker_params.outlier_ransac_threshold = 1.0;
        }
        // Camera Calibration Matrix
        Eigen::Matrix3d K = dataset_->getCameraIntrinsics();
        // set camera calibration
        frontend_params_.K                       = K;
        frontend_params_.keyframe_match_quality  = 0.5;
        frontend_params_.keyframe_unique_quality = 0.8;
        // initialise the backend
        backend_params_.setDefaultISAM2Params();
        backend_params_.isam2_params.setEvaluateNonlinearError(true);
        // set initial noise
        backend_params_.init_rotation_sigma    = 1e-6;
        backend_params_.init_translation_sigma = 1e-6;
        // pass the camera calibration matrix to the backend
        backend_params_.K = gtsam::Cal3_S2(K(0, 0), K(1, 1), K(0, 1), K(0, 2), K(1, 2));
        backend_params_.smart_projection_sigma = 4;
        backend_params_.smart_projection_params.retriangulationThreshold = 1e-3;
        backend_params_.smart_projection_params.degeneracyMode = gtsam::DegeneracyMode::HANDLE_INFINITY;
        backend_params_.smart_projection_params.throwCheirality = true;
        backend_params_.smart_projection_params.verboseCheirality = true;
    }

    bool Controller::start(){
        if(setup()){
            run();
            return true;
        }
        return false;
    }

    bool Controller::setup(){
        //make_shared is a helper function to crearte a shared pointer for the class FrontEnd  with the frontend_params
        frontend_ = std::make_shared<Frontend>(frontend_params_);
        // same as above but for backend
        backend_ = std::make_shared<Backend>(backend_params_);
        // same as above but for the viewer
        pangolin_viewer_ = std::make_shared<PangolinViewer>("Monocular SLAM");

        // to create a shared pointer for the thread class we need a exit_signal_carrier variable of type future
        std::future<void> exit_signal_carrier = exit_signal_.get_future();
        dataloader_thread_ = std::make_unique<std::thread>(&Controller::processData, this, std::move(exit_signal_carrier));

        // set the size of the queues. Basically the max number of frames that'll be processed at any time    
        frontend_->input_queue_.set_capacity(10);
        frontend_->output_vis_queue_.set_capacity(10);
        backend_->input_queue_.set_capacity(10);
        backend_->output_queue_.set_capacity(10);

        // the backend queue from the front end serves as the input for the backend
        // Processed part will help in generating factors
        frontend_->backend_queue_ =&backend_->input_queue_;
        // frontend outputs a visulisation queue which the controller passes to pangolin
        pangolin_viewer_->frame_input_queue_ = &frontend_->output_vis_queue_;
        // backend outputs a queue which serves as the backend output for the pangolin_viewer
        pangolin_viewer_->backend_input_queue_ = &backend_->output_queue_;

        return true;
    }

    bool Controller::shutdown(){
        // shutdown the controller
        exit_signal_.set_value();
        // join all the threads
        dataloader_thread_->join();
        frontend_->join();
        backend_->join();
        // Shutdown Monocular SLAM
        return true;
    }

    bool Controller::run()
    {
        //! PangolinViewer is running in the main thread
        size_t num_frames = dataset_->length();
        frontend_->init();
        backend_->init();
        pangolin_viewer_->Output_Render(num_frames);
        return shutdown();
        
    }

    void Controller::processData(std::future<void> exit_signal_carrier){
        // total number of frames in the dataset
        size_t num_frames = dataset_->length();
        //printf("%ld", num_frames);
        std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> groundtruth_poses = dataset_->getGroundTruthPoses();

        // check Sampler
        gtsam::Sampler extrinsics_sampler(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(1e-1)));

        for(utils::Index i = 0; i < num_frames; i++){
            if(exit_signal_carrier.wait_for(std::chrono::milliseconds(0)) != std::future_status::timeout){
                break;
            }
            // // pangolin is paused
            // while(pangolin_viewer_->pause_playback){
            //     std::this_thread::sleep_for(std::chrono::milliseconds(30));
            // }
            //getting information for each frame
            DatasetItem item = dataset_->getItem(i);
            // initialise the current frame
            Frame curr_frame;
            curr_frame.index = i;
            curr_frame.image = item.color_image;
            curr_frame.timestamp = item.timestamp;
            gtsam::Pose3 groundtruth_pose = groundtruth_poses.at(i).second;

            //! Initially the estimate for all frames is identity
            if(i==0)
                curr_frame.frame_T_g = groundtruth_pose;
            else{
                curr_frame.frame_T_g = groundtruth_pose.retract(extrinsics_sampler.sample());
            }

            // populate the frontend input queue
            frontend_->input_queue_.push(std::make_shared<Frame>(curr_frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        // indicate the end of sequence
        frontend_->input_queue_.push(nullptr);
    }
} // namespace mono-slam

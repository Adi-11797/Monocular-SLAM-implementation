#include <bits/stdc++.h>

#include "../data/tum_dataset.h"
#include "../utils/utils.h"

#include "../controller/controller.h"

#include "../frontend/frontend.h"
#include "../frontend/tracker.h"
#include "../frontend/feature_detector.h"

#include "../backend/backend.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/Sampler.h>

// #include "../pangolin_viewer/pangolin_viewer.h"
#include <pangolin/display/display.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>

#include "../controller/controller.h"

using namespace std;

int main(){
    // cout<<"test"<<endl;
    ///// Testing timestamp
    string dataset_path = "/home/samiran/acad/2-2/24783/project_libraries/rgbd_dataset_freiburg1_xyz/";
    mono::TumDataset tum = mono::TumDataset(dataset_path);
    utils::Index ind = 10;
    mono::DatasetItem dt =tum.getItem(ind);
    cout<<"<<<<<<<<<<<<<<<<Test 1>>>>>>>>>>>>>>>>"<<endl;
    cout<<dt.timestamp<<"Test 1: timestamp"<<endl;

    ///// frame

    mono::FrontendParams frontend_params_;
    mono::BackendParams backend_params_;

    std::shared_ptr<mono::TumDataset> dataset_;
    std::shared_ptr<mono::PangolinViewer> pangolin_viewer_;
    std::shared_ptr<mono::Frontend> frontend_;
    std::shared_ptr<mono::Backend> backend_;

    //Thread for dataloader
    std::unique_ptr<std::thread> dataloader_thread_;
    // std::promise<void> exit_signal_;
    //////////////// Pointer to the dataset 
    dataset_ = std::make_shared<mono::TumDataset>(dataset_path);
    ///////////////Get detector params
    frontend_params_.detector_params = mono::DetectorParams();
    frontend_params_.detector_params.params = mono::FASTDetectorParams();
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
    cout<<"<<<<<<<<<<<<<<<<Test 2>>>>>>>>>>>>>>>>"<<endl;
    cout<<K(0)<<" Test 2: Camera Intrinsics"<<endl;

    ////////////////////////////////// Frontend
    //
    // mono::DatasetItem item2 = dataset_->getItem(1);
    vector<mono::DatasetItem> vec_item;
    // vec_item.push_back(item); vec_item.push_back(dataset_->getItem(1));
    std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> groundtruth_poses = dataset_->getGroundTruthPoses();
    gtsam::Sampler extrinsics_sampler(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(1e-1)));
    
    // initialise the current frame
    // for(int i = 0 ; i < 10; i++){
    //     mono::DatasetItem item = dataset_->getItem(i);
    //     mono::Frame curr_frame;
    //     curr_frame.index = i;
    //     curr_frame.image = item.color_image;
    //     curr_frame.timestamp = item.timestamp;
    //     gtsam::Pose3 groundtruth_pose = groundtruth_poses.at(i).second;
    //     if(i==0)
    //     curr_frame.frame_T_g = groundtruth_pose;
    //     else{
    //     curr_frame.frame_T_g = groundtruth_pose.retract(extrinsics_sampler.sample());
    //     }
    //     frontend_->input_queue_.push(std::make_shared<mono::Frame>(curr_frame));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     cout<<i<<endl;
    // }
    //! Initially the estimate for all frames is identity
    /////////////////////// tracking
    mono::DatasetItem item = dataset_->getItem(0);
    mono::Frame prev_frame;
    prev_frame.index = 1;
    prev_frame.image = item.color_image;
    prev_frame.timestamp = item.timestamp;

    mono::DatasetItem item2 = dataset_->getItem(10);
    mono::Frame curr_frame;
    curr_frame.index = 2;
    curr_frame.image = item2.color_image;
    curr_frame.timestamp = item2.timestamp;

    cout<<"<<<<<<<<<<<<<<<<Test 3>>>>>>>>>>>>>>>>"<<endl;
    cout<<"Image testing"<<endl;
    mono::FeatureTracker track = mono::FeatureTracker(K,tracker_params);
    cv::Mat img = track.visualizeTracks(prev_frame, curr_frame);
    cv::imshow("Track",img);
    cv::waitKey(0);
    ///////////////////// feature detector
    cout<<"<<<<<<<<<<<<<<<<Test 4>>>>>>>>>>>>>>>>"<<endl;
    std::unique_ptr<mono::FeatureDetector> feature_detector_ = std::make_unique<mono::FeatureDetector>(frontend_params_.detector_params);
    feature_detector_->detect(prev_frame, cv::Mat());
    feature_detector_->detect(curr_frame, cv::Mat());
    cout<<prev_frame.features[0].first<<"feature detection"<<endl;
    
    //////////////////// tracking
    std::unique_ptr<mono::FeatureTracker> feature_tracker_ = std::make_unique<mono::FeatureTracker>(K, frontend_params_.tracker_params);

    // frontend_ = std::make_shared<Frontend>(frontend_params_);
    // front_end_->process
    gtsam::Pose3& currframe_T_g = curr_frame.frame_T_g;

    // Pose estimate of the previous frame       
    gtsam::Pose3 currframe_T_prevframe_estimate;

    gtsam::Pose3 currframe_T_prevframe = prev_frame.frame_T_g.inverse() * currframe_T_g;
    currframe_T_prevframe_estimate = feature_tracker_->track(prev_frame, curr_frame, currframe_T_prevframe.rotation());
    //! Update pose estimate for current frame
    /* currframe_T_g = prev_frame->frame_T_g * currframe_T_prevframe_estimate; */

    cout<<"<<<<<<<<<<<<<<<<Test 5>>>>>>>>>>>>>>>>"<<endl;
    cout<<curr_frame.feature_ages[0]<<"Tracking works"<<endl;


    cout<<"<<<<<<<<<<<<<<<<Test 6>>>>>>>>>>>>>>>>"<<endl;
    cout<<currframe_T_prevframe_estimate<<" Pose b/w images: currframe_T_previous"<<endl;

    /// Controller
    cout<<"<<<<<<<<<<<<<<<<Test 7>>>>>>>>>>>>>>>>"<<endl;
    mono::Controller controller(dataset_path);
    cout<<controller.frontend_params_.K<<endl;
    cout<<controller.backend_params_.K<<endl;
    cout<<"backend initialisation intrinisics"<<endl;
    // Pangolin
    cout<<"<<<<<<<<<<<<<<<<Test 8>>>>>>>>>>>>>>>>"<<endl;
    cout<<"Pangolin"<<endl;
    auto& window = pangolin::CreateWindowAndBind("Pangolin_Test", 300, 400);
    pangolin::GlTexture rgb_texture_;
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.4, 0.5f, 0.75f, 0.0f);
    rgb_texture_.Reinitialise(300, 400, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);   // - Pangolin specific
    while (!pangolin::ShouldQuit())
    {
        pangolin::FinishFrame();
    }
    cout<<"<<<<<<<<<<<<<<<<Test 9>>>>>>>>>>>>>>>>"<<endl;
    cout<<dataset_->length()<<"dataset_length"<<endl;
    cout<<"<<<<<<<<<<<<<<<<Test 10>>>>>>>>>>>>>>>>"<<endl;
    // cout<<frontend_.feature_tracks_.track_id<<endl;
    ///////////////////////backend
    // auto visualize_image = feature_tracker_->visualizeTracks(prev_frame, curr_frame);
    // cv::imshow("Tracking",visualize_image);
    // cv::waitKey(0);
    // output_vis_queue_.push(std::make_shared<cv::Mat>(visualize_image));
    
    // populate the frontend input queue
    // frontend_->input_queue_.push(std::make_shared<Frame>(curr_frame));
    
    return 0;
}

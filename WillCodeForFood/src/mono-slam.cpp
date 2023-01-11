#include "opencv2/opencv.hpp"
#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>
///////
#include <cstdlib>


#include <utils.h>
#include <CLI/CLI.hpp>

#include <thread>

#include "controller/controller.h"
#include "data/tum_dataset.h"
#include "frontend/feature_detector.h"
#include "frontend/frame.h"
#include "frontend/tracker.h"
#include "viewer/pangolin_viewer.h"


int main(int argc, char* argv[]){
    // ///home/samiran/acad/2-2/24783/project_libraries/rgbd_dataset_freiburg1_xyz/rgb
    // cv::Mat image = cv::imread("/home/samiran/acad/2-2/24783/project_libraries/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png",cv::IMREAD_GRAYSCALE);
    // //Variables to store keypoints and descriptors
    // std::vector<cv::KeyPoint> keypoints1;
    // cv::Mat descriptors1;
    // int MAX_FEATURES = 100;
    // //Detect Orb features and compute descriptors
    // cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create();
    // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    // orb->detect(image,keypoints1);
    // orb->compute(image,keypoints1,descriptors1);

    // cv::Mat outimg1;
    // cv::drawKeypoints(image,keypoints1,outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    // cv::imshow("orb",outimg1);

    // cv::imshow("TUM",image);
    // cv::waitKey(0);    
    // return 0;

    CLI::App app{ "Monocular SLAM implementation" };

    std::string dataset_path;
    app.add_option("-f, --dataset-folder", dataset_path, "Data folder to process the dataset")->required();

    CLI11_PARSE(app, argc, argv);
    mono::Controller controller(dataset_path);

    if (controller.start())
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
    return 0;
}
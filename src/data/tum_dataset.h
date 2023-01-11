#ifndef MONO_SLAM_TUM_DATASET_H
#define MONO_SLAM_TUM_DATASET_H

#include <gtsam/geometry/Pose3.h>
#include <Eigen/Eigen>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <optional>
#include "utils.h"

// Do I need to use a namespace here?
// Yes, helps prevent collisions

namespace mono{
    struct DatasetItem{
        //timestamp
        utils::Timestamp timestamp;
        cv::Mat color_image;
        //index
        // utils::Index i;
    };
    namespace fs = std::filesystem;
    class TumDataset{
        public:
        //constructor
        // TumDataset(const std::string &root_dir) : root_dir_(root_dir){};
        TumDataset(const std::string &root_dir):root_dir_(root_dir){
            parseDataset(); 
        }
        TumDataset(TumDataset &&)      = default; // move constructor: we move the data directly
        TumDataset(const TumDataset &) = default; // copy constructor: copies the data on the heap
        TumDataset &operator=(TumDataset &&) = default; // move assignment operator
        TumDataset &operator=(const TumDataset &) = default; // copy assignment operator
        virtual ~TumDataset()                  = default; // virtual destructor
        //Total number of files in the data
        inline size_t length(){
            return TumDataset::color_filenames_.size();

        }
        //access individual files
        // DatasetItem getItem(utils::Index ind);
        // std::pair<size_t, size_t> getImageSize();
        // std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> getGroundTruthPoses();
        // Eigen::Matrix3d getCameraIntrinsics();
        DatasetItem getItem(utils::Index index);

        std::tuple<size_t, size_t> getImageSize(){ return std::tuple<size_t, size_t>(height_, width_); }
        std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> getGroundTruthPoses() { return gt_stamped_poses_; }
        Eigen::Matrix3d getCameraIntrinsics() { return camera_intrinsics_; }
        // General function
        bool parseDataset();
        std::vector<std::string> readFile(const fs::path &txt_file);
        void readIntrinsics(const fs::path &txt_file);
        std::pair<std::vector<fs::path>,std::vector<utils::Timestamp>> readImageFile(const fs::path &txt_file);
        std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> readGroundTruthFile(const fs::path &txt_file);

        fs::path root_dir_;
        //path to the camera matrix file
        fs::path intrinsics_file_;
        //image width and height
        unsigned int width_;
        unsigned int height_;
        //store the camera intrinsics
        Eigen::Matrix3d camera_intrinsics_;
        //Do I need a distortion matrix, yes I do
        Eigen::Matrix<double, 5, 1> distortion_;


        //store the input data information
        std::vector<fs::path> color_filenames_;
        std::vector<utils::Timestamp> color_timestamps_;

        //store the groundtruth poses
        std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> gt_stamped_poses_;
        
    };
}

#endif
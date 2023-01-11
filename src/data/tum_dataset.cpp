#include "tum_dataset.h"
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include "utils.h"



namespace mono{
    // Constructor, initialise the root directory
    // TumDataset::TumDataset(const std::string &root_dir):root_dir_(root_dir){
    //     TumDataset::parseDataset(); 
    // }

    // inline size_t TumDataset::length(){
    //     return TumDataset::color_filenames_.size();
    // }

    // Figure this out
    DatasetItem TumDataset::getItem(utils::Index index){
        // DatasetItem di;
        // di.color_image = cv::imread(color_filenames.at(index).string(), cv::IMREAD_COLOR);
        // di.timestamp = color_timestamps_.at(index);
        // return di;
        // di.color_image = color_filenames
        DatasetItem item;
        item.color_image = cv::imread(color_filenames_.at(index).string(), cv::IMREAD_COLOR);
        item.timestamp   = color_timestamps_.at(index);

        return item;
    }

    bool TumDataset::parseDataset()
    {   // intrinsics_file is of datatype filesystem
        TumDataset::intrinsics_file_ = root_dir_ / "calibration.txt";
        fs::path color_file    = root_dir_ / "rgb.txt";


        readIntrinsics(intrinsics_file_);
        std::tie(color_filenames_, color_timestamps_) = readImageFile(color_file);
        // Might have to use tie for storage reasons
        // std::pair<std::vector<fs::path>,std::vector<utils::Timestamp>> image_output = readImageFile(color_file);
        // color_filenames_ = image_output.first;
        // color_timestamps_ = image_output.second;

        fs::path ground_truth_file = root_dir_ / "groundtruth.txt";

        TumDataset::gt_stamped_poses_ = readGroundTruthFile(ground_truth_file);
        return true;
    }
    // read and return a vector containing the filenames.
    std::vector<std::string> TumDataset::readFile(const fs::path& txt_file)
    {
        std::ifstream file_stream(txt_file.string());
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(file_stream, line))
        {
            if (line[0] == '#')
                continue;
            lines.push_back(line);
        }
        return lines;
    }
    // intrinsics
    void TumDataset::readIntrinsics(const fs::path &txt_file){
        //read files returns an vector of strings
        auto lines = readFile(txt_file);
        //get the number of files to loop over
        std::size_t n = lines.size();

        for(size_t count; count<n; count++){
            auto line = lines[count];
            auto delimiter_pos = line.find('=');
            auto name = line.substr(0,delimiter_pos);
            auto value = line.substr(delimiter_pos + 1);

            std::stringstream value_stream;
            value_stream << value;
            // std::string message;
            // update the image width
            if(name.find("width")!=std::string::npos){
                value_stream >> width_;
                //fmt open source library
                // format is like printf but safer
                // message = fmt::format(" Value : {}", width);
            }
            else if(name.find("height") != std::string::npos){
                value_stream >> height_;
                // message = fmt::format(" Value : {}", height);
            }
            else if(name.find("camera matrix") != std::string::npos){
                value_stream >> camera_intrinsics_(0) >> camera_intrinsics_(1)
                >>camera_intrinsics_(2) >>camera_intrinsics_(3) >> camera_intrinsics_(4) 
                >> camera_intrinsics_(5) >> camera_intrinsics_(6) >>camera_intrinsics_(7) 
                >> camera_intrinsics_(8);
                //Eigen  keeps all matrices and in colum-major
                camera_intrinsics_.transposeInPlace();
                // message = fmt::format(" Value:\n{}", camera_intrinsics);
            }
            else if(name.find("distortion") != std::string::npos)
            {
                value_stream >> distortion_(0) >> distortion_(1) >> distortion_(2) >> distortion_(3) >> distortion_(4);
                // message = fmt::format(" Value:\n{}", distortion);
            }
            else
            {
                // message = fmt::format("Value: {}", value);
            }
 
        }
        
    }

    std::pair<std::vector<fs::path>,std::vector<utils::Timestamp>> TumDataset::readImageFile(const fs::path &txt_file){
        auto lines             = readFile(txt_file);
        std::size_t line_count = lines.size();

        std::vector<utils::Timestamp> timestamps;
        std::vector<fs::path> image_filenames;
        timestamps.reserve(line_count);
        image_filenames.reserve(line_count);

        for(size_t count = 0; count < line_count; count++){
            auto line = lines[count];
            auto delimiter_pos = line.find(' ');
            auto timestamp =  std::stod(line.substr(0,delimiter_pos));
            auto filename = line.substr(delimiter_pos+1);
            
            //convert from seconds to microseconds
            timestamps.emplace_back(1e6 * timestamp);
            image_filenames.emplace_back(root_dir_ / filename);
        }
        return make_pair(image_filenames, timestamps);
    }

    
    std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> TumDataset::readGroundTruthFile(const fs::path &txt_file){
        auto lines = readFile(txt_file);
        std::size_t line_count = lines.size();

        std::vector<std::pair<utils::Timestamp, gtsam::Pose3>> stamped_poses;
        for(size_t count = 0; count < line_count; count++){
            auto line          = lines[count];
            auto delimiter_pos = line.find(' ');
            auto timestamp = std::stod(line.substr(0, delimiter_pos));
            auto value     = line.substr(delimiter_pos + 1);

            std::stringstream pose_stream;
            std::array<double, 7> posedata;
            pose_stream << value;
            pose_stream >> posedata[0] >> posedata[1] >> posedata[2] >> posedata[3] >>
            posedata[4] >> posedata[5] >> posedata[6];
            gtsam::Rot3 rotation(gtsam::Quaternion(posedata[6], posedata[3], posedata[4], posedata[5]));
            Eigen::Vector3d translation(posedata[0],posedata[1],posedata[2]);
            stamped_poses.emplace_back(utils::Timestamp(1e6 * timestamp), gtsam::Pose3(rotation, translation));
        }
        return stamped_poses;
    }



}
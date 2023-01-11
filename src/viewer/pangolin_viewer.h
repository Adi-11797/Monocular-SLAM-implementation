#ifndef MONO_SLAM_PANGOLIN_VIEWER_H
#define MONO_SLAM_PANGOLIN_VIEWER_H

#include <pangolin/display/view.h>
#include <pangolin/pangolin.h>
#include <tbb/tbb.h>
#include <utils.h>
#include "../backend/backend.h"
#include "../frontend/frame.h"

namespace mono
{
    class PangolinViewer
    {
        public:

        PangolinViewer(const std::string &window_name);
        PangolinViewer(PangolinViewer &&)      = delete;                    
        PangolinViewer(const PangolinViewer &) = delete;                    
        PangolinViewer &operator=(PangolinViewer &&) = delete;           
        PangolinViewer &operator=(const PangolinViewer &) = delete;       
        virtual ~PangolinViewer()                         = default;   // virtual destructor! (can't have virtual constructors) - Deleting a derived class object using a pointer of base class type that has a non-virtual destructor results in undefined behavior. To correct this situation, the base class should be defined with a virtual destructor

        void Output_Render(size_t num_frames);

        tbb::concurrent_bounded_queue<std::shared_ptr<cv::Mat>> *frame_input_queue_           = nullptr;
        tbb::concurrent_bounded_queue<std::shared_ptr<BackendOutput>> *backend_input_queue_   = nullptr;

        // pangolin::Var<bool> show_image_features;        // toggle between image features being shown/hidden - 
        // pangolin::Var<bool> pause_playback;             // toggle playback - 


        private:
        //! Window size
        static constexpr float WINDOW_WIDTH  = 1920.f;
        static constexpr float WINDOW_HEIGHT = 1080.f;

        //! UI window size
        static constexpr int UI_WINDOW_WIDTH = 400;
        //! Rendering virtual OpenGL camera properties
        static constexpr int CAMERA_WIDTH           = 640;
        static constexpr int CAMERA_HEIGHT          = 480;
        static constexpr double CAMERA_FOCAL_LENGTH = 100.0;
        static constexpr double CAMERA_NEAR_CLIP    = 0.01;
        static constexpr double CAMERA_FAR_CLIP     = 100.0;


        //! Rendering virtual OpenGL camera is at 0, 0, -1 and looking at the origin along z axis
        //! TODO: How to make constexpr EigenVectors?
        const Eigen::Vector3d CAMERA_CENTER = Eigen::Vector3d(0.0, 0.0, 1.0);
        const Eigen::Vector3d CAMERA_LOOKAT = Eigen::Vector3d(0.0, 0.0, -1.0);

        std::string window_name_;              


        pangolin::OpenGlMatrix projection_matrix_;
        pangolin::OpenGlRenderState render_transforms_;

        // pangolin::View ui_view_;
        pangolin::View main_view_;
        pangolin::View image_view_;
        pangolin::View map_view_;

        pangolin::GlTexture rgb_texture_;

        cv::Mat prev_vis_image_;
        gtsam::Pose3 prev_camera_;

        void handleImageView();
        void handleMapView();

        void drawCamera(const gtsam::Pose3 &camera_T_world,
                        double line_width,
                        std::array<uint8_t, 3> color,
                        double size_factor);

        void drawTrajectory(const std::vector<gtsam::Pose3> &cameras,
                            double trajectory_line_width,
                            std::array<uint8_t, 3> trajectory_color,
                            double camera_line_width,
                            std::array<uint8_t, 3> camera_color,
                            double camera_size_factor,
                            gtsam::Pose3 &last_Camera);
    };
}
#endif

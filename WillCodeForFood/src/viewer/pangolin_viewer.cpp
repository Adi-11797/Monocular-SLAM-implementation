#include "pangolin_viewer.h"
#include <pangolin/display/display.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace mono
{
    PangolinViewer::PangolinViewer(const std::string& window_name) : window_name_(window_name)
    {
        auto& window = pangolin::CreateWindowAndBind(window_name_, static_cast<int>(WINDOW_WIDTH), static_cast<int>(WINDOW_HEIGHT));

        // Paint pixels only if it is in-front of something rendered.
        glEnable(GL_DEPTH_TEST);
        // Set the background color set during the glClear operation below.
        glClearColor(0.75f, 0.75f, 0.75f, 0.0f);

        rgb_texture_.Reinitialise(640, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);   // - Pangolin specific

        // Camera projection matrix
        projection_matrix_   = pangolin::ProjectionMatrix(CAMERA_WIDTH,
                                                        CAMERA_HEIGHT,
                                                        CAMERA_FOCAL_LENGTH,
                                                        CAMERA_FOCAL_LENGTH,
                                                        CAMERA_WIDTH / 2.0,
                                                        CAMERA_HEIGHT / 2.0,
                                                        CAMERA_NEAR_CLIP,   //   - needed to identify minimum image rendering distance
                                                        CAMERA_FAR_CLIP);   //   - needed to identify maximum image rendering distance

        // Camera model-view matrix
        auto modelview_matrix = pangolin::ModelViewLookAt(CAMERA_CENTER(0),
                                                         CAMERA_CENTER(1),
                                                         CAMERA_CENTER(2),
                                                         CAMERA_LOOKAT(0),
                                                         CAMERA_LOOKAT(1),
                                                         CAMERA_LOOKAT(2),
                                                         pangolin::AxisNegY);

        render_transforms_   = pangolin::OpenGlRenderState(projection_matrix_, modelview_matrix);

        // User-Interface window dimensioning
        //ui_view_ = pangolin::CreatePanel("ui").SetBounds(0.0, 0.0, 0.0, 0.0);

        // Image viewer dimensioning            
        image_view_ = pangolin::CreateDisplay().SetBounds((1.0F - float(CAMERA_HEIGHT) / float(WINDOW_HEIGHT)), 1.0, 0.0, (CAMERA_WIDTH) / WINDOW_WIDTH, -640.0 / 480.0);

        // Map-view window dimensioning
        map_view_ = 
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.5, 1.0, -static_cast<double>(CAMERA_WIDTH) / static_cast<double>(CAMERA_HEIGHT))
                .SetHandler(new pangolin::Handler3D(render_transforms_))
                .SetLayout(pangolin::LayoutEqual);
        window.Resize(1920 + 16, 1080 + 9);
    }

    void PangolinViewer::Output_Render(size_t num_frames)
    {
        cv::Mat prev_visualize_image;
        std::unique_ptr<gtsam::Pose3> prev_camera;
        size_t count = 0;
        num_frames = 99;

        while (!pangolin::ShouldQuit() && count<num_frames)
        {   
            count++;
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (image_view_.IsShown())
            {
                handleImageView();
            }
            if (map_view_.IsShown())
            {
                handleMapView();
            }
            pangolin::FinishFrame();
        }
        // INFO("exited pangolin loop");
        //! call or notify mainthread to shutdown
        std::cout<<"#####   SHUTDOWN PANGOLIN VIEWER   #####"<<std::endl;
    }

    void PangolinViewer::handleImageView()
    {
        image_view_.Activate();
        std::shared_ptr<cv::Mat> vis_image;
        bool image_success = false;
        if (prev_vis_image_.empty())
        {
            frame_input_queue_->pop(vis_image);
            if (!vis_image)
            {
                return;
            }
            image_success = true;
        }
        else
        {
            image_success = frame_input_queue_->try_pop(vis_image);
        }

        if (image_success)
        {
            if (vis_image)
            {
                rgb_texture_.Upload(vis_image->data, GL_BGR, GL_UNSIGNED_BYTE);
                glColor3f(1.0f, 1.0f, 1.0f);
                rgb_texture_.RenderToViewportFlipY();
                std::swap(prev_vis_image_, *vis_image);
            }
        }
        else
        {
            rgb_texture_.Upload(prev_vis_image_.data, GL_BGR, GL_UNSIGNED_BYTE);
            glColor3f(1.0f, 1.0f, 1.0f);
            rgb_texture_.RenderToViewportFlipY();
        }
    }

    void PangolinViewer::handleMapView()
    {
        map_view_.Activate(render_transforms_);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        std::shared_ptr<BackendOutput> backend_output;
        backend_input_queue_->pop(backend_output);
        // INFO("Popped from Backend output queue");

        if (backend_output)
        {
            const PointMap& point_map    = backend_output->point_map;
            const Trajectory& trajectory = backend_output->trajectory;
            std::vector<gtsam::Vector3> point_cloud;
            std::vector<gtsam::Pose3> cameras;
            if (trajectory.size() == 0)
                return;
            for (const auto& it : trajectory)
            {
                cameras.push_back(it.second);
            }
            auto last_camera                             = cameras.back();
            Eigen::Vector3d last_camera_look_at          = last_camera.translation();
            Eigen::Vector3d last_camera_eye              = last_camera_look_at - Eigen::Vector3d(0, 0, 5);
            auto view_look_at                            = pangolin::ModelViewLookAt(last_camera_eye(0),
                                                          last_camera_eye(1),
                                                          last_camera_eye(2),
                                                          last_camera_look_at(0),
                                                          last_camera_look_at(1),
                                                          last_camera_look_at(2),
                                                          pangolin::AxisNegY);
            pangolin::OpenGlRenderState render_transform = pangolin::OpenGlRenderState(projection_matrix_, view_look_at);
            render_transform.Apply();

            //! TODO: Update points based on their keys
            for (const auto& it : point_map)
            {
                point_cloud.push_back(it.second);
            }
            pangolin::glDrawPoints(point_cloud);
            //! TODO: Update cameras based on their keys

            drawTrajectory(cameras, 1.0, { 0, 0, 0 }, 2.0, { 200, 0, 0 }, 0.2, last_camera);
        }
        pangolin::glDrawAxis(gtsam::Pose3::identity().matrix(), 1.0);
    }

    void PangolinViewer::drawCamera(const gtsam::Pose3& camera_T_world,
                                    double line_width,
                                    std::array<uint8_t, 3> color,
                                    double size_factor)
    {
        //! TODO: Aspect ratio

        //! Five end points of the camera
        std::vector<Eigen::Vector3d> points = { { 0, 0, 0 }, { 1, 1, 1 }, { 1, -1, 1 }, { -1, -1, 1 }, { -1, 1, 1 } };
        for (auto& point : points)
        {
            point(0) = point(0) * size_factor;
            point(1) = point(1) * size_factor;
            point(2) = point(2) * size_factor;
        }
        // clang-format off
        //! Conect the points to create frustum lines
        const std::vector<Eigen::Vector3d> lines = { points[0], points[1],
                                                     points[0], points[2],
                                                     points[0], points[3],
                                                     points[0], points[4],
                                                     points[1], points[2],
                                                     points[2], points[3],
                                                     points[3], points[4],
                                                     points[4], points[1] };
        // clang-format on

        glPushMatrix();
        glMultMatrixd(camera_T_world.matrix().data());
        glColor3ubv(color.data());
        glLineWidth(static_cast<float>(line_width));
        pangolin::glDrawLines(lines);
        glPopMatrix();
    }

    void PangolinViewer::drawTrajectory(const std::vector<gtsam::Pose3>& cameras,
                                        double trajectory_line_width,
                                        std::array<uint8_t, 3> trajectory_color,
                                        double camera_line_width,
                                        std::array<uint8_t, 3> camera_color,
                                        double camera_size_factor,
                                        gtsam::Pose3 &last_camera)
    {
        std::vector<Eigen::Vector3d> trajectory_lines;
        for (size_t i = 0; i < cameras.size(); ++i)
        {
            // DEBUG("Drawing camera lines");
            drawCamera(last_camera, camera_line_width, {0,200,0}, camera_size_factor);
            drawCamera(cameras[i], camera_line_width, camera_color, camera_size_factor);
            if (i != 0)
            {
                trajectory_lines.push_back(cameras[i - 1].translation());
                trajectory_lines.push_back(cameras[i].translation());
            }
        }
        glColor3ubv(trajectory_color.data());
        glLineWidth(static_cast<float>(trajectory_line_width));
        // DEBUG("Drawing trajectory lines");
        pangolin::glDrawLines(trajectory_lines);
    }

}

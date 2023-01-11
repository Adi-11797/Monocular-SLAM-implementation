#ifndef MONO_SLAM_CONTROLLER_H
#define MONO_SLAM_CONTROLLER_H

#include <utils.h>
#include <future>
#include "../backend/backend.h"
#include "../data/tum_dataset.h"
#include "../frontend/frontend.h"
#include "../viewer/pangolin_viewer.h"


namespace mono{
    
    class Controller{
        public:
            Controller(std::string dataset_path);
            ~Controller() = default;
            

            bool start();
            bool setup();
            bool run();
            bool shutdown();


            FrontendParams frontend_params_;
            BackendParams backend_params_;

            std::shared_ptr<TumDataset> dataset_;
            std::shared_ptr<PangolinViewer> pangolin_viewer_;
            std::shared_ptr<Frontend> frontend_;
            std::shared_ptr<Backend> backend_;

            //Thread for dataloader
            std::unique_ptr<std::thread> dataloader_thread_;
            std::promise<void> exit_signal_;
            void processData(std::future<void> exit_signal_carrier);
    };
}

#endif
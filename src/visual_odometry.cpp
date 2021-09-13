#include "visual_odometry.h"
#include "config.h"
#include "viewer.h"

namespace myslam
{
    VisualOdometry::VisualOdometry(std::string &config_path)
                    :config_file_path_(config_path){}

    bool VisualOdometry::Init()
    {
        if(Config::SetParameterFile(config_file_path_) == false)
        {
            std::cout << "配置文件未读入。" << std::endl;
            return false;
        }

        dataset_ =
                Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        CHECK_EQ(dataset_->Init(), true);


        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Back_end::Ptr(new Back_end);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCamera(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VisualOdometry::Run()
    {
        while (1)
        {
            std::cout << "VO is running" << std::endl;
            if (Step() == false)
            {
                break;
            }
        }

        backend_->Stop();
        viewer_->Close();

        std::cout << "VO exit";
    }

    bool VisualOdometry::Step() {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if (new_frame == nullptr) return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }
}
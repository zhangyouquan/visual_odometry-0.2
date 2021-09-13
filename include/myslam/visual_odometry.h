
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "common.h"
#include "frontend.h"
#include "viewer.h"
#include "dataset.h"
#include "backend.h"

namespace myslam
{
    class VisualOdometry
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        // 构造视觉里程计
        VisualOdometry(std::string &config_path);
        // 初始化操作
        bool Init();
        //初始化完成后，进行跟踪任务
        void Run();
        // 下一帧
        bool Step();

        //获取前端状态。
        FrontendStatus GetFrontendStatus() const
        {
            return frontend_->GetStatus();
        }

    private:
        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Back_end::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        // dataset
        Dataset::Ptr dataset_ = nullptr;
    };
}
#endif
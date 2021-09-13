/**
 *
 *     用opengl写可视化界面
 *
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "common.h"
#include "frame.h"
#include "map.h"

namespace myslam {

/**
 * 可视化
 */
    class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) { map_ = map; }

        void Close();

        void AddCurrentFrame(Frame::Ptr last_frame,Frame::Ptr current_frame);

        // 更新地图
        void UpdataMap();

    private:
        void ThreadLoop();

        void DrawFrame(Frame::Ptr frame, const float* color);

        void DrawMapPoints();

        void DrawTrajectory(Frame::Ptr frame1, Frame::Ptr frame2);

        void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

        /// plot the features in current frame into an image
        cv::Mat PlotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;
    };
}

#endif
/**
 *      视觉前端
 *      通过两两帧图像之间的特征匹配关系估计相机运动。
 *      这里用的是双目相机，每帧图像可以计算特征点的深度（初始化）
 *      然后对相邻图像进行特征匹配，根据PNP算法估计相机运动
 *      估计成功返回1，失败标记为丢失。里程计失效。
 *      满足关键帧条件就，插入关键帧
 */

#ifndef FRONTEND_H
#define FRONTEND_H

#include <opencv2/features2d.hpp>
#include "common.h"
#include "frame.h"
#include "feature.h"
#include "map.h"
#include "backend.h"

namespace myslam
{
    class Back_end;
    class Viewer;

    //枚举里程计状态
    enum class FrontendStatus{ INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        // 接收新来图像帧，并计算其定位结果
        bool AddFrame(Frame::Ptr frame);

        // Set函数
        void SetMap(Map::Ptr map) { map_ = map; }

        void SetBackend(std::shared_ptr<Back_end> backend) { backend_ = backend; }

        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

        FrontendStatus GetStatus() const
        {
            return status_;
        }
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }
        static Vec2 toVec2(const cv::Point2f p);

    private:

        //前端功能
        bool Track();
        bool Reset();
        int TrackLastFrame();//返回跟踪上的特征点数
        int EstimateCurrentPose();//返回内点个数
        bool InsertKeyframe();
        bool StereInit();
        int DeleteFeatures();
        int FindFeaturesInRight();
        bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> &points, Vec3 &pworld);
        bool BuildInitMap();
        int TriangulateNewPoints();
        void SetObservationsForKeyFrame();


        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Back_end> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;


        SE3 relative_motion_;
        int tracking_liliers_ = 0;

        // params
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;

        // utilities
        cv::Ptr<cv::GFTTDetector> gftt_;  // 特征检测器gftt特征点
    };
}
#endif


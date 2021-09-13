/**
 * Map管理所有的关键帧和地图点，完成地图中地图点状态的管理。
 *
 * */
#ifndef MAP_H
#define MAP_H

#include "common.h"
#include "MapPoint.h"
#include "frame.h"

//using namespace std;
namespace myslam
{
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        //既需要随机访问,又需要随时插入和删除,使用哈希表存储,无序
        typedef std::unordered_map<unsigned long,MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long,Frame::Ptr> KeyframesType;

        Map(){}

        //添加一个关键帧
        void InsertKeyFrame(Frame::Ptr frame);
        //增加一个地图顶点
        void InsertMapPoint(MapPoint::Ptr map_point);
        //获取所有地图点
        LandmarksType GetAllMapPoints()
        {
            std::lock_guard<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        //获取所有关键帧
        KeyframesType GetKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        //获取激活地图点
        LandmarksType GetActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        // 获取激活关键帧
        KeyframesType GetActiveKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);//锁住地图线程
            return active_keyframes_;
        }
        // 清理map中观测数量为零的点
        void CleanMap();

    private:
        // 将旧的关键帧置为不活跃状态
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;         // all landmarks
        LandmarksType active_landmarks_;  // active landmarks
        KeyframesType keyframes_;         // all key-frames
        KeyframesType active_keyframes_;  // all key-frames

        Frame::Ptr current_frame_ = nullptr;

        int num_active_keyframes_ = 7;  // 最多激活的关键帧数量

    };
};
#endif
#ifndef BACK_END_H
#define BACK_END_H

#include "common.h"
#include "frame.h"
#include "map.h"

namespace myslam
{
    class Map;

    /**
     * 后端
     * 有单独优化线程，在Map更新时启动优化
     * Map更新由前端触发
     */
    class Back_end
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Back_end> Ptr;

        Back_end();

        void SetCamera(Camera::Ptr left, Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

        // 设置地图
        void SetMap(std::shared_ptr<Map> map)
        {
            map_ = map;
        }

        // 触发地图更新，启动优化
        void UpdataMap();

        // 关闭后端优化线程
        void Stop();

    private:
        void BackendLoop();

        //对给定关键帧和路标点进行优化
        void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        //后端线程
        std::thread backend_thread_;
        std::mutex data_mutex;//后端的全局互斥锁

        std::condition_variable map_updata_;//类模板，当类对象调用wait函数，相关线程就会被阻塞并释放锁，
                        // 直到另一个线程中的相同类对象使用notify函数唤醒，相关线程才能重新拿锁继续执行
        std::atomic<bool> backend_running_;//原子操作,在多线程并发的条件下，所有不是原子性的操作，需要保证原子性时，都需要进行原子操作处理。
        // 是这样一种感觉，多个线程共享backend_running_的值时，必须等这个值真正写入内存之后其他线程才能继续操作，类似于锁，但是比锁效率高

        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;
    };


}

#endif
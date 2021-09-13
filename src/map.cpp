
#include "map.h"
#include "feature.h"

namespace myslam
{
    void Map::InsertKeyFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;
        // 判断此关键帧是否被插入了
        if(keyframes_.find(frame->keyframe_id_)==keyframes_.end())
        {
            keyframes_.insert(make_pair(frame->keyframe_id_,frame));
            active_keyframes_.insert(make_pair(frame->keyframe_id_,frame));
        } else//地图中已经有这个id对应的关键帧了，那就用新的替换.
        {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        if(active_keyframes_.size() > num_active_keyframes_)
        {
            //将旧关键帧设为不再活跃
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point)
    {
        //同样判断是否已经插入过，如果又就用新的更新
        if(landmarks_.find(map_point->id_) == landmarks_.end())
        {
            landmarks_.insert(make_pair(map_point->id_,map_point));
            active_landmarks_.insert(make_pair(map_point->id_,map_point));
        } else
        {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }
    // 清理map中观测数量为零的点
    void Map::CleanMap()
    {
        for(LandmarksType::iterator it = active_landmarks_.begin(); it != active_landmarks_.end(); )
        {
            if(it->second->observed_times_ == 0)
            {
                it = active_landmarks_.erase(it);//抹去数据迭代器不变
            } else
            {
                it++;
            }
        }
    }

    // 将旧的关键帧置为不活跃状态
    void Map::RemoveOldKeyframe()
    {
        if(current_frame_ == nullptr)
        {
            return;
        }
        // 寻找与当前帧最近和最远的两个关键帧；距离是用两帧之间的位姿变换表示
        double max_dis = 0,min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse();
        for(KeyframesType::iterator kf = active_keyframes_.begin(); kf != active_keyframes_.end(); kf++)
        {
            if(kf->second == current_frame_)continue;
            auto dis = (kf->second->Pose() * Twc).log().norm();
            //得到最大距离及对应id
            if(dis > max_dis)
            {
                max_dis = dis;
                max_kf_id = kf->first;
            }
            //得到最小距离
            if(dis < min_dis)
            {
                min_dis = dis;
                min_kf_id  = kf->first;
            }
        }

        const double min_dis_th = 0.2;
        Frame::Ptr frame_to_remove = nullptr;
        if(min_dis < min_dis_th)
        {
            frame_to_remove = keyframes_.at(min_kf_id);
        } else
        {
            frame_to_remove = keyframes_.at(max_kf_id);
        }

        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        //观测特征也要删掉
        for(auto feat : frame_to_remove->features_left_)
        {
            //删地图点要拿锁
            auto mp = feat->map_point_.lock();//
            if (mp)
            {
                mp->RemoveObservation(feat);
            }
        }
        for (auto feat : frame_to_remove->features_right_) {
            if (feat == nullptr) continue;
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveObservation(feat);
            }
        }

        CleanMap();


    }
}

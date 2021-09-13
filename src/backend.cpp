#include "backend.h"
#include "feature.h"
#include "g2o_types.h"
#include "map.h"
#include "MapPoint.h"
#include "frontend.h"

namespace myslam
{
    Back_end::Back_end()
    {
        backend_running_.store(true);//就是将类对象保存的值赋值为true，内部替换的时候是加了线程同步的，不用担心多线程访问的问题
        backend_thread_ = std::thread(std::bind(&Back_end::BackendLoop,this));
        //std::bind类似延迟调用，调用时自动构造一个与Back_end::BackendLoop功能一样的函数
    }

    void Back_end::UpdataMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex);
        map_updata_.notify_one();
    }

    void Back_end::Stop()
    {
        backend_running_.store(false);
        map_updata_.notify_one();
        backend_thread_.join();
    }

    void Back_end::BackendLoop()
    {
        while(backend_running_.load())//加载类对象中存储的值
        {
            std::unique_lock<std::mutex> lock(data_mutex);
            map_updata_.wait(lock);

            ///后端优化激活的关键帧和路标点。
            auto active_kfs = map_->GetActiveKeyFrames();
            auto active_pts = map_->GetActiveMapPoints();
            Optimize(active_kfs,active_pts);
        }
    }

    void Back_end::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
    {
        //
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        //pose顶点
        std::map<unsigned long,VertexPose*> vertices;
        unsigned long max_kf_id = 0;
        for(auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);

            //记录一下最大顶点id
            if(kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert(std::make_pair(kf->keyframe_id_,vertex_pose));
        }

        //路标点顶点
        std::map<unsigned long,VertexXYZ*> vertices_landmarks;

        Mat33 K = camera_left_->K();
        SE3 left_ext = camera_left_->pose();
        SE3 right_ext = camera_right_->pose();

        //边
        int index = 1;
        double chi2_th = 5.991;//2自由度（重投影误差）卡方检验,这里没有考虑视差
        std::map<EdgeProjection*,Feature::Ptr> edges_and_features;

        for(auto &landmark : landmarks)
        {
            //如果地图点是外点，则跳过。
            if(landmark.second->is_outlier_)continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();//观测是一些普通帧和二维特征点。
            for(auto &obs : observations)
            {
                if(obs.lock()== nullptr)continue;
                auto feat = obs.lock();//拿到帧和特征点信息
                if (feat->is_outliner_ || feat->frame_.lock() == nullptr) continue;

                auto frame = feat->frame_.lock();
                EdgeProjection* edge = nullptr;

                if(feat->is_on_left_image_)
                {
                    edge = new EdgeProjection(K,left_ext);
                } else
                {
                    edge = new EdgeProjection(K,right_ext);
                }

                //如果这个地图点还没有被加入优化，则加入到顶点里面
                if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
                {
                    VertexXYZ* v = new VertexXYZ();
                    v->setEstimate(landmark.second->GetPos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert(std::make_pair(landmark_id,v));
                    optimizer.addVertex(v);
                }

                edge->setId(index);
                edge->setVertex(0,vertices.at(frame->keyframe_id_));
                edge->setVertex(1,vertices_landmarks.at(landmark_id));
                edge->setMeasurement(Frontend::toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert(std::make_pair(edge,feat));
                optimizer.addEdge(edge);

                index++;
            }
        }
        //要开始优化了，并去除外点。
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int num_outliner = 0,num_inliner = 0;
        int iteration = 0;
        while(iteration < 10)
        {
            num_outliner = 0;
            num_inliner = 0;
            for(auto &ef: edges_and_features)
            {
                if(ef.first->chi2() > chi2_th)
                {
                    num_outliner++;
                }else
                {
                    num_inliner++;
                }
            }
            double inliner_ratio = num_inliner / double(num_inliner+num_outliner);

            if(inliner_ratio > 0.5)
            {
                break;//跳出循环,不再进来。
            } else
            {
                chi2_th *= 2;
                iteration++;
            }
        }

        for(auto &ef : edges_and_features)
        {
            if(ef.first->chi2() > chi2_th)
            {
                ef.second->is_outliner_ = true;
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            }else
            {
                ef.second->is_outliner_ = false;
            }
        }

        //设置位姿和地图点位置
        for(auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }
}
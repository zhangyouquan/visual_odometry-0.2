#include <opencv2/opencv.hpp>
#include "frontend.h"
#include "config.h"
#include "feature.h"
#include "map.h"
#include "backend.h"
#include "g2o_types.h"
#include "viewer.h"
#include <vector>

namespace myslam
{
    Frontend::Frontend()
    {
        gftt_ =
                cv::GFTTDetector::create(Config::Get<int>("number_features"),0.01,20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }
    //接收帧开始估计位姿
    bool Frontend::AddFrame(myslam::Frame::Ptr frame)
    {
        current_frame_ = frame;

        switch (status_)
        {
            case FrontendStatus::INITING:
                StereInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD: //这两种情况都继续跟踪.
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::StereInit()
    {
        int num_features_left = DeleteFeatures();
        int num_coor_features = FindFeaturesInRight();//这一步完成特征匹配,其实是用的光流法
        if(num_coor_features < num_features_init_)//匹配数不够初始化
        {
            std::cout << "提取特征数不够。退出" << std::endl;
            return false;
        }
        bool build_map_sussess = BuildInitMap();//其实就是三角化点
        if(build_map_sussess)
        {
            std::cout << "初始化成功！" << std::endl;
            status_ = FrontendStatus::TRACKING_GOOD;
            //更新显示
            if(viewer_)
            {
                viewer_->AddCurrentFrame(last_frame_,current_frame_);
                viewer_->UpdataMap();
            }
            return true;
        }
        return false;
    }

    //来到这说明已经初始化完成。
    bool Frontend::Track()
    {
        if(last_frame_)
        {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());//relative_motion_初值是多少呢？相当于恒速模型跟踪
        }
        int num_track_last = TrackLastFrame();//光流法跟踪上一帧
        tracking_liliers_ = EstimateCurrentPose();//开始估计当前位姿
        // 根据跟踪的内点数判定跟踪状态
        if(tracking_liliers_ > num_features_tracking_)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
        }else if(tracking_liliers_ > num_features_tracking_bad_)
        {
            status_ = FrontendStatus::TRACKING_BAD;
        } else
        {
            status_ = FrontendStatus::LOST;
        }

        InsertKeyframe();//判断是否要插入关键帧
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if(viewer_)
        {
            viewer_->AddCurrentFrame(last_frame_,current_frame_);
        }

        std::cout << "跟踪完毕。" << std::endl;
        return true;
    }

    //判断是否要插入关键帧很简单，就看跟踪上的点数量够不够
    bool Frontend::InsertKeyframe()
    {
        if(tracking_liliers_ >= num_features_needed_for_keyframe_)
        {
            return false;
        }
        current_frame_->SetKeyFrame();//设置成关键帧
        map_->InsertKeyFrame(current_frame_);//并插入地图中
        std::cout << "将帧 " << current_frame_->id_ << " 设置为关键帧 "
                  << current_frame_->keyframe_id_;

        SetObservationsForKeyFrame();
        //在关键帧上再次提取特征点,并找到右图对应的特征点位置，然后三角化出新的路标点
        DeleteFeatures();
        FindFeaturesInRight() ;
        TriangulateNewPoints();
        backend_->UpdataMap();

        if (viewer_) viewer_->UpdataMap();

        return true;
    }

    void Frontend::SetObservationsForKeyFrame()
    {
        for(auto &feat : current_frame_->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if(mp)
            {
                mp->AddObservation(feat);
            }
        }
    }

    int Frontend::TriangulateNewPoints()
    {
        std::vector<SE3> poses{camera_left_->pose(),camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for(size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            // 地图点没有过期并且右图有对应的地图点
            if(current_frame_->features_left_.at(i)->map_point_.expired()
                    && current_frame_->features_right_.at(i) != nullptr)
            {
                // 三角化需要两帧位姿以及两帧归一化坐标,位姿上面已经有了，下面取归一化坐标
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(toVec2(current_frame_->features_left_.at(i)->position_.pt)),
                    camera_right_->pixel2camera(toVec2(current_frame_->features_right_.at(i)->position_.pt))
                };
                Vec3 pworld = Vec3::Zero();

                if(triangulation(poses,points,pworld) && pworld[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;//再转化到当前帧来
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                            current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                            current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        std::cout << "三角化出的新点个数: " << cnt_triangulated_pts << std::endl;
        return cnt_triangulated_pts;
    }

    int Frontend::EstimateCurrentPose()
    {
        // 创建求解器
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex 只估计和优化当前帧的位姿
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        Mat33 K = camera_left_->K();

        // 单元边
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;//feature里面是帧和特征的像素位置

        for(size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_.at(i)->map_point_.lock();
            if(mp)
            {
                features.push_back(current_frame_->features_left_.at(i));
                EdgeProjectionPoseOnly *edge =
                        new EdgeProjectionPoseOnly(mp->pose_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(current_frame_->features_left_.at(i)->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        const double chi2_th = 5.991;//要用卡方检验
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration)//进行四轮迭代优化
        {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);//每轮迭代10次
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outliner_)
                {
                    e->computeError();
                }
                //进行卡方检验，决定能不能丢弃
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outliner_ = true;
                    e->setLevel(1);//下次不优化
                    cnt_outlier++;
                } else
                {
                    features[i]->is_outliner_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2) //经过两轮迭代，精度就差不多了，不需要核函数了。
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        std::cout << "外点个数： " << cnt_outlier << " 内点个数 "
                  << features.size() - cnt_outlier << std::endl;
        // Set pose and outlier
        current_frame_->SetPose(vertex_pose->estimate());

        std::cout << "当前帧位姿： \n" << current_frame_->Pose().matrix() << std::endl;

        for (auto &feat : features)
        {
            if (feat->is_outliner_)
            {
                feat->map_point_.reset();//将地图点指针指向空
                feat->is_outliner_ = false;//以后可能还有用.
            }
        }
        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame()
    {
        //仍然利用光流法进行上一帧的跟踪
        std::vector<cv::Point2f> kps_last, kps_current;
        for(auto &kp : last_frame_->features_left_)
        {
            auto mp = kp->map_point_.lock();
            if(mp)//如果上一帧特征点对应的地图点还在
            {
                auto px =
                        camera_left_->world2pixel(mp->pose_,current_frame_->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }
        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(last_frame_->left_img_,current_frame_->left_img_,kps_last,kps_current,
                                 status,error,cv::Size(11,11),3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30,0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        int num_good_pts = 0;
        for(size_t i = 0; i < status.size(); i++)
        {
            if(status[i])
            {
                // 如果是好点，那么就把这个关键点封装成特征赋予当前帧。地图点也对应上。
                cv::KeyPoint kp(kps_current[i],7);
                Feature::Ptr feature(new Feature(current_frame_,kp));
                feature->map_point_ = last_frame_->features_left_.at(i)->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }
        std::cout << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    int Frontend::DeleteFeatures()
    {
        // 复制出一个用于接收特征点检测的图像
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1,255);
        for(auto &feat : current_frame_->features_left_)
        {
            //把特征点框出来
            cv::rectangle(mask,feat->position_.pt - cv::Point2f(10,10),
                          feat->position_.pt + cv::Point2f(10,10),0,CV_FILLED);
        }
        std::vector<cv::KeyPoint> keypoints;//gftt检测器得到的直接是关键点格式
        gftt_->detect(current_frame_->left_img_,keypoints,mask);
        int cnt_detected = 0;
        for(auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(Feature::Ptr(new Feature(current_frame_,kp)));
            cnt_detected++;
        }

        std::cout << "检测到了 " << cnt_detected << " 个特征点。" << std::endl;

        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        //根据左图像素位置，通过光流法得到右图的特征点位置
        std::vector<cv::Point2f> kps_left, kps_right;
        for(auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();//这就是用weak_ptr的原因(防内存泄漏),如果可以拿锁就能获得这个地图点（shared_ptr）,如果拿不到锁代表此地图点被释放了。
            //本质是看能不能找到这个地图点.
            if(mp)
            {
                auto px = camera_right_->world2pixel(mp->pose_,current_frame_->Pose());
                kps_right.push_back(cv::Point2f(px[0],px[1]));
            } else
            {
                kps_right.push_back(kp->position_.pt);
            }
        }
        std::vector<uchar> status;
        Mat error;
        //进行光流跟踪
        cv::calcOpticalFlowPyrLK(current_frame_->left_img_,current_frame_->right_img_,
                                 kps_left,kps_right,status,error,cv::Size(11,11),
                                 3,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        int num_good_pts = 0;
        for(size_t i = 0; i < status.size(); i++)
        {
            if(status[i])
            {
                cv::KeyPoint kp(kps_right[i],7);
                Feature::Ptr feat(new Feature(current_frame_,kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            } else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        std::cout << "在右图中找到 " << num_good_pts << " 个特征点." << std::endl;
        return num_good_pts;
    }

    //用SVD分解进行三角化，和VINS里方法一致
    bool Frontend::triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> &points, Vec3 &pt_world)
    {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i)
        {
            Mat34 m = poses[i].matrix3x4();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        // 保证第三个奇异值远远小于第二个奇异值。才是优质解。
        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
        {
            return true;
        }
        return false;
    }

    bool Frontend::BuildInitMap()
    {
        //初始化vector容器
        std::vector<SE3> poses{camera_left_->pose(),camera_right_->pose()};
        size_t cnt_init_landmarks = 0;

        for(size_t i = 0; i < current_frame_->features_left_.size(); i++)
        {
            //右目能找到对应的点才能三角化
            if(current_frame_->features_right_.at(i) == nullptr) continue;
            // 三角化点(SVD方法要求特征点的归一化相机坐标)
            std::vector<Vec3> points{
                camera_left_->pixel2camera(Vec2(current_frame_->features_left_.at(i)->position_.pt.x,
                                                current_frame_->features_left_.at(i)->position_.pt.y)),
                camera_right_->pixel2camera(Vec2(current_frame_->features_right_.at(i)->position_.pt.x,
                                                 current_frame_->features_right_.at(i)->position_.pt.y))
            };
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_.at(i));
                new_map_point->AddObservation(current_frame_->features_right_.at(i));

                current_frame_->features_left_.at(i)->map_point_ = new_map_point;
                current_frame_->features_right_.at(i)->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_init_landmarks++;
            }
        }

        current_frame_->SetKeyFrame();//把第一帧设为关键帧
        map_->InsertKeyFrame(current_frame_);//把关键帧插入地图中
        backend_->UpdataMap();
        std::cout << "初始化地图有 " << cnt_init_landmarks
                  << " 个地图点" << std::endl;
        if(cnt_init_landmarks > 50)
        {
            return true;
        }
        return false;
    }

    bool Frontend::Reset()
    {
        return true;
    }

    Vec2 Frontend::toVec2(const cv::Point2f p)
    {
        return Vec2(p.x,p.y);
    }
}


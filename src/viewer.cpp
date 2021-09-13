#include "viewer.h"
#include "feature.h"
#include "frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam
{
    Viewer::Viewer()
    {
        //std::bind用来将可调用对象与其参数一起进行绑定。绑定结果可以用std::funtion保存，并延迟调用到任何我们需要的时候。
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
        std::cout << "显示线程开启。" << std::endl;
    }

    void Viewer::Close()
    {
        viewer_running_ = false;
        viewer_thread_.join();
    }

    void Viewer::AddCurrentFrame(Frame::Ptr last_frame,Frame::Ptr currentframe)
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        std::cout << "给界面添加帧" << std::endl;
        last_frame_ = last_frame;
        current_frame_ = currentframe;
    }

    void Viewer::UpdataMap()
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_ != nullptr);//检查一下地图是不是不为空
        active_keyframes_ = map_->GetActiveKeyFrames();
        active_landmarks_ = map_->GetActiveMapPoints();
        std::cout << "地图更新设置为true" << std::endl;
        map_updated_ = true;
    }

    void Viewer::ThreadLoop()
    {
        pangolin::CreateWindowAndBind("Stereo Visual Slam",1024,768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
                pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display =
                pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                        .SetHandler(new pangolin::Handler3D(vis_camera));

        const float blue[3] = {0, 0, 1};
        const float green[3] = {0, 1, 0};

        cv::namedWindow("image",0);

        while (!pangolin::ShouldQuit() && viewer_running_)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            std::unique_lock<std::mutex> lock(viewer_data_mutex_);
            if (current_frame_)
            {
                DrawFrame(current_frame_, green);
                if(last_frame_)
                {
                    //这个其实只画了相邻两帧的连线，并不会保存之前的啊，再想想。
                    DrawTrajectory(last_frame_,current_frame_);
                }

                FollowCurrentFrame(vis_camera);

                cv::Mat img = PlotFrameImage();
                cv::imshow("image", img);
                cv::waitKey(1);
            }

            // 画地图点
            if (map_)
            {
                DrawMapPoints();
            }

            pangolin::FinishFrame();
            usleep(5000);
        }
        std::cout << "Stop viewer";
    }

    void Viewer::DrawFrame(Frame::Ptr frame, const float *color)
    {
        SE3 Twc = frame->Pose().inverse();

        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if (color == nullptr) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
    {
        SE3 Twc = current_frame_->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    cv::Mat Viewer::PlotFrameImage()
    {
        cv::Mat img_out;
        cv::cvtColor(current_frame_->left_img_,img_out,CV_GRAY2BGR);
        for(size_t i = 0; i < current_frame_->features_left_.size(); i++)
        {
            if(current_frame_->features_left_.at(i)->map_point_.lock())
            {
                auto feat = current_frame_->features_left_.at(i);
                cv::circle(img_out,feat->position_.pt,2,cv::Scalar(0,255,0),2);
            }
        }
        return img_out;
    }

    void Viewer::DrawMapPoints()
    {
        const float red[3] = {1.0, 0, 0};
        for (auto& kf : active_keyframes_) {
            DrawFrame(kf.second, red);
        }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto& landmark : active_landmarks_) {
            auto pos = landmark.second->GetPos();
            glColor3f(red[0], red[1], red[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }

    void Viewer::DrawTrajectory(Frame::Ptr frame1, Frame::Ptr frame2)
    {
        glColor3f(0.0, 0.0, 0.0);
        glLineWidth(2);
        glBegin(GL_LINES);
        auto p1 = frame1->Pose().inverse(), p2 = frame2->Pose().inverse();
        glVertex3f(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3f(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        glEnd();
    }
}
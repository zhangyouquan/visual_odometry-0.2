/*
		帧：每个图象帧都有自己的id号，关键帧呢有关键帧id
*/

#ifndef FRAME_H
#define FRAME_H
#include "camera.h"
#include "common.h"

namespace myslam
{
	//提前声明地图点和特征结构
	struct MapPoint;
	struct Feature;

	struct Frame
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

		unsigned long id_ = 0;
		unsigned long keyframe_id_ = 0;
		bool is_keyframe_ = false;//默认不是关键帧
		double time_stamp_;
		SE3 pose_;

		std::mutex pose_mutex_;// 互斥量
		cv::Mat left_img_, right_img_;
		
		// 声明存放特征的容器
		std::vector<std::shared_ptr<Feature>> features_left_;
		std::vector<std::shared_ptr<Feature>> features_right_;

	public:
		Frame(){}
		Frame(long id, double time_stamp,
              const SE3 &pose, const Mat &left_img, const Mat &right_img);
		~Frame() {}
		SE3 Pose()
		{
			std::unique_lock<std::mutex> lck(pose_mutex_);//保证线程安全
			return pose_;
		}

		void SetPose(const SE3 &pose)
		{
			std::unique_lock<std::mutex> lck(pose_mutex_);
			pose_ = pose;
		}

		//设置关键帧，并分配关键帧id
		void SetKeyFrame();

		// 工厂构建模式，分配id 
		static std::shared_ptr<Frame> CreateFrame();
	};



}
#endif
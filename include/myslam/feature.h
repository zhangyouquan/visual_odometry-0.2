#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "common.h"

namespace myslam
{
	struct Frame;
	struct MapPoint;

	/*
	2D 特征点
	在三角化之后会被关联一个地图点
	*/
	struct Feature
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		typedef std::shared_ptr<Feature> Ptr;

		std::weak_ptr<Frame> frame_;//这个特征点对应的帧
		cv::KeyPoint position_;//特征点像素位置
		std::weak_ptr<MapPoint> map_point_;//特征点与地图点关联

		bool is_outliner_ = false;
		bool is_on_left_image_ = true;//是否在左图上提取的

	public:
		Feature(){}
		Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp) 
			:frame_(frame),position_(kp)
		{

		}

	};

}

#endif
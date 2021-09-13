
/**
	MapPoint表示路标点。要估计他的世界坐标位置，要对应图像帧中的特征点，
	还有存储特征匹配的描述子信息。此外还要记录每个点被观测到的次数和被匹配的次数，
	这可以作为评价这个点的好坏程度的指标。
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common.h"

namespace myslam
{
	struct Frame;
	struct Feature;

	// 路标点
	struct MapPoint
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		typedef std::shared_ptr<MapPoint> Ptr;
		unsigned long id_ = 0;
		bool is_outlier_ = false;
		Vec3 pose_ = Vec3::Zero(); //世界坐标
		int observed_times_ = 0;
		/*
			 weak_ptr被设计为与shared_ptr共同工作，
			 可以从一个shared_ptr或者另一个weak_ptr对象构造，获得资源的观测权。
			 但weak_ptr没有共享资源，它的构造不会引起指针引用计数的增加。
			 同样，在weak_ptr析构时也不会导致引用计数的减少，它只是一个静静地观察者。

			关于shared_ptr和weak_ptr的知识见 https://blog.csdn.net/fengbingchun/article/details/52203825
		*/
		std::list<std::weak_ptr<Feature>> observations_;
		std::mutex data_mutex_;//互斥锁

		MapPoint(){}
		MapPoint(long id, Vec3 position):id_(id),pose_(position) {}

		//得到地图点坐标
		Vec3 GetPos()
		{
			std::unique_lock<std::mutex> lck(data_mutex_);
			return pose_;
		}

		//重置地图点
		void SetPos(const Vec3 &pos)
		{
			std::unique_lock<std::mutex> lck(data_mutex_);
			pose_ = pos;
		}

		//给地图点添加观测信息，并记录观测次数
		void AddObservation(std::shared_ptr<Feature> feature)
		{
			std::unique_lock<std::mutex> lck(data_mutex_);
			observations_.push_back(feature);
			observed_times_++;
		}
		
		// 抹除掉一些观测信息，就像ORB里一样找出一个最佳的特征
		void RemoveObservation(std::shared_ptr<Feature> feat);
		// 取观测信息
		std::list<std::weak_ptr<Feature>> GetObs()
		{
			std::unique_lock<std::mutex> lck(data_mutex_);
			return observations_;
		}

		// 工厂模式
		static MapPoint::Ptr CreateNewMappoint();
	};

}


#endif // !MAPPOINT_H


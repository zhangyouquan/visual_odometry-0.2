
#include "MapPoint.h"
#include "feature.h"

namespace myslam
{
	MapPoint::Ptr MapPoint::CreateNewMappoint()
	{
		static long factory_id = 0;
		MapPoint::Ptr new_mappoint(new MapPoint);
		new_mappoint->id_ = factory_id++;
		return new_mappoint;
	}

	void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat)
	{
		std::unique_lock<std::mutex> lck(data_mutex_);
		// ����������
		for (std::list<std::weak_ptr<Feature>>::iterator iter = observations_.begin(); iter != observations_.end();
			iter++)
		{
			if (iter->lock() == feat)
			{
				observations_.erase(iter);
				feat->map_point_.reset();//使map_point_指向空，也就是将建立的地图点设为空了
				observed_times_--;
				break;
			}
		}
	}
}
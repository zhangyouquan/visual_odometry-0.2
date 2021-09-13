
#include "frame.h"

namespace myslam
{
	Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left_img, const Mat &right_img)
		: id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left_img), right_img_(right_img){}

	// ����ģʽ
	Frame::Ptr Frame::CreateFrame()
	{
		static long factory_id = 0;
        Frame::Ptr new_frame(new Frame);
		new_frame->id_ = factory_id++;
		return new_frame;
	}

	void Frame::SetKeyFrame()
	{
		static long keyframe_factory_id = 0;
		is_keyframe_ = true;
		keyframe_id_ = keyframe_factory_id++;
	}
}
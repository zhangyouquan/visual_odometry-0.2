
/*
	Camera ��洢������ڲκ���Σ�������������ϵ����������ϵ������������ϵ
	֮�������任��
*/


#ifndef CAMERA_H
#define CAMERA_H

#include "common.h"

namespace myslam
{

	// ���ģ��:����һ�������
	class Camera 
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//��eigen�йأ��������������new�Ķ���汾���ء�
		typedef std::shared_ptr<Camera> Ptr;//����ָ��,��֤�̰߳�ȫ��

		double fx_ = 0,fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;//����ڲι���
		SE3 pose_; //λ�ˣ��൱����Σ�����
		SE3 pose_inv_;

		Camera();//���캯��
		Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose):fx_(fx), fy_(fy),
			cx_(cx), cy_(cy), baseline_(baseline), pose_(pose)
		{
			pose_inv_ = pose_.inverse();
		}

		// ��ȡλ��
		SE3 pose() const
		{
			return pose_;
		}

		// ��ȡ�ڲ�
		Mat33 K() const
		{
			Mat33 k;
			k << fx_, 0, cx_, 
				 0, fy_, cy_, 
				 0,   0,  1;
			return k;
		}

		// ����ת��
		Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);
		Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);
		Vec2 camera2pixel(const Vec3 &p_c);
		Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
		Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);
		Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
	};

}//myslam�����ռ�












#endif
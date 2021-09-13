#include "camera.h"

namespace myslam
{
	// ������
	Camera::Camera()
	{
	}

	//��������ϵ���������ϵ��ת��
	Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w)
	{
		return T_c_w * p_w;
	}

	Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w)
	{
		return T_c_w.inverse() * p_c;
	}

	//typedef Eigen::Matrix<double, 2, 1> Vec2;
	Vec2 Camera::camera2pixel(const Vec3 &p_c)
	{
		return Vec2(fx_ * p_c(0,0) / p_c(2,0) + cx_,
				fy_ * p_c(1, 0) / p_c(2, 0) + cy_
				);
	}

	Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth)
	{
		return Vec3((p_p(0, 0) - cx_) * depth / fx_,
					(p_p(1, 0) - cy_) * depth / fy_,
					depth
				);

		
	}

	Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w)
	{
		Vec3 p_c = world2camera(p_w, T_c_w);
		return camera2pixel(p_c);
	}

	Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) 
	{
		Vec3 p_c = pixel2camera(p_p, depth);
		return camera2world(p_c, T_c_w);
	}

}

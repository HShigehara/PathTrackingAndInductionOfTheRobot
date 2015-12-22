/*
* @file EV3Control.hpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief EV3を制御するためのクラスのヘッダ
* @date 2015.12.15
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __EV3CONTROL_HPP__
#define __EV3CONTROL_HPP__

#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class EV3Control
* @brief EV3を制御するためのクラス
*/
class EV3Control
{
private:

public:
	EV3Control(); //!<コンストラクタ
	~EV3Control(); //!<デストラクタ

	void set6DoFEV3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle); //!<最小二乗法によって求めた平均座標と位置をEV3の制御のために構造体に格納する(c80)
	DoF6d ev3_6dof; //!<EV3の6自由度(c80)

	void getVelocity(); //!<EV3の速度を計算する(c85)
	DoF6d before; //前フレームの情報
	DoF6d current; //現フレームの情報
	double velocity; //!<速度(c85)
	bool flag_velocity; //最初の1フレームのためのフラグ
};
/* インクルードガードの終了 */
#endif /* __EV3CONTROL_HPP__ */

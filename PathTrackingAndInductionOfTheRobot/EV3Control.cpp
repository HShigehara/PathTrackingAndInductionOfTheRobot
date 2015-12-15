/*
* @file EV3Control.cpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief EV3を制御するためのメソッド群
* @date 2015.12.15
* @author H.Shigehara
*/

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "EV3Control.hpp"

/*!
* @brief メソッドEV3Control::EV3Control().コンストラクタ
* @param なし
* @return なし
*/
EV3Control::EV3Control()
{
	//コンストラクタはなし
}

/*!
* @brief メソッドEV3Control::EV3Control().コンストラクタ
* @param なし
* @return なし
*/
EV3Control::~EV3Control()
{
	//デストラクタはなし
}

void EV3Control::set6DoFEV3(Point3d centroid, AttitudeAngle attitude_angle)
{
	ev3_6dof.x = centroid.x;
	ev3_6dof.y = centroid.y;
	ev3_6dof.z = centroid.z;
	ev3_6dof.yaw = attitude_angle.yaw;
	ev3_6dof.roll = attitude_angle.roll;
	ev3_6dof.pitch = attitude_angle.pitch;

	cout << ev3_6dof.x << ", " << ev3_6dof.y << ", " << ev3_6dof.z << ", " << ev3_6dof.yaw << ", " << ev3_6dof.roll << ", " << ev3_6dof.pitch << endl;
	return;
}
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
*/
EV3Control::EV3Control()
{
	//コンストラクタはなし
	current = { 0 };
	before = { 0 };
	flag_velocity = false; //1フレーム目のときは処理をしないようにするためのフラグ(c85)
}

/*!
* @brief メソッドEV3Control::EV3Control().コンストラクタ
*/
EV3Control::~EV3Control()
{
	//デストラクタはなし
}

/*!
 * @brief メソッドEV3Control::set6DoFEV3()．最小二乗法によって求めた平均座標と位置をEV3の制御のために構造体に格納する
 * @param pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle
 */
void EV3Control::set6DoFEV3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle)
{
	//FILE *fp;
	//char filepath_measuredata[NOC];
	//sprintf_s(filepath_measuredata, "data/%s/measuredata.dat", directoryName);
	//fopen_s(&fp, filepath_measuredata, "w");

	ev3_6dof.x = centroid.x;
	ev3_6dof.y = centroid.y;
	ev3_6dof.z = centroid.z;
	ev3_6dof.yaw = attitude_angle.yaw;
	ev3_6dof.roll = attitude_angle.roll;
	ev3_6dof.pitch = attitude_angle.pitch;
	cout << "[X, Y, Z, Yaw, Roll, Pitch, PointCloudNum]" << endl;
	cout << "[ " << ev3_6dof.x << ", " << ev3_6dof.y << ", " << ev3_6dof.z << ", " << ev3_6dof.yaw << ", " << ev3_6dof.roll << ", " << ev3_6dof.pitch << ", " << inputPointCloud->size() << " ]" << endl;

	//fprintf_s(fp, "[\tX\tY\tZ\tYaw\tRoll\tPitch\tPointCloudNum\t]\n");
	//fprintf_s(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z, ev3_6dof.yaw, ev3_6dof.roll, ev3_6dof.pitch, inputPointCloud->size());

	//fclose(fp);
	return;
}

/*!
 * @brief メソッドEV3Control::getVelocity()．EV3の速度を計算するメソッド
 */
void EV3Control::getVelocity()
{
	if (flag_velocity == false){ //1フレーム目の処理
		before.x = ev3_6dof.x; //速度の計算に必要なx座標
		before.z = ev3_6dof.z; //速度の計算に必要なz座標
		flag_velocity = true; //フラグをtrueにする
	}
	else{ //2フレーム目以降の処理
		cout << "c_x " << current.x*1000 << ", c_z " << current.z*1000 << ", b_x " << before.x*1000 << ", b_z " << before.z*1000 << endl;
		current.x = ev3_6dof.x; //現在のxの値を格納
		current.z = ev3_6dof.z; //現在のzの値を格納
		velocity = sqrt(pow((current.x-before.x),2)+pow((current.z-before.z),2));
		before.x = current.x;
		before.z = current.z;
	}

	cout << "Velocity => " << velocity/1000 << "[mm/frame]" << endl;
	return;
}
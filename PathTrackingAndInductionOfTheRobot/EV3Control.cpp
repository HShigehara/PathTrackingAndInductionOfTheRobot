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

	current_average = { 0 };
	before1_average = { 0 };
	before2_average = { 0 };
	before3_average = { 0 };
	before4_average = { 0 };
	before5_average = { 0 };
	count_average = 0;
	flag_average = false;
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

	ev3_6dof.x = round(centroid.x);
	ev3_6dof.y = round(centroid.y);
	ev3_6dof.z = round(centroid.z);
	ev3_6dof.yaw = round(attitude_angle.yaw);
	ev3_6dof.roll = round(attitude_angle.roll);
	ev3_6dof.pitch = round(attitude_angle.pitch);
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
		before.y = ev3_6dof.y; //速度の計算に必要なy座標
		before.z = ev3_6dof.z; //速度の計算に必要なz座標
		flag_velocity = true; //フラグをtrueにする
	}
	else{ //2フレーム目以降の処理
		//cout << "c_x " << current.x << ", c_z " << current.z << ", b_x " << before.x << ", b_z " << before.z << endl;
		current.x = ev3_6dof.x; //現在のxの値を格納
		current.y = ev3_6dof.y; //現在のyの値を格納
		current.z = ev3_6dof.z; //現在のzの値を格納
		velocity = sqrt(pow((current.x-before.x),2)+pow((current.y-before.y),2)+pow((current.z-before.z),2));
		before.x = current.x;
		before.y = current.y;
		before.z = current.z;
	}

	cout << "Velocity => " << velocity << "[mm/frame]" << endl;
	return;
}

ControlParamd EV3Control::getAverageVelocityAndYaw()
{
	//最初の5フレームはデータを保存するだけ
	if (flag_average == false){ //5フレームの間の処理
		if (count_average == 0){
			cout << "1フレーム目なので保存" << endl;
			current_average.velocity = velocity;
			current_average.yaw = ev3_6dof.yaw;
			before5_average = current_average;
			count_average++;
		}
		else if (count_average == 1){
			cout << "2フレーム目なので保存" << endl;
			current_average.velocity = (velocity + before5_average.velocity) / 2.0;
			current_average.yaw = (ev3_6dof.yaw + before5_average.yaw) / 2.0;
			before4_average = current_average;
			count_average++;
		}
		else if (count_average == 2){
			cout << "3フレーム目なので保存" << endl;
			current_average.velocity = (velocity + before4_average.velocity + before5_average.velocity) / 3.0;
			current_average.yaw = (ev3_6dof.yaw + before4_average.yaw + before5_average.yaw) / 3.0;
			before3_average = current_average;
			count_average++;
		}
		else if (count_average == 3){
			cout << "4フレーム目なので保存" << endl;
			current_average.velocity = (velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 4.0;
			current_average.yaw = (ev3_6dof.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 4.0;
			before2_average = current_average;
			count_average++;
		}
		else if (count_average == 4){
			cout << "5フレーム目なので保存" << endl;
			current_average.velocity = (velocity + before2_average.velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 5.0;
			current_average.yaw = (ev3_6dof.yaw + before2_average.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 5.0;
			before1_average = current_average;
			count_average++;
			flag_average = true;
		}
	}
	else{
		//現在の速度の平均を計算する
		current_average.velocity = (velocity + before1_average.velocity + before2_average.velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 6.0; //現在と過去5フレーム分の速度から現在の速度を計算
		//現在のヨー角の平均を計算する
		current_average.yaw = (ev3_6dof.yaw + before1_average.yaw + before2_average.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 6.0;

		//現在の速度とヨー角の平均を1フレームずつずらす
		before1_average = current_average;
		before2_average = before1_average;
		before3_average = before2_average;
		before4_average = before3_average;
		before5_average = before4_average;
	}
	cout << "[Velocity, Yaw] => " << "[ " << current_average.velocity << " , " << current_average.yaw << " ]" << endl;
	return current_average;
}
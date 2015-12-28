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
	current = { 0 }; //前フレームの6DoFを初期化
	before = { 0 }; //現フレームの6DoFを初期化
	flag_velocity = false; //1フレーム目のときは処理をしないようにするためのフラグ(c85)

	current_average = { 0 }; //現フレーム前の平均速度と平均ヨー角の初期化
	before1_average = { 0 }; //1フレーム前の平均速度と平均ヨー角の初期化
	before2_average = { 0 }; //2フレーム前の平均速度と平均ヨー角の初期化
	before3_average = { 0 }; //3フレーム前の平均速度と平均ヨー角の初期化
	before4_average = { 0 }; //4フレーム前の平均速度と平均ヨー角の初期化
	before5_average = { 0 }; //5フレーム前の平均速度と平均ヨー角の初期化
	count_average = 0; //速度とヨー角の過去5フレーム分の平均を取るために最初の5フレームをカウントするための変数
	flag_average = false; //始めの5フレーム分をチェックするためのフラグ

	save_flag = false; //6DoF情報を出力するかチェックするためのフラグ

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
	ev3_6dof.x = round(centroid.x);
	ev3_6dof.y = round(centroid.y);
	ev3_6dof.z = round(centroid.z);
	ev3_6dof.yaw = round(attitude_angle.yaw);
	ev3_6dof.roll = round(attitude_angle.roll);
	ev3_6dof.pitch = round(attitude_angle.pitch);
	cout << "[X, Y, Z, Yaw, Roll, Pitch, PointCloudNum]" << endl;
	cout << "[ " << ev3_6dof.x << ", " << ev3_6dof.y << ", " << ev3_6dof.z << ", " << ev3_6dof.yaw << ", " << ev3_6dof.roll << ", " << ev3_6dof.pitch << ", " << inputPointCloud->size() << " ]" << endl;

	return;
}

/*!
 * @brief メソッドEV3Control::getVelocity()．EV3の速度を計算するメソッド
 */
void EV3Control::getVelocity()
{
	if (flag_velocity == false){ //1フレーム目の処理
		before = ev3_6dof; //速度の計算に必要な座標を保存
		flag_velocity = true; //フラグをtrueにする
	}
	else{ //2フレーム目以降の処理
		current = ev3_6dof; //現在の値を格納
		velocity = sqrt(pow((current.x-before.x),2)+pow((current.y-before.y),2)+pow((current.z-before.z),2));
		before = current; //現在の情報を過去の情報に更新する
	}
	cout << "Velocity => " << velocity << "[mm/frame]" << endl; //確認用
	return;
}

void EV3Control::getAverageVelocityAndYaw()
{
	//最初の5フレームはデータを保存するだけ
	if (flag_average == false){ //5フレームの間の処理
		if (count_average == 0){ //1フレーム目の処理
			current_average.velocity = velocity;
			current_average.yaw = ev3_6dof.yaw;
			before5_average = current_average;
			count_average++;
		}
		else if (count_average == 1){ //2フレーム目の処理
			current_average.velocity = (velocity + before5_average.velocity) / 2.0;
			current_average.yaw = (ev3_6dof.yaw + before5_average.yaw) / 2.0;
			before4_average = current_average;
			count_average++;
		}
		else if (count_average == 2){ //3フレーム目の処理
			current_average.velocity = (velocity + before4_average.velocity + before5_average.velocity) / 3.0;
			current_average.yaw = (ev3_6dof.yaw + before4_average.yaw + before5_average.yaw) / 3.0;
			before3_average = current_average;
			count_average++;
		}
		else if (count_average == 3){ //4フレーム目の処理
			current_average.velocity = (velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 4.0;
			current_average.yaw = (ev3_6dof.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 4.0;
			before2_average = current_average;
			count_average++;
		}
		else if (count_average == 4){ //5フレーム目の処理
			current_average.velocity = (velocity + before2_average.velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 5.0;
			current_average.yaw = (ev3_6dof.yaw + before2_average.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 5.0;
			before1_average = current_average;
			flag_average = true;
		}
	}
	else{ //6フレーム以降
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
	cout << "5 Frame Average [Velocity, Yaw] => " << endl;// << "[ " << round(current_average.velocity) << " , " << round(current_average.yaw) << " ]" << endl;
	cout << current_average.velocity << endl;
	return;
}

/*!
 * @brief メソッドEV3Control::output6Dof()．現フレームの6DoF情報をファイルに出力する
 */
void EV3Control::output6DoF(int save_count, char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud)
{
	//6DoF情報を保存
	FILE *dof6_fp; //最終1フレーム分．gnuplotで表示するために平均座標(重心)をファイルに出力する用
	char filepath_dof6[NOC];
	sprintf_s(filepath_dof6, "%s/%s-%02d.dat", original_dirpath, output_filename, save_count);
	fopen_s(&dof6_fp, filepath_dof6, "w");
	fprintf_s(dof6_fp, "%d %d %d %d %d %d %d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z, ev3_6dof.yaw, ev3_6dof.roll, ev3_6dof.pitch, outputPointCloud->size());
	fclose(dof6_fp);

	return;
}

/*!
 * @brief メソッドEV3Control::output6DoFContinuous()．キーを入力したときの6DoF情報を連続してcsv形式で保存する
 */
void EV3Control::output6DoFContinuous(char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputPointCloud)
{
	FILE *dof6con_fp;
	char filepath_dof6con[NOC];
	sprintf_s(filepath_dof6con, "%s/%s.csv", original_dirpath, output_filename);
	fopen_s(&dof6con_fp, filepath_dof6con, "a");
	if (save_flag == false){
		fprintf(dof6con_fp, "x,y,z,Yaw,Roll,Pitch,Data Size\n");
	}
	fprintf_s(dof6con_fp, "%d,%d,%d,%d,%d,%d,%d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z, ev3_6dof.yaw, ev3_6dof.roll, ev3_6dof.pitch, outputPointCloud->size());
	fclose(dof6con_fp);

	return;
}

/*!
 * @brief メソッドEV3Control::outputEV3RouteContinuous()．EV3の走行軌道を保存する
 */
void EV3Control::outputEV3RouteContinuous(char* original_dirpath, char* output_filename)
{
	//保存用のファイル作成
	FILE *ev3route;
	char filepath_ev3route[NOC];
	sprintf_s(filepath_ev3route, "%s/%s.dat", original_dirpath, output_filename);
	fopen_s(&ev3route, filepath_ev3route, "a"); //ファイルオープン
	fprintf_s(ev3route, "%d %d %d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z); //データをファイルに書き込む
	fclose(ev3route);

	return;
}

/*!
 * @brief メソッドEV3Control::outputControlInformation()．EV3の制御情報を出力する
 */
void EV3Control::outputControlInformation(double sumtime_ms, char* original_dirpath, char* output_filename)
{
	FILE *time_averagevandyaw;
	char filepath_timeaveragevandyaw[NOC];
	sprintf_s(filepath_timeaveragevandyaw, "%s/%s.dat", original_dirpath, output_filename);
	fopen_s(&time_averagevandyaw, filepath_timeaveragevandyaw, "a");
	fprintf_s(time_averagevandyaw, "%f %f %f\n", sumtime_ms / 1000.0, current_average.velocity, current_average.yaw);
	fclose(time_averagevandyaw);

	return;
}
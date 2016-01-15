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
	ControlParamd before1_average; //!<1フレーム前の平均速度と平均ヨー角
	ControlParamd before2_average; //!<2フレーム前の平均速度と平均ヨー角
	ControlParamd before3_average; //!<3フレーム前の平均速度と平均ヨー角
	ControlParamd before4_average; //!<4フレーム前の平均速度と平均ヨー角
	ControlParamd before5_average; //!<5フレーム前の平均速度と平均ヨー角

public:
	EV3Control(); //!<コンストラクタ
	~EV3Control(); //!<デストラクタ

	void set6DoFEV3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle); //!<最小二乗法によって求めた平均座標と位置をEV3の制御のために構造体に格納する(c80)
	DoF6i ev3_6dof; //!<EV3の6自由度(c80)

	void getVelocityinSec(double time_ms); //!<EV3の速度を計算する(c85)
	DoF6i before; //!<前フレームの6DoF情報
	DoF6i current; //!<現フレームの6DoF情報
	double velocity; //!<速度v(c85)
	bool flag_velocity; //!<最初の1フレームのためのフラグ

	void getAverageVelocityAndYaw(); //!<平均の速度とヨー角を計算する
	ControlParamd current_average; //!<現在の平均の速度とヨー角
	int count_average; //!<速度とヨー角の過去5フレーム分の平均を取るために最初の5フレームをカウントするための変数
	bool flag_average; //!<始めの5フレーム分用

	//データ出力メソッド
	void output6DoF(int save_count, char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud); //!<現フレームの6DoF情報をファイルに出力する
	void output6DoFContinuous(char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud); //!<キーを入力したときの6DoF情報を連続してcsv形式で保存する
	bool save_flag; //!<6DoF情報を出力するかチェックするためのフラグ
	void outputEV3RouteContinuous(char* original_dirpath, char* output_filename); //!<EV3の走行軌道を保存する
	void outputControlInformation(double sumtime_ms, char* original_dirpath, char* output_filename); //!<EV3の制御情報を出力する
	void outputControlInformation(); //!<EV3に必要な速度とヨー角をファイルに出力
};

/* インクルードガードの終了 */
#endif /* __EV3CONTROL_HPP__ */

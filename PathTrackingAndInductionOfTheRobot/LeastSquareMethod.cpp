/*
 * @file LeastSquareMethod.cpp
 * @brief 最小二乗法を行うためのメソッド群
 * @date 2015.07.16
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "LeastSquareMethod.hpp"

/*!
 * @brief メソッドLeastSquareMethod::LeastSquareMethod().コンストラクタ
 */
LeastSquareMethod::LeastSquareMethod()
{
	//コンストラクタ
}

/*!
* @brief メソッドLeastSquareMethod::~LeastSquareMethod().デストラクタ
*/
LeastSquareMethod::~LeastSquareMethod()
{
	//デストラクタ
}

/*!
 * @brief メソッドLeastSquareMethod::getCoefficient()．最小二乗法によって平面ax+by+c=0の係数[a b c]'を求めるメソッド
 * @param &inputPointCloud pcl::PointCloud<pcl::PointXYARGB>::Ptr型．入力するポイントクラウド 
 * @return coefficient_plane Eigen::Vector3f型．平面の係数
 */
Eigen::Vector3f LeastSquareMethod::getCoefficient(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	Eigen::Vector3f coefficient_plane(0.0, 0.0, 0.0);
	Eigen::Matrix3f m1;
	m1.Zero();
	Eigen::Vector3f m2(0.0, 0.0, 0.0);

	//3×3行列 S=[Sxx Sxy Sx;Sxy Syy Sy;Sx Sy inputPointCloud->size()]の初期化
	double Szz = 0, Sxz = 0, Syz = 0, Sz = 0, Sxx = 0, Sxy = 0, Sx = 0, Syy = 0, Sy = 0;
	//最小二乗法によって求めたA=[a b c]'の要素の初期化
	//double a = 0, b = 0, c = 0;
	//cout << "input Size => " << inputPointCloud->size() << endl;
	for (int i = 1; i < inputPointCloud->size(); i++){ //それぞれの要素の計算
		Szz = Szz + inputPointCloud->points[i].z * inputPointCloud->points[i].z;
		Sxz = Sxz + inputPointCloud->points[i].x * inputPointCloud->points[i].z;
		Syz = Syz + inputPointCloud->points[i].y * inputPointCloud->points[i].z;
		Sz = Sz + inputPointCloud->points[i].z;
		Sxx = Sxx + inputPointCloud->points[i].x * inputPointCloud->points[i].x;
		Sxy = Sxy + inputPointCloud->points[i].x * inputPointCloud->points[i].y;
		Sx = Sx + inputPointCloud->points[i].x;
		Syy = Syy + inputPointCloud->points[i].y * inputPointCloud->points[i].y;
		Sy = Sy + inputPointCloud->points[i].y;
	}

	m1 << Sxx, Sxy, Sx,
		Sxy, Syy, Sy,
		Sx, Sy, inputPointCloud->size();
	//cout << "m1 => \n" << m1 << endl;
	//cout << "m1_inverse => \n" << m1.inverse() << endl;

	m2 << Sxz, Syz, Sz;
	//cout << "m2 => \n" << m2 << endl;

	coefficient_plane = m1.inverse() * m2;
	//cout << "coefficient_plane => \n" << coefficient_plane << endl;

	return coefficient_plane;
}

/*!
 * @brief メソッドLeastSquareMethod::calcYawRollPitch()．最小二乗法によって求めた[a b c]'を用いて平面の姿勢を計算する
 * @param coefficient_plane Eigen::Vector3f．平面の係数
 * @return attitude_angle_deg AttitudeAngle3d型．姿勢角[deg]
 */
AttitudeAngle3d LeastSquareMethod::calcYawRollPitch(Eigen::Vector3f coefficient_plane)
{
	AttitudeAngle3d attitude_angle_rad; //姿勢角(ラジアン)
	AttitudeAngle3d attitude_angle_deg; //姿勢角(度)

	//ヨー角の計算
	attitude_angle_rad.yaw = -atan(coefficient_plane.x());
	attitude_angle_deg.yaw = attitude_angle_rad.yaw / M_PI * 180.0;

	//ロール角の計算
	attitude_angle_rad.roll = atan2(-coefficient_plane.x(), coefficient_plane.y());
	//attitude_angle_rad.roll = atan2(-coefficient_plane.y(), coefficient_plane.x());
	//attitude_angle_rad.roll = -atan2(-coefficient_plane.x(), coefficient_plane.y());
	
	//xが+ならロール角に+90°して補正，xが-ならロール角に-90°して補正(c85)
	if (coefficient_plane.x() >= 0){
		attitude_angle_deg.roll = (attitude_angle_rad.roll / M_PI * 180.0) + 90;
	}
	else if (coefficient_plane.x() < 0){
		attitude_angle_deg.roll = (attitude_angle_rad.roll / M_PI * 180.0) - 90;
	}

	//ピッチ角の計算
	attitude_angle_rad.pitch = atan2(1, coefficient_plane.y());
	attitude_angle_deg.pitch = attitude_angle_rad.pitch / M_PI * 180.0;

	return attitude_angle_deg;
}
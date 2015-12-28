/*
 * @file LeastSquareMethod.hpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief 最小二乗法を行うクラスのヘッダ
 * @date 2015.07.16
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __LEASTSQUAREMETHOD_HPP__
#define __LEASTSQUAREMETHOD_HPP__

/* インクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class LeastSquareMethod
* @brief 最小二乗法を行うクラス
*/
class LeastSquareMethod
{
private:

public:
	LeastSquareMethod(); //!<コンストラクタ
	~LeastSquareMethod(); //!<デストラクタ

	//最小二乗法
	Eigen::Vector3f getCoefficient(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //!<最小二乗法によって平面ax+by+c=0の係数[a b c]'を求めるメソッド
	Eigen::Vector3f coefficient_plane; //!<平面の係数
	AttitudeAngle3d calcYawRollPitch(Eigen::Vector3f coefficient_plane); //!<最小二乗法によって求めた[a b c]'を用いて平面の姿勢を計算する
	AttitudeAngle3d attitude_angle; //!<姿勢角(c78)
};

/* インクルードガードの終了 */
#endif /* __LEASTSQUAREMETHOD_HPP__ */
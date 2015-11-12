/*
* @file LeastSquareMethod.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief 最小二乗法を行うクラスのヘッダ
* @date 2015.07.16
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __LEASTSQUAREMETHOD_HPP__
#define __LEASTSQUAREMETHOD_HPP__

/* インクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

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

	void getSphereData(int actualExtractedNum); //!<最小二乗法によって求めた球の情報を取得するメソッド
};

/* インクルードガードの終了 */
#endif /* __LEASTSQUAREMETHOD_HPP__ */
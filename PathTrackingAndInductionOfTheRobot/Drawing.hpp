/*
 * @file Drawing.hpp 
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief gnuplot処理関連のクラスのヘッダ
 * @date 2014.12.10
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __DRAWING_HPP__
#define __DRAWING_HPP__

/* インクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class Drawing
* @brief 経路描画用のクラス
*/
class Drawing
{
private:
	FILE *gnuplot; //プロット用
	FILE *gpr; //リアルタイム出力用のプロット(c43)
	bool first = true; //
	int plotMode = 1; //!<gnuplotのスクリプトをファイルに出力する際に，計測する対象の状況に応じて値の範囲を変更する.中心固定用(1)，縦移動用(2)，横移動用(3)(c46)，自動縮尺(4)(c47)

	int save_count;

public:
	Drawing(); //!<コンストラクタ
	~Drawing(); //!<デストラクタ
	void plot3D(const string* outputDataName); //!<3D座標ファイルをプロットするメソッド
	void gnuplotScript(const string* dataFileName); //
	void plot3DRealTime(int countDataNum, outputData outputData[OUTPUTDATA_MAX]); //リアルタイムでgnuplotに出力(c43)
	void gnuplotScriptCoG(const string* cogFileName); //!<球の重心座標をプロットするメソッド(c52)

	void gnuplotScriptEV3Unit(Eigen::Vector3f coefficient_plane); //!<EV3の点群をプロットするためのスクリプト(c78)
	
	void gnuplotScriptEV3Route();
};

/* インクルードガードの終了 */
#endif /* __DRAWING_HPP__ */
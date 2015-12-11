/*
* @file DrawingMethod.hpp 
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief gnuplot処理関連のクラスのヘッダ
* @date 2014.12.10
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __DRAWINGMETHOD_HPP__
#define __DRAWINGMETHOD_HPP__

/* インクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

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

	FILE *centroid; //!<重心座標用のファイル出力ストリーム(c76)
	FILE *splot_ev3unit; //!<EV3のユニット用(gnuplot)(c78)

public:
	Drawing(); //!<コンストラクタ
	~Drawing(); //!<デストラクタ
	void plot3D(const string* outputDataName); //!<3D座標ファイルをプロットするメソッド
	void gnuplotScript(const string* dataFileName); //
	void plot3DRealTime(int countDataNum, outputData outputData[OUTPUTDATA_MAX]); //リアルタイムでgnuplotに出力(c43)
	void gnuplotScriptCoG(const string* cogFileName); //!<球の重心座標をプロットするメソッド(c52)

	void outputEV3Route(Point3f ev3_centroid); //!<EV3の重心を渡したらファイルに出力するメソッド(c76)

	void gnuplotScriptEV3Unit(Eigen::Vector3f coefficient_plane); //!<EV3の点群をプロットするためのスクリプト(c78)
};

/* インクルードガードの終了 */
#endif /* __DRAWINGMETHOD_HPP__ */
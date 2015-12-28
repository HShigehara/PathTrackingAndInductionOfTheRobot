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

	//int save_count;

public:
	Drawing(); //!<コンストラクタ
	~Drawing(); //!<デストラクタ

	void gnuplotScriptEV3(int save_count, char* original_dirpath, char* output_filename, Eigen::Vector3f coefficient_plane); //!<EV3の点群をプロットするためのスクリプト(c78)
	void gnuplotScriptEV3Route(char* original_dirpath, char* output_filename); //!<EV3の軌道をプロットするためのスクリプト

	void gnuplotScriptTime2V(); //!<時間と速度のプロット
	void gnuplotScriptTime2Yaw(); //!<時間とヨー角のプロット

};

/* インクルードガードの終了 */
#endif /* __DRAWING_HPP__ */
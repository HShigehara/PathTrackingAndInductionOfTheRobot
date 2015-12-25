/*
* @file OutputDatafile.hpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief データをファイルに出力するためのクラスのヘッダ
* @date 2015.12.25
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __OUTPUTDATAFILE_HPP__
#define __OUTPUTDATAFILE_HPP__

/* インクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class OutputDatafile
* @brief データをファイルに出力するためのクラス
*/
class OutputDatafile
{
private:

public:
	OutputDatafile(); //!<コンストラクタ
	~OutputDatafile(); //!<デストラクタ


	void saveDataEveryEnterKey(Mat& current_image, Mat& bin_image, DoF6i dof6, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, double fps);
	bool save_flag; //!<6DoF情報を出力するフラグ
	void saveDataContinuously(double sum_time, DoF6i centroid, ControlParamd current); //平均座標を連続で保存する
};

/* インクルードガードの終了 */
#endif /* __OUTPUTDATAFILE_HPP__ */
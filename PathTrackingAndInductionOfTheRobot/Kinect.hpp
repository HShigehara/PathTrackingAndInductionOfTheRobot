/*
 * @file KinectMethod.hpp
 * @link https://github.com/HShigehara/PathTPATHTRACKINGANDINDUCTIONTHEROBOTrackingAndInductionOfTheRobot.git
 * @brief kinect処理のクラスのヘッダ
 * @date 2014.12.10
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __KINECT_HPP__
#define __KINECT_HPP__

/* インクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*! @brief エラーチェック */
#define ERROR_CHECK(ret)\
if (ret != S_OK){ \
stringstream ss; \
ss << "faild" #ret " " << hex << ret << endl; \
throw runtime_error(ss.str().c_str()); \
}

/*! Kinectの解像度の設定 */
const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

/*!
* @class Kinect
* @brief Kinect操作用のクラス
*/
class Kinect
{
private:
	INuiSensor* kinect; //!<INuiSensor型のポインタ
	HANDLE imageStreamHandle; //!<RGBカメラのストリームデータを扱うためのハンドル
	HANDLE depthStreamHandle; //!<Depthカメラのストリームデータを扱うためのハンドル
	DWORD width; //!<幅
	DWORD height; //!<高さ
	int countKinect; //!<接続されているKinectの数をカウントする変数
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //!<点群データ保存用(c57)

public:
	Kinect(); //!<コンストラクタ
	~Kinect(); //!<デストラクタ
	
	void initialize(); //!<Kinectの初期化
	void createInstance(); //!<インスタンスの生成

	Mat drawRGBImage(Mat& image); //!<RGBカメラの処理
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(Mat& Mat_image); //!<Depthカメラの処理(c57)
	int getDistance(Mat& image); //!<距離を取得(c49)
	
	HANDLE streamEvent; //!<RGB,Depthカメラのフレーム更新イベントを待つためのイベントハンドル
	int key; //!<ウィンドウ表示のウェイトタイム格納変数
	int actualExtractedNum; //実際に距離が抽出された数(0以外だった数)(c31)
};

/* インクルードガードの終了 */
#endif /* __KINECT_HPP__ */
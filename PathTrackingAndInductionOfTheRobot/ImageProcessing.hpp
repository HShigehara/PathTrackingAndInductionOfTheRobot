/*
 * @file ImageProcessing.hpp 
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief 画像処理関連のクラスのヘッダ
 * @date 2014.12.10
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __IMAGEPROCESSING_HPP__
#define __IMAGEPROCESSING_HPP__

/* インクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class ImageProcessing
* @brief 画像処理用のクラス
*/
class ImageProcessing
{
private:
	//キャリブレーション用のカメラパラメータ(c71)
	Mat internal_cameraparam; //!<カメラ内部パラメータ
	Mat distortion_coefficients; //!<歪み係数

public:
	ImageProcessing(); //!<コンストラクタ
	~ImageProcessing(); //!<デストラクタ

	void showImage(string windowName, Mat& input_image); //!<ウインドウの名前を引数に追加(c31)．Matの表示(c17)
	//複数の画像を1つのウインドウにまとめて表示する．オーバーロード(c36)
	void showImageTogether(Mat& image1, Mat& image2); //!<2つの画像を一緒に表示(c36)
	void showImageTogether(Mat& image1, Mat& image2, Mat& image3); //!<3つの画像を一緒に表示(c36)
	
	void loadInternalCameraParameter(const string cameraParamFile); //!<カメラキャリブレーションによって得られたパラメータを適用する(c54)
	Mat getUndistortionImage(Mat& inputOriginalImage); //!<キャリブレーションデータを用いてKinectから取得した画像を補正する(c71)
	Mat getBackgroundSubstractionBinImage(Mat& current_image, Mat& backgound_gray_image/*, int threshold, int neighborhood, int closing_times*/); //!<背景差分によって得られた二値画像(c75)
	int th; //!<二値化するときの閾値(c82)
	int neighborhood;
	int closing_times;
	Mat getUnitMask(Mat& input_binimage);

	void openCVSettingTrackbar(const string trackbar_name);
};

/* インクルードガードの終了 */
#endif /* __IMAGEPROCESSING_HPP__ */
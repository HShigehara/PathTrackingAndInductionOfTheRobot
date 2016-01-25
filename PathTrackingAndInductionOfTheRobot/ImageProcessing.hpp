/*
 * @file ImageProcessing.hpp 
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
	Mat getBackgroundSubstractionBinImage(Mat& current_image, Mat& backgound_gray_image); //!<背景差分によって得られた二値画像(c75)

	void openCVSettingTrackbar(const string trackbar_name); //!<画像処理関連のトラックバーを表示するメソッド
	int th; //!<二値化するときの閾値(c82)
	int neighborhood; //!<平滑化を行うときの近傍
	int closing_times; //!<クロージングを行う回数

	Mat getUnitMask(Mat& input_binimage); //!<EV3のユニット部のみのマスク画像を取得するメソッド

	void outputImageSelectDirectory(int save_count, char* original_dirpath, char* save_filename, Mat& output_image); //画像を指定したディレクトリに保存(c86)
};

/* インクルードガードの終了 */
#endif /* __IMAGEPROCESSING_HPP__ */
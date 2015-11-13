/*
* @file ImageProcessingMethod.hpp 
* @link https://github.com/HShigehara/Masters-Thesis.git 
* @brief 画像処理関連のクラスのヘッダ
* @date 2014.12.10
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __IMAGEPROCESSINGMETHOD_HPP__
#define __IMAGEPROCESSINGMETHOD_HPP__

/* インクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class ImageProcessing
* @brief 画像処理用のクラス
*/
class ImageProcessing
{
private:
	//Mat hsvceimg; //!<色抽出後のHSV画像
	/* (c26) */
	Mat hue;
	Mat mask;
	Mat hist;
	Mat backproj;
	int _vmin = vmin;
	int _vmax = vmax;

public:
	ImageProcessing(); //!<コンストラクタ
	~ImageProcessing(); //!<デストラクタ
	void showImage(char* windowName, Mat& input_image); //!<ウインドウの名前を引数に追加(c31)．Matの表示(c17)
	//複数の画像を1つのウインドウにまとめて表示する．オーバーロード(c36)
	void showTogetherImage(Mat& image1, Mat& image2); //!<2つの画像を一緒に表示(c36)
	void showTogetherImage(Mat& image1, Mat& image2, Mat& image3); //!<3つの画像を一緒に表示(c36)
	Mat convertRGB2HSV(Mat& input_image); //!<RGB画像imageをHSVに変換するメソッド
	Mat extractColor(Mat& input_hsvimg); //!<変換したHSV画像から特定の色を抽出するメソッド
	Mat getCoordinate(Mat& input_binimg); //!<二値画像から座標を抽出するメソッド(c40)
	/*オープニング処理用にとっておく．(c31)*/
	//Mat extractBlack(Mat& opening_img); //!<オープニング処理後の画像から座標を抽出する(c24)
	Mat grayscaleImage(Mat& hsv_ceimg); //!<グレースケールに変換(c19)
	Mat binarizationImage(Mat& gray_img); //!<抽出した画像の二値化(c19)
	Mat OpeningImage(Mat& bin_img); //!<抽出して二値化した画像に対してオープニング処理(同じ数だけ縮小→膨張)を行う(c19)
	void trackingObject(Mat& hsv_img); //!<対象を追跡する(c26)
	void drawCenterPoint(Mat& image, Point3ius averageCoordinate/*, const string* mainWindowName*/); //!<計算した平均座標を画面上に表示する(c45)

	Mat mTrim_img;

	//中間発表以降，背景差分処理により動体を検出する(c66)
	Mat backGroundSubstraction(/*Mat& image*/); //!<背景差分処理により前景画像を取得する(c66)
	Mat foreGroundMask_image; //!<前景画像(c66)
};

/* インクルードガードの終了 */
#endif /* __IMAGEpROCESSINGMETHOD_HPP__ */
﻿/*
* @file ImageProcessing.hpp 
* @link https://github.com/HShigehara/Masters-Thesis.git 
* @brief 画像処理関連のクラスのヘッダ
* @date 2014.12.10
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __IMAGEPROCESSING_HPP__
#define __IMAGEPROCESSING_HPP__

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

	//キャリブレーション用のカメラパラメータ(c71)
	Mat internal_cameraparam; //!<カメラ内部パラメータ
	Mat distortion_coefficients; //!<歪み係数

public:
	ImageProcessing(); //!<コンストラクタ
	~ImageProcessing(); //!<デストラクタ

	void showImage(char* windowName, Mat& input_image); //!<ウインドウの名前を引数に追加(c31)．Matの表示(c17)
	//複数の画像を1つのウインドウにまとめて表示する．オーバーロード(c36)
	void showImageTogether(Mat& image1, Mat& image2); //!<2つの画像を一緒に表示(c36)
	void showImageTogether(Mat& image1, Mat& image2, Mat& image3); //!<3つの画像を一緒に表示(c36)
	Mat convertRGB2HSV(Mat& inputImage); //!<RGB画像imageをHSVに変換するメソッド
	Mat extractColor(Mat& inputHsvImage); //!<変換したHSV画像から特定の色を抽出するメソッド
	Mat getCoordinate(Mat& inputBinImage); //!<二値画像から座標を抽出するメソッド(c40)
	/*オープニング処理用にとっておく．(c31)*/
	//Mat extractBlack(Mat& opening_img); //!<オープニング処理後の画像から座標を抽出する(c24)
	Mat getGrayscaleImage(Mat& hsvCEImage); //!<グレースケールに変換(c19)
	Mat getBinImage(Mat& grayImage); //!<抽出した画像の二値化(c19)
	Mat OpeningImage(Mat& binImage); //!<抽出して二値化した画像に対してオープニング処理(同じ数だけ縮小→膨張)を行う(c19)
	
	void trackingObject(Mat& hsvImage); //!<対象を追跡する(c26)
	Mat mTrim_img; //!<追跡の際に切り取った周辺画像

	void drawCenterPoint(Mat& inputOriginalImage, Point3ius averageCoordinate/*, const string* mainWindowName*/); //!<計算した平均座標を画面上に表示する(c45)

	void loadInternalCameraParameter(char* cameraParamFile); //!<カメラキャリブレーションによって得られたパラメータを適用する(c54)
	Mat getUndistortionImage(Mat& inputOriginalImage); //!<キャリブレーションデータを用いてKinectから取得した画像を補正する(c71)
	Mat undistortionImage; //!<補正後の画像(c71)

	Mat getBackgroundSubstractionBinImage(Mat& current_image, Mat& backgound_gray_image); //!<背景差分によって得られた二値画像(c75)

	Mat getUnitMask(Mat& input_binimage);
};

/* インクルードガードの終了 */
#endif /* __IMAGEPROCESSING_HPP__ */
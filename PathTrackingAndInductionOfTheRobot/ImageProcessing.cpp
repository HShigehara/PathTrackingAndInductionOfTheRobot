/*
 * @file ImageProcessing.cpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief 画像処理を行うためのメソッド群
 * @date 2014.10.17
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "ImageProcessing.hpp"

/*!
 * @brief メソッドImageProcessing::ImageProcessing().コンストラクタ
 */
ImageProcessing::ImageProcessing()
{
	//コンストラクタは今のところなし
	th = 13; //閾値の初期値(c82)
	neighborhood = 2;
	closing_times = 3;
}

/*!
 * @brief メソッドImageProcessing::~ImageProcessing().デストラクタ
 */
ImageProcessing::~ImageProcessing()
{
	//デストラクタは今のところなし
}

/*!
* @brief メソッドImageProcessing::showImage().cv::Matを表示
* @param std::string windowName, cv::Mat& input_image
*/
void ImageProcessing::showImage(string windowName, Mat& input_image)
{
	namedWindow(windowName, CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	imshow(windowName, input_image);
	return;
}

/*!
* @brief メソッドImageProcessing::showTogetherImage().2つのcv::Matを1つのウインドウに表示
* @param cv::Mat& image1, cv::Mat& image2
*/
void ImageProcessing::showImageTogether(Mat& image1, Mat& image2)
{
	//ウインドウ名と合成画像を定義
	char* window_name = "処理結果";
	int win_width = image1.cols + image2.cols;
	int win_height = max(image1.rows, image2.rows); //ウインドウの高さは高い方に合わせる
	Mat all_img(Size(win_width, win_height), CV_8UC3);

	//画像を合成
	Mat roi1(all_img, Rect(0, 0, image1.cols, image1.rows));
	Mat roi2(all_img, Rect(image1.cols, 0, image2.cols, image2.rows));
	image1.copyTo(roi1);
	image2.copyTo(roi2);

	showImage(window_name, all_img); //合成した画像を表示

	return;
}

/*!
* @brief メソッドImageProcessing::showTogetherImage().3つのcv::Matを1つのウインドウに表示
* @param cv::Mat& image1, cv::Mat& image2, cv::Mat& image3
*/
void ImageProcessing::showImageTogether(Mat& image1, Mat& image2, Mat& image3)
{
	//ウインドウ名と合成画像を定義
	char* window_name = "処理結果";
	int win_width = image1.cols + image2.cols + image3.cols;
	int win_height = max({ image1.rows, image2.rows, image3.rows }); //ウインドウの高さは高い方に合わせる
	Mat all_img(Size(win_width, win_height), CV_8UC3);

	//画像を合成
	Mat roi1(all_img, Rect(0, 0, image1.cols, image1.rows));
	Mat roi2(all_img, Rect(image1.cols, 0, image2.cols, image2.rows));
	Mat roi3(all_img, Rect(image1.cols + image2.cols, 0, image3.cols, image3.rows));
	image1.copyTo(roi1);
	image2.copyTo(roi2);
	image3.copyTo(roi3);

	showImage(window_name, all_img); //合成した画像を表示

	return;
}

/*!
 * @brief ImageProcessing::loadInternalCameraParam().カメラキャリブレーションによって得られたカメラパラメータを適用するメソッド(c54)
 * @param const string cameraParamFile
 */
void ImageProcessing::loadInternalCameraParameter(const string cameraParamFile)
{
	//xmlファイルの読み込み
	FileStorage fs(cameraParamFile, FileStorage::READ); //読み込みモード
	//内部パラメータの読み込み
	fs["camera_matrix"] >> internal_cameraparam; //内部パラメータを読み込む
	fs["distortion_coefficients"] >> distortion_coefficients; //歪み係数を読み込む

	cout << "loaded " << cameraParamFile << ". " << endl;
	return;
}

/*!
 * @brief メソッドImageProcessing::getUndistortionImage()．キャリブレーションデータを用いてKinectから取得した画像を補正する
 * @param cv::Mat& inputOriginalImage
 * @return cv::Mat undistortionImage
 */
Mat ImageProcessing::getUndistortionImage(Mat& inputOriginalImage)
{
	Mat undistotionImage;

	undistort(inputOriginalImage, undistotionImage, internal_cameraparam, distortion_coefficients, Mat());
	
	return undistotionImage;
}

/*!
 * @brief メソッドImageProcessing::getBackgroundSubstractionBinImage()．背景差分によって得られた二値画像(c75)
 * @param cv::Mat& current_image, cv::Mat& background_image 
 * @return cv::Mat median_bin_image
 */
Mat ImageProcessing::getBackgroundSubstractionBinImage(Mat& current_image, Mat& background_gray_image/*, int threshold, int neighborhood, int closing_times*/)
{
	Mat current_gray_image; //!<現在のグレースケール画像(c75)
	Mat diff_gray_image; //!<背景差分画像(c74)
	Mat diff_bin_image; //!<背景差分画像の二値画像(c75)
	Mat median_bin_image; //!<背景差分画像の二値画像を平滑化したもの(c75)
	//Mat opening_image; //(仮)
	Mat closing_image;

	cvtColor(current_image, current_gray_image, CV_BGR2GRAY); //現フレームの画像をグレースケールに
	absdiff(current_gray_image, background_gray_image, diff_gray_image); //差分画像取得
	//showImage("差分画像", diff_gray_image);
	
	//EV3用
	//threshold(diff_gray_image, diff_bin_image, /*13*/20, 255, THRESH_BINARY); //二値化
	////showImage("二値画像", diff_bin_image);
	//medianBlur(diff_bin_image, median_bin_image, 7); //ノイズ除去
	////showImage("平滑化処理後", median_bin_image);
	////morphologyEx(median_bin_image, opening_image, MORPH_OPEN, Mat(), Point(-1, -1), 5); //オープニング(縮小→膨張)処理
	//morphologyEx(median_bin_image, closing_image, MORPH_CLOSE, Mat(), Point(-1, -1), 7); //クロージング(膨張→収縮)処理．穴埋めに使われる
	////showImage("穴埋め処理後", closing_image);


	threshold(diff_gray_image, diff_bin_image, th, 255, THRESH_BINARY); //二値化
	//showImage("二値画像", diff_bin_image);
	medianBlur(diff_bin_image, median_bin_image, 2*neighborhood+1); //ノイズ除去
	//showImage("平滑化処理後", median_bin_image);
	//morphologyEx(median_bin_image, opening_image, MORPH_OPEN, Mat(), Point(-1, -1), 5); //オープニング(縮小→膨張)処理
	morphologyEx(median_bin_image, closing_image, MORPH_CLOSE, Mat(), Point(-1, -1), 2*closing_times+1); //クロージング(膨張→収縮)処理．穴埋めに使われる
	//showImage("穴埋め処理後", closing_image);
	return closing_image;
	//return median_bin_image;
}

Mat ImageProcessing::getUnitMask(Mat& input_binimage)
{
	int x_min;
	int x_border1;
	int x_border2;
	int x_max;
	int y_min;
	int y_border;
	int y_max;

	bool ymin_check = false; //y_minが見つかっていないとき

	//xの最大値と最小値の計測
	x_min = input_binimage.cols;
	x_max = 0;
	//yの最大値と最小値の計測
	y_min = 0;
	y_max = 0;

	//xとyの最大値と最小値を探す
	for (int y = 0; y < input_binimage.rows;y++){
		for (int x = 0; x < input_binimage.cols;x++){
			if (input_binimage.at<unsigned char>(y, x) == 255) //白ピクセルなら特定の処理を行う
			{
				if (x_min > x){
					x_min = x;
				}
				if (x_max < x){
					x_max = x;
				}
				if (ymin_check == false){ //y_minがみつかっていなければ．一度しか実行されない
					y_min = y; //そのときのyを保存
					ymin_check = true; //フラグをtrueにする
				}
				if (y_max < y){ //現代の最大より新しいyが大きければ
					y_max = y; //新しいyを最大値とする
				}
			}
		}
	}
	cout << "x_min => " << x_min << ", x_max => " << x_max << ", y_min => " << y_min << ", y_max => " << y_max << endl;

	//x方向の切り取り
	x_border1 = (x_min + x_max) * /*0.18*/0.17;
	x_border2 = (x_min + x_max) * /*0.82*/0.83;
	//左部を白にする
	/*for (int y = 0; y < input_binimage.rows; y++){
		for (int x = x_min; x < x_border1; x++){
			if (input_binimage.at<unsigned char>(y, x) == 255){
				input_binimage.at<unsigned char>(y, x) = 0;
			}
		}
	}
	//右部を白にする
	for (int y = 0; y < input_binimage.rows; y++){
		for (int x = x_border2; x < x_max; x++){
			if (input_binimage.at<unsigned char>(y, x) == 255){
				input_binimage.at<unsigned char>(y, x) = 0;
			}
		}
	}*/
	//imwrite("before_cut.jpg", input_binimage);
	//上限と下限からカットするボーダーを決定する
	y_border = (y_max + y_min) * /*0.45*/0.5; //影の影響でy_maxが増えるため少し大きめに設定するのが良い 
	//下部を白にする
	int step;
	step = 15;
	for (int y = y_min; y < y_min + step; y++){
		for (int x = 0; x < input_binimage.cols; x++){
			if (input_binimage.at<unsigned char>(y, x) == 255){
				input_binimage.at<unsigned char>(y, x) = 0;
			}
		}
	}

	//下部の切り取り
	for (int y = y_border; y <= y_max; y++){
		for (int x = 0; x < input_binimage.cols; x++){
			if (input_binimage.at<unsigned char>(y, x) == 255){
				input_binimage.at<unsigned char>(y, x) = 0;
			}
		}
	}

	//showImage("TEST", input_binimage);
	return input_binimage;
}

/*
 * @file ImageProcessingMethod.cpp
 * @link https://github.com/HShigehara/Masters-Thesis.git
 * @brief 画像処理を行うためのメソッド群
 * @date 2014.10.17
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp" //ヘッダファイルのインクルード
#include "ImageProcessingMethod.hpp"

/*!
 * @brief メソッドImageProcessing::ImageProcessing().コンストラクタ
 * @param なし
 * @return なし
 */
ImageProcessing::ImageProcessing()
{
	//コンストラクタは今のところなし
}

/*!
 * @brief メソッドImageProcessing::~ImageProcessing().デストラクタ
 * @param なし
 * @return なし
 */
ImageProcessing::~ImageProcessing()
{
	//デストラクタは今のところなし
}

/*!
* @brief メソッドImageProcessing::showImage().cv::Matを表示
* @param windowName,input_image
* @return なし
*/
void ImageProcessing::showImage(char* windowName, Mat& input_image)
{
	namedWindow(windowName, CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	imshow(windowName, input_image);
	return;
}

/*!
* @brief メソッドImageProcessing::showTogetherImage().2つのcv::Matを1つのウインドウに表示
* @param image1,image2
* @return なし
*/
void ImageProcessing::showTogetherImage(Mat& image1, Mat& image2)
{
	//ウインドウ名と合成画像を定義
	char* windowName = "処理結果";
	int winWidth = image1.cols + image2.cols;
	int winHeight = max(image1.rows, image2.rows); //ウインドウの高さは高い方に合わせる
	Mat all_img(Size(winWidth, winHeight), CV_8UC3);

	//画像を合成
	Mat mRoi1(all_img, Rect(0, 0, image1.cols, image1.rows));
	Mat mRoi2(all_img, Rect(image1.cols, 0, image2.cols, image2.rows));
	image1.copyTo(mRoi1);
	image2.copyTo(mRoi2);

	showImage(windowName, all_img); //合成した画像を表示

	return;
}

/*!
* @brief メソッドImageProcessing::showTogetherImage().3つのcv::Matを1つのウインドウに表示
* @param image1,image2,image3
* @return なし
*/
void ImageProcessing::showTogetherImage(Mat& image1, Mat& image2, Mat& image3)
{
	//ウインドウ名と合成画像を定義
	char* windowName = "処理結果";
	int winWidth = image1.cols + image2.cols + image3.cols;
	int winHeight = max({ image1.rows, image2.rows, image3.rows }); //ウインドウの高さは高い方に合わせる
	Mat mAll_img(Size(winWidth, winHeight), CV_8UC3);

	//画像を合成
	Mat mRoi1(mAll_img, Rect(0, 0, image1.cols, image1.rows));
	Mat mRoi2(mAll_img, Rect(image1.cols, 0, image2.cols, image2.rows));
	Mat mRoi3(mAll_img, Rect(image1.cols + image2.cols, 0, image3.cols, image3.rows));
	image1.copyTo(mRoi1);
	image2.copyTo(mRoi2);
	image3.copyTo(mRoi3);

	showImage(windowName, mAll_img); //合成した画像を表示

	return;
}

/*!
 * @brief メソッドImageProcessing::convertRGB2HSV().RGB画像をHSVに変換する
 * @param input_image
 * @return hsvimg
 */
Mat ImageProcessing::convertRGB2HSV(Mat& input_image)
{
	Mat smoothimg; //!<フィルタ処理時に利用する変数(c17)
	Mat hsvimg; //!<HSV画像格納用変数(c17)
	medianBlur(input_image, smoothimg, 7); //メディアンフィルタで平滑化．(初期値7)
	cvtColor(smoothimg, hsvimg, CV_BGR2HSV); //HSVに変換
	return (hsvimg);
}

/*!
* @brief メソッドImageProcessing::colorExtraction().HSV画像から特定の色(青)を抽出して，白黒の画像を生成する
* @param input_hsvimg
* @return hsvceimg
*/
Mat ImageProcessing::extractColor(Mat& input_hsvimg)
{
	Mat hsvceimg; //!<色抽出後の画像

	hsvceimg = Mat(Size(input_hsvimg.cols, input_hsvimg.rows), CV_8UC3); //HSV抽出後用の定義.imageと同じ幅と高さにし,3チャンネルにする
	//hsv_ceimg = Scalar(0, 0, 0); //背景を黒で初期化
	hsvceimg = Scalar(255, 255, 255); //背景を白で初期化

	//(c3)
	for (int y = 1; y < input_hsvimg.rows; y++)
	{
		for (int x = 1; x < input_hsvimg.cols; x++){
			int px = input_hsvimg.step*y + (x * 3);
			//if (input_hsvimg.data[px] >= 110 && input_hsvimg.data[px] <= 123 && input_hsvimg.data[px + 1] >= 80 && input_hsvimg.data[px + 2] >= 40){ //結構いい感じ(青いボール)
			if (input_hsvimg.data[px] >= 110 && input_hsvimg.data[px] <= 120 && input_hsvimg.data[px + 1] >= 80 && input_hsvimg.data[px + 2] >= 45){ //結構いい感じ(青いボール)
				//抽出された色をわかりやすい色に変更する(オレンジは[px]=0;[px+1]=150;)
				hsvceimg.data[px] = 0;
				hsvceimg.data[px + 1] = 0;
				hsvceimg.data[px + 2] = 255;
			}
		}
	}
	//imshow("TEST", hsvceimg);
	return (hsvceimg);
}

/*!
* @brief メソッドImageProcessing::getCoordinate().座標を抽出するメソッド
* @param input_binimg
* @return extblackimage
*/
Mat ImageProcessing::getCoordinate(Mat& input_binimg)
{
	Mat extblackimg;
	//(c33)
	for (int i = 0; i < /*ALLPIXEL*/extractedNum; i++){
		extCoordinate[i].x = 0;
		extCoordinate[i].y = 0;
		extCoordinate[i].z = 0;
		extractedPointOneDim[i] = 0;
	}
	extractedNum = 0; //(c6)1フレームごとに抽出されたピクセルをカウントするためのextractedNumを初期化

	extblackimg = Mat(Size(input_binimg.cols, input_binimg.rows), CV_8UC1); //
	//extblackimg = Scalar(0, 0, 0); //背景を黒で初期化
	extblackimg = Scalar(255, 255, 255); //背景を白で初期化

	//(c3)
	//取得する座標データを間引く．for(◯,◯<△,◯++)をfor(◯,◯<△,◯+=増分にする)(c48)
	for (int y = 1; y < input_binimg.rows; y+=1)
	{
		for (int x = 1; x < input_binimg.cols; x+=1){
			int px = input_binimg.step * y + x;
			if (input_binimg.data[px] ==  0){ //結構いい感じ(青いボール)
				//抽出された色をわかりやすい色に変更する(オレンジは[px]=0;[px+1]=150;)
				extblackimg.data[px] = 0;

				//(c6)．抽出した画像に対して座標を取得する場合には利用する．(c24)ではオープニング処理の後に座標を取得したいのでコメントアウト
				extractedPointOneDim[extractedNum] = (trackWindow.x + x - 1) + WIDTH * (trackWindow.y + y - 1) - WIDTH; //抽出された(x,y)を1次元の値に変換
				extCoordinate[extractedNum].x = trackWindow.x + x - 1; //抽出されたx座標(c8)
				extCoordinate[extractedNum].y = trackWindow.y + y - 1; //抽出されたy座標(c8)
				//cout << extCoordinate[extractedNum].x << " , " << extCoordinate[extractedNum].y << endl;
				extractedNum++;
			}
		}
	}
	return (extblackimg);
}

/*!
* @brief メソッドImageProcessing::grayscaleImage().抽出した画像をグレースケールに変換する
* @param cv::Mat& hsv_ceimg
* @return grayimg
*/
Mat ImageProcessing::grayscaleImage(Mat& hsv_ceimg)
{
	Mat grayimg;
	cvtColor(hsv_ceimg, grayimg, CV_RGB2GRAY); //抽出した画像をグレースケールにする
	//showImage("GrayImage", grayimg); //確認用(c40)
	return (grayimg);
}

/*!
* @brief メソッドImageProcessing::binarizationImage().特定の色を抽出した画像に対して二値化処理を行う
* @param gray_img
* @return binimg
*/
Mat ImageProcessing::binarizationImage(Mat& gray_img)
{
	Mat binimg;
	cv::threshold(gray_img, binimg, 0, 255, THRESH_BINARY | THRESH_OTSU); //グレースケールの画像を二値化する
	//showImage("BinarizationImage", binimg); //確認用(c40)
	return (binimg);
}

/*!
* @brief メソッドImageProcessing::OpeningImage().特定の色を抽出して二値化した画像に対してオープニング処理を行う
* @param bin_img
* @return openingimg
*/
Mat ImageProcessing::OpeningImage(Mat& bin_img)
{
	Mat openingimg;
	morphologyEx(bin_img, openingimg, MORPH_OPEN, Mat(), Point(-1, -1), 2); //オープニング(縮小→膨張)処理
	//showImage("MorphologyEX", openingimg); //確認用(c40)
	return (openingimg);
}

/*!
* @brief メソッドImageProcessing::trackObject().対象を追跡するメソッド(c26)
* @param Mat& hsv_img
* @return なし
*/
void ImageProcessing::trackingObject(Mat& hsv_img)
{
	if (trackObject)
	{
		inRange(hsv_img, Scalar(0, smin, MIN(_vmin, _vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);

		//Hueの抽出
		int ch[] = { 0, 0 };
		hue.create(hsv_img.size(), hsv_img.depth());
		mixChannels(&hsv_img, 1, &hue, 1, ch, 1);

		//矩形領域の選択
		if (trackObject < 0)
		{
			//トラッキングウインドウを設定
			trackWindow = selection;
			trackObject = 1;

			//ヒストグラムを計算
			Mat roi(hue, selection), maskroi(mask, selection);
			calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
			normalize(hist, hist, 0, 255, CV_MINMAX);
		}

		//バックプロジェクション
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		
		//CamShiftアルゴリズム
		RotatedRect trackBox = CamShift(backproj, trackWindow, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));

		if (trackWindow.area() <= 1)
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,trackWindow.x + r, trackWindow.y + r) & Rect(0, 0, cols, rows);
		}

		if (backprojMode)
		{
			cvtColor(backproj, image, CV_GRAY2BGR);
		}

		//画像を切り取る際に，余裕をもって切り取るようにする(c33).ただし，この場合，(x,y)<(10,10)だと切り取る座標がマイナスになりOpenCVのエラーが起こるため，場合分けをする
		if (trackWindow.x - 10 > 0 && trackWindow.y - 10 > 0){ //(x,y) > (10,10)であれば，上下左右余裕をもって切り取ることができる
			Mat trim(image, Rect(trackWindow.x - 10, trackWindow.y - 10, trackWindow.width + 20, trackWindow.height + 20)); //画像を切り取る際，範囲を広げておく(c33)
			mTrim_img = trim.clone(); //余裕を持って切り取った画像を完全にコピーする(c33)
		}
		else{ //(x,y) < (10,10)であれば，切り取る際にエラーが起きるため，左と上はそのまま切り取る．右と下は余裕をもって切り取る．(c33)
			Mat trim(image, Rect(trackWindow.x, trackWindow.y, trackWindow.width + 20, trackWindow.height + 20)); //
			mTrim_img = trim.clone(); //切り取った画像を完全にコピーする(c33)
		}

		//追跡する円の表示
		ellipse(image, trackBox, Scalar(0, 0, 255), 2, CV_AA);
	}

	//選択領域を表示
	if (selectObject && selection.width > 0 && selection.height > 0)
	{
		Mat roi(image, selection);
		bitwise_not(roi, roi);
	}

	return;
}

/*!
* @brief メソッドImageProcessing::drawCenterPoint().中心座標を描画するメソッド(c26)
* @param なし
* @return なし
*/
void ImageProcessing::drawCenterPoint(Mat& image, Point3ius averageCoordinate/*, const string* mainWindowName*/)
{
	//Mat mAveragePoint; //平均座標を描画する変数(c45)

	//mAveragePoint = image.clone();

	//circle(mAveragePoint, Point2i(averageCoordinate.x, averageCoordinate.y), 1, Scalar(0, 0, 255), 2, CV_AA);
	circle(image, Point2i(averageCoordinate.x, averageCoordinate.y), 1, Scalar(0, 0, 255), 2, CV_AA);
	//imshow(mainWindowName, mAveragePoint);
	//imshow(mainWindowName, image);

	return;
}

/*!
 * @brief メソッドImageProcessing::backGroundSubstraction().背景差分処理により前景画像を取得する(c66)
 * @param Mat image
 * @return Mat foreGroundMask
 */
Mat ImageProcessing::backGroundSubstraction(/*Mat& image*/)
{
	BackgroundSubtractorMOG2 backGroundSubstractor;
	Mat foreGroundMask, output;

	backGroundSubstractor(image, foreGroundMask);

	bitwise_and(image, image, output, foreGroundMask);

	//imshow("TEST", foreGroundMask);
	return foreGroundMask;
}
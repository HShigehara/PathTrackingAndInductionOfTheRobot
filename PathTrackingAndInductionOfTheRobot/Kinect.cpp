/*
 * @file Kinect.cpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief Kinectを扱うためのメソッド群
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "Kinect.hpp"

/*!
 * @brief メソッドKinect::Kinect().コンストラクタ
 */
Kinect::Kinect()
{
	countKinect = 0; //Kinectの数を初期化
}

/*!
 * @brief メソッドKinect::~Kinect().デストラクタ
 */
Kinect::~Kinect()
{
	//終了処理
	if (kinect != 0){
		kinect->NuiShutdown();
		kinect->Release();
	}
}

/*!
 * @brief メソッドKinect::createInstance().インスタンスの生成
 */
void Kinect::createInstance()
{
	//接続されているKinectの数を取得する
	ERROR_CHECK(::NuiGetSensorCount(&countKinect));
	if (countKinect == 0){
		throw runtime_error("Please Connect the Kinect.");
	}

	ERROR_CHECK(::NuiCreateSensorByIndex(0, &kinect)); //最初のKinectのインスタンスを生成する

	//Kinectの状態を取得する
	HRESULT status = kinect->NuiStatus();
	if (status != S_OK){
		throw runtime_error("You Cannot Use the Kinect.");

	}

	return;
}

/*!
 * @brief メソッドKinect::initialize().Kinectの初期化
 */
void Kinect::initialize()
{
	createInstance(); //createInstance()の処理へ以降

	ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH)); //Kinectの設定を初期化
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION, 0, 2, 0, &imageStreamHandle)); //RGBカメラを初期化
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle)); //Depthカメラを初期化
	ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE)); //Nearモード

	//フレーム更新のイベントハンドルを作成
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

	::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height); //指定した解像度の画面サイズを取得する

	return;
}

/*!
 * @brief メソッドKinect::drawRGBImage(Mat& image).RGBカメラの処理
 * @param cv::Mat& image
 * @return cv::Mat image
 */
Mat Kinect::drawRGBImage(Mat& image)
{
	try{
		//RGBカメラのフレームデータを取得する
		NUI_IMAGE_FRAME imageFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(imageStreamHandle, 0, &imageFrame));

		//画像データの取得
		NUI_LOCKED_RECT colorData;
		imageFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);

		//画像データのコピー
		image = Mat(height, width, CV_8UC4, colorData.pBits);

		//フレームデータの解放
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame));
	}
	catch (exception& ex){ //例外処理(c57)
		cout << ex.what() << endl;
	}
	return (image); //RGBカメラから画像を取得し返す(c30)
}

/*
 * @brief メソッドKinect::setDepthImage(Mat& Mat_image).デプス画像を取得するメソッド(c57)
 * @param cv::Mat& Mat_image
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr points
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getPointCloud(Mat& Mat_image)
{
	try{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>()); //ポイントクラウド保存用(c57)
		points->width = width;
		points->height = height;

		//距離カメラのフレームデータを取得
		NUI_IMAGE_FRAME depthFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame));

		//距離データを取得する
		NUI_LOCKED_RECT depthData = { 0 };
		depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

		USHORT* depth = (USHORT*)depthData.pBits;
		for (int i = 0; i < (depthData.size / sizeof(USHORT)); ++i){

			USHORT distance = ::NuiDepthPixelToDepth(depth[i]);

			//USHORT player = ::NuiDepthPixelToPlayerIndex(depth[i]);
			LONG depthX = i % width;
			LONG depthY = i / width;
			LONG colorX = depthX;
			LONG colorY = depthY;

			// 距離カメラの座標を、RGBカメラの座標に変換する
			kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, depthX, depthY, 0/*depth[i]*/, &colorX, &colorY);

			//点群取得処理．渡された差分画像に応じて条件を入れ替える
			//Vector4 real = NuiTransformDepthImageToSkeleton(depthX, depthY, distance, CAMERA_RESOLUTION);
			Vector4 real = NuiTransformDepthImageToSkeleton(depthX, depthY, distance << 3, CAMERA_RESOLUTION); //左に3ビットすることでプレーヤー情報を含む深度データを渡し，座標を変換する
			if (Mat_image.at<UCHAR>(colorY, colorX) == 255){ //二値画像に対して点群を抽出する際はこっち(白色の点群を抽出)(c70)
				pcl::PointXYZRGB point; //点群用の変数を確保
				point.x = real.x*1000.0f; //ポイントクラウドのx座標を格納[mm]
				point.y = real.y*1000.0f; //ポイントクラウドのy座標を格納[mm]
				point.z = real.z*1000.0f; //ポイントクラウドのz座標を格納[mm]

				//cout << point << endl;
				//テクスチャ(その座標の色を格納していく)
				Vec4b color = image.at<Vec4b>(colorY, colorX); //色格納用の変数
				point.r = color[2]; //赤要素
				point.g = color[1]; //緑要素
				point.b = color[0]; //青要素
				points->push_back(point); //点群を出力
			}
		}
		cloud = points; //点群をコピー
		//フレームデータを開放する(c58)
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));
	}
	catch (exception& ex){
		cout << ex.what() << endl;
	}
	return cloud;
}
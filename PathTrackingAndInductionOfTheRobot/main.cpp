﻿/*
 * @file main.cpp
 * @link https://github.com/HShigehara/Masters-Thesis.git
 * @brief main関数
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "KinectMethod.hpp"
#include "ImageProcessingMethod.hpp"
#include "RouteDrawingMethod.hpp"
#include "System.hpp"
#include "LeastSquareMethod.hpp" //(c49)
#include "PointCloudMethod.hpp" //ポイントクラウド処理のヘッダを追加(c57)

/* グローバル変数 */
//画像データ
Mat image; //!<RGB画像格納用の変数
//Mat depth_image; //!<距離画像格納用の変数(c58)

char directoryName[NOC]; //!<フォルダ名

//(c6)
int extractedPointOneDim[ALLPIXEL]; //!<抽出された座標の1次元の値
int extractedNum; //(c6)colorExtraction()で何ピクセル抽出されたかカウントする変数

Point3ius extCoordinate[ALLPIXEL]; //抽出された座標(c37)

Vector4 XYZCoordinate[ALLPIXEL]; //3次元に変換された(X,Y,Z)(c49)

//(c25)
Rect trackWindow; //!<追跡ウインドウ
int hsize = 16;
float hranges[] = { 0, 180 }; //!<Hの範囲
const float* phranges = hranges;
bool backprojMode = false; //!<バックプロジェクトモード
bool selectObject = false; //!<オブジェクト選択
int trackObject = 0; //!<追跡するオブジェクト
Point origin; //!<オリジナルの座標
Rect selection; //!<選択
int vmin = 10, vmax = 256, smin = 30; //!<HSVの範囲指定
void onMouse(int event, int x, int y, int flags, void* param); //!<マウス操作

//(c26)
/*int*/bool avgFlag; //!<平均を計算したとき用のフラグ(c30)
/*int*/bool mouseFlag; //!<マウス操作確認用のフラグ(c26)

/*!
 * @brief 関数main()
 * @param なし
 * @return なし
 */
int main()
{
	RETRY: //goto文．再計測をやり直す場合
	//インスタンスの生成
	System sys; //!<システム的なメソッドをまとめているクラス
	RouteDrawing routedraw; //!<RouteDrawingクラスのインスタンスを生成
	LeastSquareMethod lsm; //!<最小二乗法を行うクラスのインスタンスを生成(c49)

	//変数の宣言
	//int checkNum; //!<プログラム終了時にデータを保存するか確認するための変数(c38)
	//outputData outputData[OUTPUTDATA_MAX]; //!<出力するデータを宣言．最大10000個(c41)

	//ファイル名の定義(c39)
	//const string outputDataName = "3d.dat"; //計測データが出力されるファイル名(c39)
	//const string centerofgravityDataName = "cog.dat"; //重心座標が出力されるファイル名(c52)

	//画像関係の変数
	//Mat mHSVforObjectTracking_img; //!<ヒストグラム作成のためのHSV変換後のデータ保存用
	//Mat mHSVforTrim_img; //!<切り取り後の画像格納用(c31)
	//Mat mHSVColorExtraction_img; //!<(ce:color extraction).HSVに変換する用の変数

	//オープニング処理用にとっておく
	//Mat mGray_img; //!<グレースケール用の変数(c19)
	//Mat mBin_img; //!<抽出した後に二値化した画像を保存する変数(c21)
	//Mat mOpening_img; //!<オープニングを行った画像から距離を抽出する(c24).オープニング処理を行うには必要*/
	//Mat mExtractedBlack_img; //!<オープニング後の二値画像から抽出された黒い座標を格納している変数(c40)

	Mat flip_image; //確認用に反転した画像
	Mat current_image; //現在のフレームの画像(c75)
	Mat bin_image; //背景差分によって得られた二値画像(c75)
	Mat background_image; //!<背景画像(c75)
	Mat background_gray_image;

	//ポイントクラウド関係の変数(c57)
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //ポイントクラウド保存用(c57)

	//メインの処理
	try{
		sys.startMessage(); //プログラム開始時のメッセージを表示

		Kinect kinect; //Kinectクラスのインスタンスを生成
		ImageProcessing imgproc; //Imageprocessingクラスのインスタンスを生成
		PointCloudMethod pcm(/*false*/true, /*false*/true, /*false*/true, false, /*false*/true,false); //PointCloudMethodクラスのインスタンスを生成(c57)


		//.plyファイルの読み込み
		//pcm.loadPLY("20151208_EV3COLOR.ply");

		//動画保存用
		//VideoWriter writer; //動画保存用 

		//座標関係の変数
		//Vector4 realPoint; //!<変換後の世界座標系の値を格納
		//Point3ius averageCoordinate; //!<平均座標を格納するクラス内のローカル変数(c38)

		//カウント用変数
		int countDataNum; //出力されたデータ数をカウントする(c39)

		//ウインドウ名とファイル名の定義
		const string mainWindowName = "動画像"; //メインウインドウの名前をつけておく．(c31)
		//const string outputVideoName = "video.avi"; //計測中の動画ファイル名(c39)

		//xmlファイル名の定義
		char* cameraParameterName = "cameraParam.xml"; //カメラキャリブレーションによって得られたファイル名(c54)

		//変数の初期化
		countDataNum = 0;
		//sumTime = 0.0; //合計の時間をカウントする変数
		//realPoint.x = 0.0; //3次元座標のx座標
		//realPoint.y = 0.0; //3次元座標のy座標
		//realPoint.z = 0.0; //3次元座標のx座標
		avgFlag = /*0*/false; //再計測のために平均座標を計算したかチェックするフラグ変数を初期化
		mouseFlag = /*0*/false; //再計測のためにマウスをクリックしたかをチェックするフラグ変数を初期化

		//sys.makeDirectory(); //起動時刻をフォルダ名にしてフォルダを作成

		kinect.initialize(); //Kinectの初期化

		//動画を保存(c40)
		//writer = sys.outputVideo(&outputVideoName); //動画を保存したいときはコメントをはずす．while文内のwriter << imageのコメントも

		//背景用に一度撮影(c75)
		sys.countdownTimer(/*3*/2000);
		system("cls");
		//データの更新を待つ
		DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //フレーム更新をイベントとして待つ
		::ResetEvent(kinect.streamEvent); //イベントが発生したら次のイベントに備えてリセット
		//Kinect処理・画像処理
		background_image = kinect.drawRGBImage(image); //RGBカメラの処理
		imgproc.showImage("Background Image", background_image);
		PlaySound(TEXT("sound/shutter_nikon.wav"), NULL, (SND_ASYNC | SND_FILENAME));
		cvtColor(background_image, background_gray_image, CV_BGR2GRAY);

		cout << "Process will Start in " << endl; //プログラム本編開始の通知
		sys.countdownTimer(/*5*/3000); //5000msカウントダウンする
		system("cls"); //コンソール内の表示をリセット(c64)
		pcm.initializePointCloudViewer("Point Cloud"); //クラウドビューワーの初期化

		while (!pcm.viewer->wasStopped() && kinect.key != 'q' && !GetAsyncKeyState('Q')){ //(c3).メインループ．1フレームごとの処理を繰り返し行う．(c63)CloudViewerが終了処理('q'キーを入力)したらプログラムが終了する
			//タイマー開始(c65)
			sys.startTimer();

			//setMouseCallback(mainWindowName, onMouse, 0); //(c25)．マウスコールバック関数をセット(c31)

			//データの更新を待つ
			DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //フレーム更新をイベントとして待つ
			::ResetEvent(kinect.streamEvent); //イベントが発生したら次のイベントに備えてリセット

			//Kinect処理・画像処理
			current_image = kinect.drawRGBImage(image); //RGBカメラの処理
			//Kinectのキャリブレーションを行い，キャリブレーション結果を適用する(c71)
			//imgproc.loadinternal_cameraparameter(cameraParameterName);
			//imgproc.undistortionImage = imgproc.getUndistortionImage(imgproc.current_image);

			flip(current_image, flip_image, 1);
			imgproc.showImage("Original - Flip", flip_image); //Kinectから取得した画像を表示

			bin_image = imgproc.getBackgroundSubstractionBinImage(current_image, background_gray_image);
			imgproc.showImage("Mask Image", bin_image);

			//ポイントクラウドの取得(c57)
			//pcm.cloud = kinect.getPointCloud(imgproc.current_image); //ポイントクラウドの取得(c57)．前景画像を2値化した画像を引数として与える(c67)
			pcm.cloud = kinect.getPointCloud(bin_image); //ポイントクラウドの取得(c57)．切り取った画像をもとにする
			//pcm.cloud = kinect.getPointCloud(/*depth_image*/imgproc.foreGroundMaskImage/*binimage*//*imgproc.diffBinImage*/); //ポイントクラウドの取得(c57)．前景画像を2値化した画像を引数として与える(c67)
			//pcm.cloud = kinect.getPointCloud(imgproc.foreGroundMaskBinImage); //ポイントクラウドの取得(c57)．前景画像を2値化した画像を引数として与える(c67)
			pcm.flagChecker(); //各点群処理のフラグをチェックするメソッド(c64)
			cout << "==========================================================================================" << endl;
			cout << "Original PointCloud Size\t=>\t" << pcm.cloud->size() << endl;

			//PCLの処理
			if (pcm.FlagPassThrough == true){  //外れ値除去(c59)
				pcm.cloud = pcm.passThroughFilter(pcm.cloud); //Kinectから取得した初期の外れ値を除去(c60)
				//cloud = pcm.radiusOutlierRemoval(cloud); //半径を指定して外れ値を除去(c60)
			}

			if (pcm.FlagDownsampling == true){	//ダウンサンプリング処理(c59)
				pcm.cloud = pcm.downSamplingUsingVoxelGridFilter(pcm.cloud, 0.0002f, 0.0002f, 0.0002f); //Default=all 0.003
				//pcm.cloud = pcm.downSamplingUsingVoxelGridFilter(pcm.cloud, 0.003, 0.003, 0.003); //Default=all 0.003
				//pcm.cloud = pcm.downSamplingUsingVoxelGridFilter(pcm.cloud, 0.001, 0.001, 0.001); //Default=all 0.003
			}

			if (pcm.FlagStatisticalOutlierRemoval == true){
				pcm.cloud = pcm.removeOutlier(pcm.cloud); //統計的な外れ値除去(c60)
			}

			if (pcm.FlagDownsampling == true && pcm.FlagMLS == true){  //スムージング処理(c60)
				pcm.cloud = pcm.smoothingUsingMovingLeastSquare(pcm.cloud, true, true, 0.001); //0.002 < radius < ◯．小さいほど除去される

			}
			else if (pcm.FlagDownsampling == false && pcm.FlagMLS == true){
				cout << "MLSを有効にするためには，ダウンサンプリングを有効にしてください" << endl;
			}

			if (pcm.FlagExtractPlane == true){	//平面検出とクラスタリング(c61)
				pcm.cloud = pcm.getExtractPlaneAndClustering(pcm.cloud, true, 1, 0.00008/*0.000003*//*0.0001*//*0.0009*//*0.0005*//*0.003*/, false, true, /*0.035*//*0.003タイヤの下が省かれる*/0.005/*0.05*/, /*350*/150, /*25000*//*20000*/1500); //Default=0.03(前処理なしの場合)
			}

			/*if (pcm.FlagIcpRegistration == true){
				pcm.cloud = pcm.cloudRegistration(pcm.cloud, pcm.model);
			}*/
			cout << "==========================================================================================" << endl;
			pcm.viewer->showCloud(pcm.cloud); //処理後の点群を表示

			//メインの処理(c26)(c30)
			//if (mouseFlag == true){ //mouseFlagがtrueであれば=マウスのボタンが上に上がったら
			////writer << image; //動画を保存(c35)
			//mHSVforObjectTracking_img = imgproc.convertRGB2HSV(image); //CamShiftで利用するヒストグラムを作成するためにHSVへ変換(c29)
			//imgproc.trackingObject(mHSVforObjectTracking_img); //画像を追跡するメソッドへ移行(c26)
			//mHSVforTrim_img = imgproc.convertRGB2HSV(imgproc.mTrim_img); //余裕を持って切り取られた画像をHSVに変換する(c33)
			//mHSVColorExtraction_img = imgproc.extractColor(mHSVforTrim_img); //余裕を持って切り取った画像から特定の色を抽出する(c33)
			//オープニングを行うなら必要な処理(c30)
			//グレースケール変換(c19)
			//mGray_img = imgproc.grayscaleImage(mHSVColorExtraction_img);
			//抽出したピクセルからさらに精度を高めるために膨張・収縮を行う(c19)
			//mBin_img = imgproc.binarizationImage(mGray_img); //抽出した画像の二値化
			//mOpening_img = imgproc.OpeningImage(mBin_img); //オープニング処理(c19)
			//mExtractedBlack_img = imgproc.getCoordinate(mOpening_img);

			//確認用
			//imgproc.showImage("計測対象の点", mExtractedBlack_img);

			//最小二乗法を行う(仮)(c49)
			////lsm.getSphereData(kinect.actualExtractedNum); //最小二乗法の処理へ移行(c49)

			//averageCoordinate = kinect.getAverageCoordinate(image); //Depthカメラの処理(c5)
			////imgproc.drawCenterPoint(image, averageCoordinate/*, &mainWindowName*/); //平均座標を表示するメソッド(c45)

			/*if (avgFlag == true){ //平均座標が抽出されていたら
				realPoint = kinect.getLocalPosition(averageCoordinate); //1フレームあたりの平均座標の3次元座標を取得する(c10)(c28)
				end = getTickCount();
				time = (end - start) * f;
				sumTime += time;
				fps = 1000.0 / time; //フレームレートを計算

				putText(image, to_string((int)fps) + "fps", Point(WIDTH - 100, HEIGHT - 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, CV_AA); //動画上にフレームレートを出力

				outputData[countDataNum].totalTime = sumTime; //計測されたトータルの時間(c41)
				outputData[countDataNum].x = realPoint.x; //Kinect座標系に変換されたx(c41)
				outputData[countDataNum].y = realPoint.y; //Kinect座標系に変換されたy(c41)
				outputData[countDataNum].z = realPoint.z; //Kinect座標系に変換されたz(c41)

				//抽出されたデータを表示
				//cout << "X => " << realPoint.x << "\tY => " << realPoint.y << "\tZ => " << realPoint.z << endl;
				//routedraw.plot3DRealTime(countDataNum, outputData); //位置をリアルタイムでプロットする(c43)
				countDataNum++; //出力されたデータ数をカウントする(c41)
				}
				else{ //座標が抽出されていないとき．オープニング処理を行っているときは白か黒の二値画像なのでここにくることはない(c41)
				end = getTickCount(); //タイマー終了(c41)
				}*/
			//imgproc.showTogetherImage(mHSVforTrim_img, mHSVColorExtraction_img); //2つの画像を1つのウインドウに表示．確認用(c36)
			//}
			//else{ //マウスがクリックされていないときのタイマー終了(c41)
			//end = getTickCount(); //タイマー終了
			//}

			//PCLのフレームレートを計算する用(c61)
			sys.endTimer(); //タイマーを終了(c65)
			cout << sys.getProcessTimeinMiliseconds() << "[ms], " << sys.getFrameRate() << " fps" << "\n" << endl;


			//終了のためのキー入力チェック兼表示のためのウェイトタイム
			kinect.key = waitKey(1);
			if (GetAsyncKeyState('R')){
				system("cls");
				destroyAllWindows(); //PCLまたは，OpenCV画面上で'q'キーが入力されたらOpenCVのウインドウを閉じて処理を終了(c66)
				pcm.viewer->~CloudViewer(); //クラウドビューアーの削除
				cout << "RETRY" << endl;
				goto RETRY;
			}
			/*if (kinect.key == 'q'){ //計測終了
				//sys.outputAllData(&outputDataName, outputData, countDataNum);
				//routedraw.plot3D(outputDataName); //(c4)
				destroyAllWindows();
				break;
				}*/
			//else if (kinect.key == 'r'){ //再計測するときに前のファイルを削除しておく(c31)
			//destroyAllWindows(); //OpenCVで作成したウインドウを全て削除する(c35)
			//sys.removeDirectory();
			//cout << "Data Removed." << endl;
			//goto RETRY;
			//}

			system("cls"); //コンソール内の表示をリセット(c64)
		}
		destroyAllWindows(); //PCLまたは，OpenCV画面上で'q'キーが入力されたらOpenCVのウインドウを閉じて処理を終了(c66)
	}

	catch (exception& ex){ //例外処理
		cout << ex.what() << endl;
		destroyAllWindows(); //OpenCVで作成したウインドウを全て削除する(c35)
		//異常終了した時はデータを保存する必要がないので削除
		//sys.removeDirectory();
		//cout << "Data Removed." << endl;
		//return -1;
	}

	//データを保存するかの確認(c27)
	//cout << "Save Data?" << endl;
	//checkNum = sys.alternatives(); //'1'なら保存，'0'なら削除

	//if (checkNum == 0){ //削除するとき(c55)
	//sys.removeDirectory(/*checkNum*/); //ディレクトリを削除するかどうか
	//}
	//else{ //保存するとき
	//routedraw.gnuplotScript(/*checkNum, */&outputDataName); //後で3D座標をプロットする用のgnuplotスクリプトを作るかどうか
	//routedraw.gnuplotScriptCoG(/*checkNum, */&centerofgravityDataName); //後で重心座標をプロットする用のgnuplotスクリプトを作るかどうか(c52)
	//}

	//sys.endMessage(checkNum);
	sys.endMessage();

	return 0;
}
/*
 * @file main.cpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief main関数
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "Kinect.hpp"
#include "ImageProcessing.hpp"
#include "Drawing.hpp"
#include "System.hpp"
#include "LeastSquareMethod.hpp" //(c49)
#include "PointCloudLibrary.hpp" //ポイントクラウド処理のヘッダを追加(c57)
#include "EV3Control.hpp" //EV3を制御するようのクラスを作成(c80)

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
bool avgFlag; //!<平均を計算したとき用のフラグ(c30)
bool mouseFlag; //!<マウス操作確認用のフラグ(c26)

/*!
 * @brief 関数main()
 * @param なし
 * @return なし
 */
int main()
{
	RETRY: //goto文．再計測をやり直す場合
	//インスタンスの生成
	Kinect kinect; //Kinectクラスのインスタンスを生成
	System sys; //!<システム的なメソッドをまとめているクラス
	ImageProcessing imgproc; //Imageprocessingクラスのインスタンスを生成
	Drawing draw; //!<drawingクラスのインスタンスを生成
	LeastSquareMethod lsm; //!<最小二乗法を行うクラスのインスタンスを生成(c49)
	PointCloudLibrary pointcloudlibrary(/*false*/true, /*false*/true, /*false*/true, false, false/*true*/); //PointCloudLibraryクラスのインスタンスを生成(c57)
	EV3Control ev3control; //!<EV3を制御する用のクラスを作成(c80)

	//変数の宣言
	//ファイル名の定義(c39)

	//画像関係の変数
	Mat flip_image; //確認用に反転した画像
	Mat current_image; //現在のフレームの画像(c75)
	Mat bin_image; //背景差分によって得られた二値画像(c75)
	Mat background_image; //!<背景画像(c75)
	Mat background_gray_image;

	//ポイントクラウド関係の変数(c57)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //処理を受け取る点群

	//EV3ユニットの平面の係数(c78)
	Eigen::Vector3f coefficient_plane; //平面の係数
	AttitudeAngle3d attitude_angle; //姿勢角(c78)

	//メインの処理
	try{
		sys.startMessage(); //プログラム開始時のメッセージを表示
		//初期設定(利用する場合はコメントを外す)
		//pointcloudlibrary.loadPLY("EV3COLOR.ply"); //.plyファイルの読み込み．動作しない
		//VideoWriter writer; //動画保存用 

		//ウインドウ名とファイル名の定義
		const string main_windowname = "Current Image"; //メインウインドウの名前をつけておく．(c31)
		const string backgroundimage_windowname = "Background Image"; //背景画像を表示するためのウインドウ名
		const string maskbinimage_windowname = "Mask Image"; //マスク画像を表示するためのウインドウ名
		//const string outputVideoName = "video.avi"; //計測中の動画ファイル名(c39)
		const string cameraparameter_name = "sourcedata/cameraParam.xml"; //xmlファイル名の定義．カメラキャリブレーションによって得られたファイル名(c54)
		const char* basedirectory_name = "data"; //データ保存先のディレクトリ名
		const string cloudviewer_windowname = "Cloud Viewer"; //クラウドビューアーの名前の定義(c81)
		//変数の初期化
		avgFlag = false; //再計測のために平均座標を計算したかチェックするフラグ変数を初期化
		mouseFlag = false; //再計測のためにマウスをクリックしたかをチェックするフラグ変数を初期化

		kinect.initialize(); //Kinectの初期化
		sys.checkDirectory(basedirectory_name); //base_directoryが存在するか確認し，存在しなければ作成(c81)
		sys.makeDirectory(); //起動時刻をフォルダ名にしてフォルダを作成
		
		//動画を保存するために利用する(c40)
		//writer = sys.outputVideo(&outputVideoName); //動画を保存したいときはコメントをはずす．while文内のwriter << imageのコメントも

		imgproc.loadInternalCameraParameter(cameraparameter_name); //キャリブレーションを行うためのパラメータを取得(c79)

		//背景用に一度撮影(c75)
		cout << "Take a Background Image in " << endl;
		sys.countdownTimer(3);
		system("cls");
		DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //フレーム更新をイベントとして待つ
		::ResetEvent(kinect.streamEvent); //イベントが発生したら次のイベントに備えてリセット
		//Kinectから画像を取得し，背景画像とする
		PlaySound(TEXT("sourcedata/shutter.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
		background_image = kinect.drawRGBImage(image); //RGBカメラの処理
		background_image = imgproc.getUndistortionImage(background_image); //キャリブレーション後の画像を今の背景画像とする
		imgproc.showImage(backgroundimage_windowname, background_image); //背景画像を表示する
		cvtColor(background_image, background_gray_image, CV_BGR2GRAY); //グレースケールに変換
		
		Sleep(1000);

		//プログラム開始の通知
		cout << "Process will Start in " << endl;
		sys.countdownTimer(5); 
		PlaySound(TEXT("sourcedata/shutter.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
		system("cls"); //コンソール内の表示をリセット(c64)

		pointcloudlibrary.initializePointCloudViewer(cloudviewer_windowname); //クラウドビューアーの初期化

		while (!pointcloudlibrary.viewer->wasStopped() && kinect.key != 'q'/* && !GetAsyncKeyState('Q')*/){ //(c3).メインループ．1フレームごとの処理を繰り返し行う．(c63)CloudViewerが終了処理('q'キーを入力)したらプログラムが終了する
			//タイマー開始(c65)
			sys.startTimer();

			//setMouseCallback(mainwindow_name, onMouse, 0); //(c25)．マウスコールバック関数をセット(c31)

			DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //フレーム更新をイベントとして待つ
			::ResetEvent(kinect.streamEvent); //イベントが発生したら次のイベントに備えてリセット

			//Kinectから画像を取得し，画像処理を行う
			current_image = kinect.drawRGBImage(image); //RGBカメラの処理
			current_image = imgproc.getUndistortionImage(current_image); //Kinectのキャリブレーションを行い，キャリブレーション結果を適用する(c71)
			imgproc.showImage(main_windowname, current_image);
			//flip(current_image, flip_image, 1);
			//imgproc.showImage("Original - Flip", flip_image); //Kinectから取得した画像を表示
			
			//タイヤも含めて前面の点群を取得する
			bin_image = imgproc.getBackgroundSubstractionBinImage(current_image, background_gray_image);
			//ユニット部だけ切り取る(c77)
			//bin_image = imgproc.getUnitMask(bin_image);
			imgproc.showImage(maskbinimage_windowname, bin_image); //確認用に切り取った画像を表示する

			//ポイントクラウドの取得(c57)
			cloud = kinect.getPointCloud(bin_image); //ポイントクラウドの取得(c57)．切り取った画像をもとにする
			pointcloudlibrary.flagChecker(); //各点群処理のフラグをチェックするメソッド(c64)
			cout << "==========================================================================================" << endl;
			cout << "Original PointCloud Size\t=>\t" << cloud->size() << endl;

			//PCLの処理
			if (pointcloudlibrary.passthrough_flag == true){  //外れ値除去(c59)
				cloud = pointcloudlibrary.passThroughFilter(cloud, "z", 400, 20000); //Kinectから取得した外れ値を除去(c60)．与えた軸の中で自分が取得したい範囲の下限と上限を与えることでそれ以外を省く(c81)
				//cloud = pointcloudlibrary.radiusOutlierRemoval(cloud); //半径を指定して外れ値を除去(c60)
			}

			if (pointcloudlibrary.downsampling_flag == true){	//ダウンサンプリング処理(c59)
				//cloud = pointcloudlibrary.downSamplingUsingVoxelGridFilter(pointcloudlibrary.cloud, 2.0f, 2.0f, 2.0f); //Default=all 0.003
				cloud = pointcloudlibrary.downSamplingUsingVoxelGridFilter(cloud, 2.5f, 2.5f, 2.5f); //Default=all 0.003
				//pointcloudlibrary.cloud = pointcloudlibrary.downSamplingUsingVoxelGridFilter(pointcloudlibrary.cloud, 0.001, 0.001, 0.001); //Default=all 0.003
			}

			if (pointcloudlibrary.statisticaloutlierremoval_flag == true){
				cloud = pointcloudlibrary.removeOutlier(cloud); //統計的な外れ値除去(c60)
			}

			if (pointcloudlibrary.downsampling_flag == true && pointcloudlibrary.mls_flag == true){  //スムージング処理(c60)
				cloud = pointcloudlibrary.smoothingUsingMovingLeastSquare(cloud, true, true, 2.5); //0.002 < radius < ◯．小さいほど除去される

			}
			else if (pointcloudlibrary.downsampling_flag == false && pointcloudlibrary.mls_flag == true){
				cout << "MLSを有効にするためには，ダウンサンプリングを有効にしてください" << endl;
			}

			if (pointcloudlibrary.extractplane_flag == true){	//平面検出とクラスタリング(c61)
				cloud = pointcloudlibrary.getExtractPlaneAndClustering(cloud, true, 100, 5, false, true, 0.02, /*350*/150, /*25000*//*20000*/200000); //Default=0.03(前処理なしの場合)
			}

			pointcloudlibrary.centroid = pointcloudlibrary.getCentroidCoordinate3d(cloud); //重心座標の計算
			coefficient_plane = lsm.getCoefficient(cloud); //最小二乗法を行い平面の係数[a b c]'を取得する(c78)
			attitude_angle = lsm.calcYawRollPitch(coefficient_plane); //姿勢角を取得(c78)
			//cout << "[Yaw, Roll, Pitch]" << attitude_angle.yaw << " , " << attitude_angle.roll << " , " << attitude_angle.pitch << endl;
			
			ev3control.set6DoFEV3(cloud, pointcloudlibrary.centroid, attitude_angle); //6DoFをまとめる
			cout << "==========================================================================================" << endl;
			pointcloudlibrary.viewer->showCloud(cloud); //処理後の点群を表示

			//PCLのフレームレートを計算する用(c61)
			sys.endTimer(); //タイマーを終了(c65)
			cout << sys.getProcessTimeinMiliseconds() << "[ms], " << sys.getFrameRate() << " fps" << "\n" << endl;



			//終了のためのキー入力チェック兼表示のためのウェイトタイム
			kinect.key = waitKey(1);
			if (GetAsyncKeyState('R')){
				system("cls");
				destroyAllWindows(); //PCLまたは，OpenCV画面上で'q'キーが入力されたらOpenCVのウインドウを閉じて処理を終了(c66)
				pointcloudlibrary.viewer->~CloudViewer(); //クラウドビューアーの削除
				sys.removeDirectory(); //再計測を行う場合は，現在のデータは必要ないため削除
				cout << "Data Removed." << endl;
				system("cls"); //cmdをクリア
				cout << "RETRY" << endl;
				goto RETRY;
			}
			system("cls"); //コンソール内の表示をリセット(c64)
		}
		//計測が終了したところ
		destroyAllWindows(); //PCLまたは，OpenCV画面上で'q'キーが入力されたらOpenCVのウインドウを閉じて処理を終了(c66)
		pointcloudlibrary.viewer->~CloudViewer(); //クラウドビューアーの削除
		draw.gnuplotScriptEV3Unit(coefficient_plane); //gnuplot用のスクリプト
		//その時の画像を保存
		char filepath_currentimage[NOC];
		sprintf_s(filepath_currentimage, "data/%s/current_image.jpg", directoryName);
		imwrite(filepath_currentimage, current_image);
		char filepath_binimage[NOC];
		sprintf_s(filepath_binimage, "data/%s/background_image.jpg", directoryName);
		imwrite(filepath_binimage, bin_image);

		//データを保存するかの確認
		cout << "Save Data?" << endl;
		int checkNum = sys.alternatives(); //'1'なら保存，'0'なら削除
		if (checkNum == 1){
			sys.endMessage(checkNum);
		}else{
			sys.removeDirectory(); //ディレクトリの削除
			sys.endMessage(checkNum);
		}
	}
	catch (exception& ex){ //例外処理
		cout << ex.what() << endl;
		destroyAllWindows(); //OpenCVで作成したウインドウを全て削除する(c35)
		pointcloudlibrary.viewer->~CloudViewer(); //クラウドビューアーの削除
		//異常終了した時はデータを保存する必要がないので削除
		sys.removeDirectory();
		cout << "Data Removed." << endl;
		return -1;
	}
	//sys.endMessage();
	return 0;
}
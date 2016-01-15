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

char directoryName[NOC]; //!<フォルダ名

//(c25)
bool selectObject = false; //!<オブジェクト選択
int trackObject = 0; //!<追跡するオブジェクト
Point origin; //!<オリジナルの座標
Rect selection; //!<選択
void onMouse(int event, int x, int y, int flags, void* param); //!<マウス操作

/*!
 * @brief 関数main()
 * @param なし
 * @return なし
 */
int main()
{
	RETRY: //goto文．再計測をやり直す場合
	//インスタンスの生成
	Kinect kinect; //!<Kinectクラスのインスタンスを生成
	System sys; //!<システム的なメソッドをまとめているクラス
	ImageProcessing imgproc; //!<Imageprocessingクラスのインスタンスを生成
	Drawing draw; //!<drawingクラスのインスタンスを生成
	LeastSquareMethod lsm; //!<最小二乗法を行うクラスのインスタンスを生成(c49)
	PointCloudLibrary pointcloudlibrary(/*false*/true, /*false*/true, /*false*/true, false, false/*true*/); //!<PointCloudLibraryクラスのインスタンスを生成(c57)
	EV3Control ev3control; //!<EV3を制御する用のクラスを作成(c80)

	//変数の宣言
	bool saveev3route_flag = false; //!<EV3の軌道を保存するためのフラグ(c82)
	int save_count = 0; //!<pキーを入力するたびに何回保存したかを数える変数

	//画像関係の変数
	Mat flip_image; //!<確認用の反転した画像
	Mat current_image; //!<現在のフレームの画像(c75)
	Mat bin_image; //!<背景差分によって得られた二値画像(c75)
	Mat background_image; //!<背景画像(c75)
	Mat background_gray_image; //!<背景画像を二値化した画像

	//ポイントクラウド関係の変数(c57)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //!<処理を受け取る点群
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>); //!<法線を格納する変数(c84)

	//EV3ユニットの平面の係数(c78)
	
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
		const char* datadirectory_name = "data"; //データ保存先のディレクトリ名
		char base_dirname[NOC]; //出力先のdataと日付ディレクトリを合わせたもの(c86)
		const string cloudviewer_windowname = "Cloud Viewer"; //クラウドビューアーの名前の定義(c81)
		const string pclvisualizer_windowname = "3D Viewer";
		const string param_windowname = "OpenCV Parameter Setting Window"; //パラメータ調整用のウインドウ(c82)

		kinect.initialize(); //Kinectの初期化
		sys.checkDirectory(datadirectory_name); //base_directoryが存在するか確認し，存在しなければ作成(c81)
		sys.makeDirectoryBasedDate(); //起動時刻をフォルダ名にしてフォルダを作成
		sprintf_s(base_dirname, "%s/%s", datadirectory_name, directoryName); //保存先の基本となるディレクトリ名を定義(c86)
		
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
		sys.countdownTimer(2); 
		PlaySound(TEXT("sourcedata/shutter.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
		system("cls"); //コンソール内の表示をリセット(c64)

		pointcloudlibrary.initializePCLVisualizer(pclvisualizer_windowname); //クラウドビジュアライザーの初期化

		while (/*!pointcloudlibrary.viewer->wasStopped() && */!pointcloudlibrary.visualizer->wasStopped() && kinect.key != 'q' && !GetAsyncKeyState('Q')){ //(c3).メインループ．1フレームごとの処理を繰り返し行う．(c63)CloudViewerが終了処理('q'キーを入力)したらプログラムが終了する
			//タイマー開始(c65)
			sys.startTimer();

			//setMouseCallback(mainwindow_name, onMouse, 0); //(c25)．マウスコールバック関数をセット(c31)

			DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //フレーム更新をイベントとして待つ
			::ResetEvent(kinect.streamEvent); //イベントが発生したら次のイベントに備えてリセット

			//===================================== 画像処理開始 =====================================
			//Kinectから画像を取得し，画像処理を行う
			current_image = kinect.drawRGBImage(image); //RGBカメラの処理
			current_image = imgproc.getUndistortionImage(current_image); //Kinectのキャリブレーションを行い，キャリブレーション結果を適用する(c71)
			imgproc.showImage(main_windowname, current_image);
			//flip(current_image, flip_image, 1);
			//imgproc.showImage("Original - Flip", flip_image); //Kinectから取得した画像を表示
			
			//タイヤも含めて前面の点群を取得する
			imgproc.openCVSettingTrackbar(maskbinimage_windowname);
			bin_image = imgproc.getBackgroundSubstractionBinImage(current_image, background_gray_image/*, imgproc.th, imgproc.med, imgproc.cnt*/);
			//ユニット部だけ切り取る(c77)
			//bin_image = imgproc.getUnitMask(bin_image);
			imgproc.showImage(maskbinimage_windowname, bin_image); //確認用に切り取った画像を表示する
			//===================================== 画像処理終了 =====================================

			//===================================== PCLで行う処理開始 =====================================
			//ポイントクラウドの取得(c57)
			cloud = kinect.getPointCloud(bin_image); //ポイントクラウドの取得(c57)．切り取った画像をもとにする
			pointcloudlibrary.flagChecker(); //各点群処理のフラグをチェックするメソッド(c64)
			cout << "==========================================================================================" << endl;
			cout << "Original PointCloud Size\t=>\t" << cloud->size() << endl;

			//namedWindow("PCL", CV_WINDOW_KEEPRATIO);
			//createTrackbar("thresh", "PCL", &pointcloudlibrary.th,1000);
			//createTrackbar("tol", "PCL", &pointcloudlibrary.tor, 1000);

			//PCLの処理
			if (pointcloudlibrary.passthrough_flag == true){  //外れ値除去(c59)
				cloud = pointcloudlibrary.passThroughFilter(cloud, "z", 400, 30000); //Kinectから取得した外れ値を除去(c60)．与えた軸の中で自分が取得したい範囲の下限と上限を与えることでそれ以外を省く(c81)
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
				cloud = pointcloudlibrary.getExtractPlaneAndClustering(cloud, true, 100,/* 30, */false, true,/* 50, *//*350*/150, /*25000*//*20000*/200000); //Default=0.03(前処理なしの場合)
			}

			//平均座標を取得
			pointcloudlibrary.centroid = pointcloudlibrary.getCentroidCoordinate3d(cloud); //重心座標の計算
			
			//平均座標のポイントクラウドを作成(c83)
			pcl::PointXYZ sphere; //!<平均座標を確認するための球
			sphere.x = pointcloudlibrary.centroid.x; //平均座標のx座標
			sphere.y = pointcloudlibrary.centroid.y; //平均座標のy座標
			sphere.z = pointcloudlibrary.centroid.z; //平均座標のz座標

			lsm.coefficient_plane = lsm.getCoefficient(cloud); //最小二乗法を行い平面の係数[a b c]'を取得する(c78)
			lsm.attitude_angle = lsm.calcYawRollPitch(lsm.coefficient_plane); //姿勢角を取得(c78)
			ev3control.set6DoFEV3(cloud, pointcloudlibrary.centroid, lsm.attitude_angle); //6DoFをまとめる
			
			//EV3の速度を計算(c85)
			//ev3control.getVelocity(); //速度を計算

			//法線を計算(c84)
			//cloud_normals = pointcloudlibrary.getSurfaceNormals(cloud); //法線を計算(c84)
			cout << "==========================================================================================" << endl;
			//===================================== PCLで行う処理終了 =====================================

			//===================================== 点群表示開始 =====================================
			//Kinectから取得した点群を描画
			//pointcloudlibrary.visualizer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals, 30, 10, "normals");
			pointcloudlibrary.visualizer->addSphere(sphere, 10, 0.5, 0.0, 0.0, "sphere"); //平均座標に球を描画
			pointcloudlibrary.visualizer->addPointCloud(cloud, "show cloud"); //点群を描画
			//点群の表示
			pointcloudlibrary.visualizer->spinOnce(); //PCLVisualizerを描画
			//PCLVisualizerに描画した点群を削除する
			pointcloudlibrary.visualizer->removePointCloud("show cloud");
			pointcloudlibrary.visualizer->removeShape("sphere");
			//===================================== 点群表示終了 =====================================
			
			//===================================== キー入力関連開始 =====================================
			//終了のためのキー入力チェック兼表示のためのウェイトタイム
			kinect.key = waitKey(1); //OpenCVのウインドウを表示し続ける
			//キーが入力されていれば以下を実行する．GetAsyncKeyStateを利用することで
			if (GetAsyncKeyState('R')){ //Rキーの入力処理
				destroyAllWindows(); //PCLまたは，OpenCV画面上で'q'キーが入力されたらOpenCVのウインドウを閉じて処理を終了(c66)
				sys.removeDirectory(); //再計測を行う場合は，現在のデータは必要ないため削除
				save_count = 0;
				cout << "Data Removed." << endl;
				system("cls"); //cmdをクリア
				cout << "RETRY" << endl;
				goto RETRY;
			}
			else if (GetAsyncKeyState('P')){ //その時点のデータを保存する．複数回データを計測する際はプログラムを起動しなおす手間が省ける
				char output_basedirpath[NOC]; //その時ごとの保存先のパスを作成する
				sprintf_s(output_basedirpath, "%s/%02d", base_dirname, save_count);
				sys.makeDirectory(base_dirname, save_count); //その時点でのディレクトリを作成
				imgproc.outputImageSelectDirectory(save_count, output_basedirpath, "current-image", current_image); //現在の画像を出力
				imgproc.outputImageSelectDirectory(save_count, output_basedirpath, "mask-image", bin_image); //作成したマスク画像を保存
				pointcloudlibrary.outputPointCloud(save_count, output_basedirpath, "pointcloud", cloud); //現在の点群を出力
				ev3control.output6DoF(save_count, output_basedirpath, "6dof", cloud); //6DoF情報を保存
				ev3control.output6DoFContinuous(base_dirname, "6dof_continuous", cloud); //6DoF情報を続けて保存
				pointcloudlibrary.outputPointCloudPLY(save_count, output_basedirpath, "pointcloud", cloud); //現在の点群をply形式で保存
				draw.gnuplotScriptEV3(save_count, output_basedirpath, "splot_ev3", lsm.coefficient_plane); //gnuplot用のスクリプト
				cout << "Save Current Data." << endl;
				ev3control.save_flag = true; //6DoF情報を出力するフラグをオンにする(c82)
				save_count++; //保存した数をカウントする変数をインクリメントする
			}
			else if (GetAsyncKeyState('L')){ //'l'が入力されたら．EV3の平均速度・ヨー角，軌道が欲しい時に入力する
				saveev3route_flag = true; //処理を実行するフラグをtrueにする
			}
			else if (GetAsyncKeyState('H')){ //'h'が入力されたらヘルプを表示
				sys.showHelpMessage(); //ヘルプメッセージの表示
			}
			//===================================== キー入力関連終了 =====================================
			
			//===================================== 処理時間計測 =====================================
			sys.endTimer(); //タイマーを終了(c65)
			cout << sys.getProcessTimeinMiliseconds() << "[ms], " << sys.getFrameRate() << " fps" << "\n" << endl;
			sys.sum_time = sys.sum_time + sys.getProcessTimeinMiliseconds(); //合計時間を計算
			//===================================== 処理時間計測 =====================================

			//===================================== フラグがtrueのとき実行する処理の開始 =====================================
			//'l'キーが入力されていれば，平均座標の軌道を追跡し続ける(c82)
			if (saveev3route_flag == true){ //フラグがtrueであれば，平均座標の軌道を保存する(c82)
				//outputdatafile.saveDataContinuously(sys.sum_time, ev3control.ev3_6dof, current);
				//EV3の速度を計算(c85)
				ev3control.getVelocityinSec(sys.getProcessTimeinMiliseconds()); //速度を計算
				ev3control.getAverageVelocityAndYaw(); //平均速度とヨー角を計算する
				ev3control.outputEV3RouteContinuous(base_dirname, "ev3route"); //EV3の軌道をファイルに出力
				ev3control.outputControlInformation(sys.sum_time, base_dirname, "time-averagevandyaw"); //時間と平均速度・平均ヨー角を出力
				ev3control.outputControlInformation(); //EV3の速度とヨー角をファイルに出力
			}
			//===================================== フラグがtrueのとき実行する処理の終了 =====================================

			//system("cls"); //コンソール内の表示をリセット(c64)
		}

		//===================================== 計測終了後処理開始 =====================================
		//計測が終了したところ(PCL上, OpenCVウインドウ上, コンソール上で'q'が押されてたとき)
		destroyAllWindows(); //PCLまたは，OpenCV画面上で'q'キーが入力されたらOpenCVのウインドウを閉じて処理を終了(c66)
		pointcloudlibrary.visualizer->~PCLVisualizer(); //PCLVisualizerの削除
		if (saveev3route_flag == true){ //処理の間にlが押されていれば
			draw.gnuplotScriptEV3Route(base_dirname,"splot_ev3route"); //軌道をプロットするスクリプトを保存する
			draw.gnuplotScriptTime2V(); //時間と速度をプロットするスクリプトを生成
			draw.gnuplotScriptTime2Yaw(); //時間とヨー角をプロットするスクリプトを生成
		}

		//データを保存するかの確認
		if (saveev3route_flag == true || ev3control.save_flag == true){ //データを保存するフラグがtrue(=データが保存されている)なら保存するかどうか確認する
			cout << "Save Data?" << endl;
			int checkNum = sys.alternatives(); //'1'なら保存，'0'なら削除
			if (checkNum == 1){ //データを保存するなら
				sys.endMessage(checkNum);
			}
			else{ //データを削除するなら
				sys.removeDirectory(); //ディレクトリの削除
				sys.endMessage(checkNum);
			}
		}
		else{ //データが一度も保存されていなければ確認せずにディレクトリを削除
			sys.removeDirectory(); //ディレクトリの削除
			sys.endMessage();
		}
		//===================================== 計測終了後処理終了 =====================================

	}
	catch (exception& ex){ //例外処理
		cout << ex.what() << endl;
		destroyAllWindows(); //OpenCVで作成したウインドウを全て削除する(c35)
		//pointcloudlibrary.viewer->~CloudViewer(); //クラウドビューアーの削除
		pointcloudlibrary.visualizer->~PCLVisualizer(); //PCLVisualizerの削除
		//異常終了した時はデータを保存する必要がないので削除
		sys.removeDirectory();
		cout << "Data Removed." << endl;
		return -1;
	}
	return 0;
}
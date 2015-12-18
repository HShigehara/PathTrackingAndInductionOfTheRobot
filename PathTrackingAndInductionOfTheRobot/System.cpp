/*
 * @file System.hpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief kinect処理のクラスのヘッダ
 * @date 2014.12.19
 * @author H.Shigehara
 */

/* */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "System.hpp"

/*!
* @brief System::System().コンストラクタ
*/
System::System()
{
	FlagStartTimer = false; //スタート用のタイマーが実行されたかのフラグを初期化(c65)
	FlagEndTimer = false; //終了用のタイマーが実行されたかのフラグを初期化(c65)
	time = 0.0; //時間計測用の変数を初期化(c65)
}

/*!
* @brief System::~System().デストラクタ
*/
System::~System()
{
	//デストラクタの処理はなし
}

/*!
* @brief System::startMessage().プログラム起動時のメッセージ
*/
void System::startMessage()
{
	cout << "==================================================================" << endl;
	cout << " Starting the Program...." << endl;
	//cout << " Please Enclose the Object You Want to Track." << endl;
	cout << " If You Enter a 'q' Key, the Program Terminates." << endl;
	cout << " If You Enter a 'r' Key, the Program Restart." << endl;
	//cout << " To Initialize Tracking, Re-Select the Object with Mouse." << endl;
	cout << "\n";
	cout << " Switching of Point Cloud Processing." << endl;
	cout << "  Pass Through Filter \t\t -> \t Enter 'x' Key." << endl;
	cout << "  Downsampling \t\t\t -> \t Enter 'c' Key." << endl;
	cout << "  Remove Outlier \t\t -> \t Enter 'v' Key." << endl;
	cout << "  Moving Least Square \t\t -> \t Enter 'b' Key." << endl;
	cout << "  Extract Plane & Clustering \t -> \t Enter 'n' Key." << endl;
	cout << "==================================================================" << endl;

	return;
}

/*!
 * @brief System::endMessage().プログラム終了時のメッセージ
 */
void System::endMessage(int cNum)
{
	cout << "==================================================================" << endl;
	cout << "Closing the Program...." << endl;
	if (cNum == 1){
		cout << "Data Has Been Output to \"data/" << directoryName << "\"." << endl;
		openDirectory();
	}
	cout << "==================================================================" << endl;
	
	return;
}

/*!
* @brief System::endMessage().プログラム終了時のメッセージ
*/
void System::endMessage()
{
	cout << "==================================================================" << endl;
	cout << "Closing the Program...." << endl;
	cout << "==================================================================" << endl;
	return;
}

/*!
 * @brief メソッドcountdownTimer()．タイマーによるカウントダウン
 * @param int time_sec 
 */
void System::countdownTimer(int time_sec)
{
	string playfile_name;
	while (time_sec > 0){
		cout << time_sec << " seconds" << "\r";
		switch (time_sec)
		{
		case 1: 
			PlaySound(TEXT("sourcedata/count1.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 2: 
			PlaySound(TEXT("sourcedata/count2.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 3:
			PlaySound(TEXT("sourcedata/count3.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 4:
			PlaySound(TEXT("sourcedata/count4.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 5:
			PlaySound(TEXT("sourcedata/count5.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 6:
			PlaySound(TEXT("sourcedata/count6.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 7:
			PlaySound(TEXT("sourcedata/count7.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 8:
			PlaySound(TEXT("sourcedata/count8.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 9:
			PlaySound(TEXT("sourcedata/count9.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		case 10:
			PlaySound(TEXT("sourcedata/count10.wav"), NULL, (SND_ASYNC | SND_FILENAME)); //音声ファイルを再生
			break;
		default: break;
		}
		Sleep(1000);
		time_sec = time_sec - 1;
	}
	cout << "\n";
	return;
}

/*!
 * @brief メソッドstartTimer().タイマーを開始する
 */
void System::startTimer()
{
	f = 1000.0 / getTickFrequency();
	start = getTickCount(); //スタート
	FlagStartTimer = true; //スタート用のタイマーが実行されたのでフラグをtrueに
	return;
}

/*!
* @brief メソッドendTimer().タイマーを終了する
*/
void System::endTimer()
{
	if (FlagStartTimer == true){
		end = getTickCount();
		time = (end - start) * f;
		FlagEndTimer = true;
	}
	else{
		cerr << "Before you use endTimer() method, please run the System::startTimer()." << endl;
		exit(-1);
	}
	return;
}

/*!
* @brief メソッドgetTime().計測した時間を取得する(c65)
* @return double time
*/
double System::getProcessTimeinMiliseconds()
{
	if (FlagStartTimer == true && FlagEndTimer == true){
		return time;
	}
	else if (FlagStartTimer == false && FlagEndTimer == true){
		cerr << "Before you use getProcessTimeinMiliseconds() method, please run the System::startTimer()." << endl;
		exit(-1);
	}
	else if (FlagStartTimer==true && FlagEndTimer == false){
		cerr << "Before you use getProcessTimeinMiliseconds() method, please run the System::endTimer()." << endl;
		exit(-1);
	}
	else{
		cerr << "Before you use getProcessTimeinMiliseconds() method, please run the System::startTimer() and System::endTimer()." << endl;
		exit(-1);
	}
}

/*!
* @brief メソッドgetFrameRate().フレームレートを取得する(c65)
* @return double time
*/
double System::getFrameRate()
{
	if (FlagStartTimer == true && FlagEndTimer == true){
		fps = 1000.0 / time;
		return fps;
	}
	else if (FlagStartTimer == false && FlagEndTimer == true){
		cerr << "Before you use getFrameRate() method, please run the System::startTimer()." << endl;
		exit(-1);
	}
	else if (FlagStartTimer == true && FlagEndTimer == false){
		cerr << "Before you use getFrameRate() method, please run the System::endTimer()." << endl;
		exit(-1);
	}
	else{
		cerr << "Before you use getFrameRate() method, please run the System::startTimer() and System::endTimer()." << endl;
		exit(-1);
	}
}

/*!
 * @brief System::checkDirectory()．ファイルやディレクトリがあるかチェックし，無ければ作成する(c81)
 * @param string checkdir_name
 */
void System::checkDirectory(const char* check_dirname)
{
	struct stat st;
	int re = stat(check_dirname, &st);
	if (re != 0){
		_mkdir(check_dirname);
		cout << "Make " << check_dirname << ". " << endl;
	}
	return;
}

/*!
* @brief System::makeDirectory().ディレクトリを作成
*/
void System::makeDirectory()
{
	//フォルダ名を区別するために時刻を取得しておく(c4)
	SYSTEMTIME st;
	char savedirpath[NOC];

	GetLocalTime(&st);
	sprintf_s(directoryName, "[%4d%02d%02d]%02d%02d%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	sprintf_s(savedirpath, "data/%s", directoryName);
	_mkdir(savedirpath);

	return;
}

/*!
* @brief System::removeDirectory().ディレクトリを削除(c21)
*/
void System::removeDirectory()
{
	char rmdirCommand[NOC]; //ディレクトリを削除するコマンド(c21).変数名を変更&このメソッドのみで有効な変数(c30)

	//ディレクトリを削除する
	cout << directoryName << endl;
	sprintf_s(rmdirCommand, "rmdir /s /q \"data\\%s\"", directoryName); //ディレクトリの削除コマンドを書く．パスは[\"\"]で囲み，階層があるときは[\\]で書く
	system(rmdirCommand);
	cout << "Not Save.\n" << endl;

	return;
}

/*!
* @brief System::alternatives().Yes/Noの2択のチェック(c21)
* @return checkNum
*/
int System::alternatives()
{
	char checkNum; //!<0か1をチェックするための変数(c27).このメソッドのみで有効な変数(c30)

	cout << "1. Yes" << endl;
	cout << "0. No" << endl;
	cin >> checkNum;
	cout << "\n";

	while (1)
	{
		if (checkNum == '0'){
			return (atoi(&checkNum));
		}
		else if (checkNum == '1'){
			return (atoi(&checkNum));
		}
		else{
			cout << "Please Input 1 or 0." << endl;
			cout << "1. Yes" << endl;
			cout << "0. No" << endl;
			cin >> checkNum;
		}
	}
}

/*!
* @brief System::openDireectory().出力したディレクトリを開く(c39)
*/
void System::openDirectory()
{
	cout << "Opening This Directory." << endl;
	char openDirectoryCommand[NOC]; //自動で開くウインドウのパス
	sprintf_s(openDirectoryCommand, "explorer \"data\\%s\"", directoryName); //ディレクトリを開くコマンド
	system(openDirectoryCommand); //ディレクトリを開くコマンドを実行

	return;
}

/*!
* @brief System::outputData().データをファイルに書き出すメソッド(c41)
* @param outputDataName, outputData, countDataNum
*/
void System::outputAllData(const string* outputDataName, outputData* outputData, int countDataNum)
{
	//ファイルポインタ
	FILE *extractedCoordinate; //!<抽出した座標の距離(c7)
	extractedCoordinate = NULL; //!<ファイルポインタの初期化(c7)

	//(X,Y,Z)データ格納用のファイル
	char outputDataPath[NOC]; //!<データファイル出力の際のパス(c37)
	
	sprintf_s(outputDataPath, "%s/%s", directoryName, outputDataName); //(c7)
	fopen_s(&extractedCoordinate, outputDataPath, "w"); //(c7)
	if (outputDataPath == NULL){ //ファイルオープンエラー処理(c40)
		cerr << outputDataPath << " is Not Opened.";
		exit(1);
	}

	//ファイルに出力する処理(c42)
	for (int i = 2; i < countDataNum - 15; i++){ //最初と最後のいくつかのデータをファイルに出力しない(c41)
		fprintf_s(extractedCoordinate, "%f %f %f %f\n", outputData[i].totalTime, outputData[i].x, outputData[i].y, outputData[i].z);
	}
	fclose(extractedCoordinate); //(c8)

	return;
}

/*!
* @brief System::outputVideo().動作確認用に動画を出力するメソッド
* @param cameraParamFile
* @return VideoWriter writer
*/
VideoWriter System::outputVideo(const string* outputVideoName)
{
	//動画を出力(c40)
	char outputVideoPath[NOC]; //!<動画出力時のパス(c38)
	sprintf_s(outputVideoPath, "%s/%s", directoryName, outputVideoName); //(c38)
	//VideoWriter writer(outputVideoPath, /*CV_FOURCC('D','I','B',' ')*/-1/*CV_FOURCC('X','V','I','D')*//*CV_FOURCC('P','M','I','1')*/, 20, /*Size(WIDTH, HEIGHT)*/Size(640, 480), true); //動画に出力.録画が必要なときはコメントアウト(c35)
	VideoWriter writer(outputVideoPath, /*CV_FOURCC('D','I','B',' ')*/-1/*CV_FOURCC('X','V','I','D')*//*CV_FOURCC('P','M','I','1')*/, 20, /*Size(WIDTH, HEIGHT)*/Size(640, 480), true); //動画に出力.録画が必要なときはコメントアウト(c35)
	if (!writer.isOpened()){ //オープンエラー処理(c40)
		cerr << outputVideoPath << " is Not Opened." << endl;
	}
	return (writer);
}
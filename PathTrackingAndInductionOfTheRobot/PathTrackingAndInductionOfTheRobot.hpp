/*
 * @file PathTrackingAndInductionOfTheRobot.h
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief 距離データを取得し画像として表示するプログラムのヘッダファイル
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __PATHTRACKINGANDINDUCTIONTHEROBOT_HPP__
#define __PATHTRACKINGANDINDUCTIONTHEROBOT_HPP__

//よく使うヘッダはstdafx.hに
/* 構造体 */
typedef struct Point3{ //抽出された座標を保存する構造体(c37)
	int x;
	int y;
	USHORT z;
}Point3ius;

typedef struct outputData{ //ファイルに出力するデータ群(c41)
	double totalTime;
	float x;
	float y;
	float z;
}outputData;

//姿勢構造体の定義(c78)
typedef struct DoF{
	double x; //x座標
	double y; //y座標
	double z; //z座標
	double yaw; //y軸回転
	double roll; //z軸回転
	double pitch; //x軸回転
}DoF6d;

typedef struct AttitudeAngle{
	double yaw;
	double roll;
	double pitch;
}AttitudeAngle3d;

/* グローバル変数 */
//画像関係
extern Mat image; //!<RGB画像格納用の変数
//extern Mat depth_image; //!<距離画像格納用の変数(c58)

//ファイル名関係
extern char directoryName[NOC]; //!<フォルダ名

//(c6)
extern int extractedPointOneDim[ALLPIXEL]; //!<抽出された座標の1次元の値
extern int extractedNum; //!<(c6)colorExtraction()で何ピクセル抽出されたかカウントする変数

//(c8)
extern Point3ius extCoordinate[ALLPIXEL]; //!<抽出された座標を保存する変数(c37)

//(c49)
extern Vector4 XYZCoordinate[ALLPIXEL]; //!<3次元に変換された(X,Y,Z)のデータ(c49)

/* CamShift用変数(c25) */
extern bool backprojMode; //!<バックプロジェクトモード
extern bool selectObject; //!<オブジェクト選択
extern int trackObject; //!<追跡するオブジェクト
extern Point origin; //!<オリジナルの座標
extern Rect selection; //!<選択
extern int vmin, vmax, smin; //!<HSVの範囲指定
extern void onMouse(int event, int x, int y, int, void*); //!<マウス操作
/* (c25) */
extern Rect trackWindow; //!<追跡ウインドウ
extern int hsize;
extern float hranges[];//!<Hの範囲
extern const float* phranges;

/* (c26) */
extern /*int*/bool avgFlag; //!<平均を計算したとき用のフラグ(c30)
extern /*int*/bool mouseFlag; //!<マウス操作確認用のフラグ(c26)



/* インクルードガードの終了 */
#endif /* __PATHTRACKINGANDINDUCTIONTHEROBOT_HPP__ */
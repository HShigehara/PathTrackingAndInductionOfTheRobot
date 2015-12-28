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
typedef struct DoF6d{
	double x; //x座標
	double y; //y座標
	double z; //z座標
	double yaw; //y軸回転
	double roll; //z軸回転
	double pitch; //x軸回転
}DoF6d;

typedef struct DoF6i{
	int x;
	int y;
	int z;
	int yaw;
	int roll;
	int pitch;
}DoF6i;

typedef struct AttitudeAngle{
	double yaw;
	double roll;
	double pitch;
}AttitudeAngle3d;

typedef struct ControlParamd{
	double velocity;
	double yaw;
}ControlParamd;

/* グローバル変数 */
//画像関係
extern Mat image; //!<RGB画像格納用の変数

//ファイル名関係
extern char directoryName[NOC]; //!<フォルダ名

/* CamShift用変数(c25) */
extern bool selectObject; //!<オブジェクト選択
extern int trackObject; //!<追跡するオブジェクト
extern Point origin; //!<オリジナルの座標
extern Rect selection; //!<選択
extern void onMouse(int event, int x, int y, int, void*); //!<マウス操作

/* インクルードガードの終了 */
#endif /* __PATHTRACKINGANDINDUCTIONTHEROBOT_HPP__ */
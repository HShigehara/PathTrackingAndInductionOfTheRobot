/*
 * @file PathTrackingAndInductionOfTheRobot.h
 * @brief 距離データを取得し画像として表示するプログラムのヘッダファイル
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __PATHTRACKINGANDINDUCTIONTHEROBOT_HPP__
#define __PATHTRACKINGANDINDUCTIONTHEROBOT_HPP__

//よく使うヘッダはstdafx.hに
/* 構造体 */
/*!
 * @struct Point3
 * @brief 抽出された座標を保存する構造体(c37)
 */
typedef struct Point3{
	int x; //!<x座標
	int y; //!<y座標
	USHORT z; //!<z座標
}Point3ius;

/*!
 * @struct outputData
 * @brief ファイルに出力するデータ群(c41)
 */
typedef struct outputData{
	double totalTime; //!<合計時間
	float x; //!<x座標
	float y; //!<y座標
	float z; //!<z座標
}outputData;

/*!
 * @struct DoF6d
 * @brief 姿勢構造体の定義(c78)．doule型
 */
typedef struct DoF6d{
	double x; //!<x座標
	double y; //!<y座標
	double z; //!<z座標
	double yaw; //!<ヨー角
	double roll; //!<ロール角
	double pitch; //!<ピッチ角
}DoF6d;

/*!
 * @struct DoF6i
 * @brief姿勢構造体の定義．int型
 */
typedef struct DoF6i{
	int x; //!<x座標
	int y; //!<y座標
	int z; //!<z座標
	int yaw; //!<ヨー角
	int roll; //!<ロール角
	int pitch; //!<ピッチ角
}DoF6i;

/*!
 * @struct AttitudeAngle
 * @brief 姿勢角
 */
typedef struct AttitudeAngle{
	double yaw; //!<ヨー角
	double roll; //!<ロール角
	double pitch; //!<ピッチ角
}AttitudeAngle3d;

/*!
 * @struct ControlParamd
 * @brief 走行制御用構造体．double型
 */
typedef struct ControlParamd{
	double velocity; //!<速度
	double yaw; //!<ヨー角
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
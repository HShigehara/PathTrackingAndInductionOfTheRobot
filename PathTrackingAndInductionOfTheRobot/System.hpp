/*
* @file System.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief kinect処理のクラスのヘッダ
* @date 2014.12.19
* @author H.Shigehara
*/


/* インクルードガード */
#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

class System
{
private:
	char fullPathName[NOC]; //!<フルパスを取得(c38)
	
	//タイマー系の変数(c65)
	bool FlagStartTimer; //!<スタート用のタイマーが実行されたかのフラグ.初期値はfalse(c65)
	bool FlagEndTimer; //!<終了用のタイマーが実行されたかのフラグ.諸お基地はfalse(c65) 
	int64 start; //スタート時のタイマー変数
	double f;
	int64 end; //終了時のタイマー変数
	double time; //!<処理時間の結果
	double fps; //!<フレームレート
	//double sumTime; //!<処理の合計時間

	//int time_ms; //!<カウントダウンタイマー用の変数(c75)

public:
	System();
	~System();

	void countdownTimer(int time_ms); //!<引数の時間[ms]に応じてカウントダウンを開始する(c75)

	void startMessage(); //!<プログラム開始時のメッセージを表示(c26)
	void endMessage(int cNum); //!<プログラム終了時のメッセージを表示(c38)
	void endMessage(); //!<プログラム終了時のメッセージを表示(c63)

	void startTimer(); //!<タイマーを開始(c65)
	void endTimer(); //!<タイマーを終了(c65)
	double getProcessTimeinMiliseconds(); //!<計測した時間をミリ秒単位で取得.startTimer()とendTimer()が実行されていることが前提(c65)
	double getFrameRate(); //!<フレームレートを取得.startTimer()とendTimer()が実行されていることが前提(c65)

	void makeDirectory(); //ディレクトリの作成
	void removeDirectory(/*int cNum*/); //!<取得したデータが不要だった場合ディレクトリを削除する
	
	int alternatives(); //!<数字の入力をチェックする
	
	void openDirectory(); //!<ディレクトリを開く(c38)
	
	void outputAllData(const string* outputDataName, outputData* outputData, int countDataNum); //!<データをファイルに書き出すメソッド(c41)
	
	VideoWriter outputVideo(const string* outputVideoName); //!<動画を出力する
};

#endif /* __SYSTEM_HPP__ */
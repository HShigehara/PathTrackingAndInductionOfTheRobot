// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#define _CRT_SECURE_NO_WARNINGS //fopen等昔の関数の警告を非表示にする

/* よく利用するヘッダーをプリコンパイルする */
/* ヘッダファイルのインクルード */
#include <iostream> //!<標準入出力ストリーム
#include <sstream> //!<ストリングストリーム
#include <fstream> //!<ファイル入出力ストリーム
#include <string> //!<文字列
using namespace std; //!<名前空間

#include <stdio.h> //!<Cの標準入出力ストリーム
#include <stdlib.h> //!<標準ライブラリ
#include <direct.h> //!<ディレクトリを作成するため用いる
#include <math.h> //!<数学関数用のライブラリ(c12)
#include <ctype.h> //!<文字の種類の判定や文字の変換を行う(c25)
#include <sys/stat.h> //!<ファイルやディレクトリの存在を確認するためのヘッダ(c81)

/* NuiApi.hの前にWindows.hをインクルードする */
#include <Windows.h>
#include <mmsystem.h>
#include <NuiApi.h>

/* OpenCV関連のインクルード */
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video\tracking.hpp> //!<動画のトラッキングを行うためのライブラリ(c25)
#include <opencv2\flann\flann.hpp>
using namespace cv; //!<名前空間

/* PCL関連のインクルード */
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\io\io.h>
#include <pcl\io\pcd_io.h> //.pcd出力用
#include <pcl\io\ply_io.h> //.ply読み込み用
#include <pcl\common\common_headers.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\filters\passthrough.h> //Kinectから取得した初期の外れ値を削除
#include <pcl\filters\statistical_outlier_removal.h> //外れ値フィルター用
#include <pcl\filters\radius_outlier_removal.h> //外れ値フィルター用(c60)
#include <pcl\kdtree\kdtree_flann.h> //スムージング用
#include <pcl\surface\mls.h> //スムージング用
#include <pcl\filters\voxel_grid.h> //ダウンサンプリング用
//平面検出・除去処理用(c61)
#include <pcl\ModelCoefficients.h>
#include <pcl\sample_consensus\method_types.h>
#include <pcl\sample_consensus\model_types.h>
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\filters\extract_indices.h>
//クラスタリング(c62)
#include <pcl\features\normal_3d.h>
#include <pcl\segmentation\extract_clusters.h>

//法線計算(c84)
#include <pcl\features\integral_image_normal.h>

#include <boost\thread\thread.hpp>

//ICPアルゴリズムによる位置合わせ
#include <pcl\registration\icp.h>
#include <pcl\PCLPointField.h>

//#include "PCLAdapter.h"



/* 定義 */
//マクロ
#define WIDTH 640 //!<画像の幅
#define HEIGHT 480 //!<画像の高さ
#define ALLPIXEL WIDTH*HEIGHT //!<1フレームの全ピクセル数
#define NOC 64 //!<Number of Characters．(ファイルの名前を付けるときの文字数制限)
#define OUTPUTDATA_MAX 10000 //!<出力するデータの上限


// TODO: プログラムに必要な追加ヘッダーをここで参照してください。

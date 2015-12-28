/*
 * @file PointCloudLibrary.hpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
 * @brief PCL関連の処理を行うクラスのヘッダ
 * @date 2015.10.30
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __POINTCLOUDLIBRARY_HPP__
#define __POINTCLOUDLIBRARY_HPP__

/* インクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class PointCloudLibrary
* @brief 点群処理を行うクラス
*/
class PointCloudLibrary{
private:

public:
	PointCloudLibrary(); //!<コンストラクタ
	PointCloudLibrary(bool passthroughflag, bool downsamplingflag, bool statisticaloutlierremovalflag, bool mlsflag, bool extractplaneflag); //!<コンストラクタ(c64)
	~PointCloudLibrary(); //!<デストラクタ

	void initializePCLVisualizer(string pclvisualizer_name); //!<PCL Visualizerの初期化

	void loadPLY(char* ply_name); //!<.plyファイルの読み込み
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model; //!<.plyファイルの点群(モデル)

	void flagChecker(); //!<フラグを判定するメソッド(c64)

	//外れ値除去
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, char* axis, float min, float max); //!<パススルーフィルタ．zの値の距離に応じてカット可能
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //!<外れ値を除去するメソッド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //!<外れ値を除去するメソッド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ); //!<ダウンサンプリングを行うメソッド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius); //!<スムージングを行うメソッド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getExtractPlaneAndClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, int maxIterations,/* double threshold, */bool negative1, bool negative2,/* double tolerance, */int minClusterSize, int maxClusterSize); //!<平面検出とクラスタリング

	Point3d centroid; //!<平均座標
	Point3d getCentroidCoordinate3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //!<取得した点群の平均座標を取得するメソッド

	pcl::PointCloud<pcl::Normal>::Ptr getSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //!<法線を計算する
	
	//PCLVisualizer用
	pcl::visualization::PCLVisualizer *visualizer; //PCL Visualizer

	int th; //!<平面検出とクラスタリング時用の閾値スライダー変数
	int tor; //!<平面検出とクラスタリング時用のスライダー変数

	//各点群処理を行うか否かのフラグ変数(c64)
	bool passthrough_flag; //!<パススルーフィルターを用いるかどうかのフラグ
	bool downsampling_flag; //!<ダウンサンプリングを行うかどうかのフラグ
	bool statisticaloutlierremoval_flag; //!<外れ値を除去するかどうかのフラグ
	bool mls_flag; //!<スムージングを行うかどうかのフラグ
	bool extractplane_flag; //!<平面検出とクラスタリングを行うかどうかのフラグ

	void outputPointCloud(int save_count, char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud); //!<点群を出力するメソッド
	void outputPointCloudPLY(int save_count, char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud); //!<点群をply形式で保存するメソッド
};

/* インクルードガードの終了 */
#endif /* __POINTCLOUDLIBRARY_HPP__ */
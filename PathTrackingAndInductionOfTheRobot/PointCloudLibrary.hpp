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
	PointCloudLibrary(); //コンストラクタ
	PointCloudLibrary(bool passthroughflag, bool downsamplingflag, bool statisticaloutlierremovalflag, bool mlsflag, bool extractplaneflag); //コンストラクタ(c64)
	~PointCloudLibrary(); //デストラクタ

	void initializePointCloudViewer(string cloudviewer_name); //!<ポイントクラウドビューアーを初期化
	void initializePCLVisualizer(string pclvisualizer_name);

	void loadPLY(char* ply_name); //.plyファイルの読み込み
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model; //.plyファイルの点群(モデル)

	void flagChecker(); //フラグを判定するメソッド(c64)

	//外れ値除去
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, char* axis, float min, float max); //!<パススルーフィルタ．zの値の距離に応じてカット可能
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ); //!<ダウンサンプリングの
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius); //!<スムージング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getExtractPlaneAndClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, int maxIterations, double threshold, bool negative1, bool negative2, double tolerance, int minClusterSize, int maxClusterSize); //!<平面検出とクラスタリング

	Point3d centroid; //!<平均座標
	Point3d getCentroidCoordinate3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //取得した点群の平均座標を取得するメソッド

	pcl::PointCloud<pcl::Normal>::Ptr getSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud); //!<法線を計算する
	
	//クラウドビューワー用
	pcl::visualization::CloudViewer *viewer;

	//PCLVisualizer用
	pcl::visualization::PCLVisualizer *visualizer;

	//各点群処理を行うか否かのフラグ変数(c64)
	bool passthrough_flag;
	bool downsampling_flag;
	bool statisticaloutlierremoval_flag;
	bool mls_flag;
	bool extractplane_flag;
};

/* インクルードガードの終了 */
#endif /* __POINTCLOUDLIBRARY_HPP__ */
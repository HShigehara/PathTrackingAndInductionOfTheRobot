/*
* @file PointCloudLibrary.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL関連の処理を行うクラスのヘッダ
* @date 2015.10.30
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __POINTCLOUDLIBRARY_HPP__
#define __POINTCLOUDLIBRARY_HPP__

/* インクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class PointCloudLibrary
* @brief 点群処理を行うクラス
*/
class PointCloudLibrary{
private:

public:
	PointCloudLibrary(); //コンストラクタ
	PointCloudLibrary(bool flagPassThrough, bool flagDownsampling, bool flagStatisticalOutlierRemoval, bool flagMLS, bool flagExtractPlane); //コンストラクタ(c64)
	~PointCloudLibrary(); //デストラクタ

	void initializePointCloudViewer(string cloudViewerName);
	void flagChecker(); //フラグを判定するメソッド(c64)

	void loadPLY(char* ply_filename); //.plyファイルの読み込み

	//外れ値除去
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	
	//ダウンサンプリング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ);

	//スムージング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius);

	//平面検出とクラスタリング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getExtractPlaneAndClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, int maxIterations, double threshold, bool negative1, bool negative2, double tolerance, int minClusterSize, int maxClusterSize);

	//ICPアルゴリズムによる位置合わせ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputModel);
	
	//点群の重心座標と距離を取得
	Point3f getCentroidCoordinate3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);

	//最小二乗法
	Eigen::Vector3f leastSquareMethod(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);

	//法線計算
	pcl::PointCloud<pcl::Normal>::Ptr getSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	
	//クラウドビューワー用
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model; //.plyファイルの点群(モデル)

	//各点群処理を行うか否かのフラグ変数(c64)
	bool FlagPassThrough;
	bool FlagDownsampling;
	bool FlagStatisticalOutlierRemoval;
	bool FlagMLS;
	bool FlagExtractPlane;
	bool FlagIcpRegistration;
};

/* インクルードガードの終了 */
#endif /* __POINTCLOUDLIBRARY_HPP__ */
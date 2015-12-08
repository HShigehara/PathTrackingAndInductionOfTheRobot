﻿/*
* @file PointCloudMethod.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL関連の処理を行うクラスのヘッダ
* @date 2015.10.30
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __POINTCLOUDMETHOD_HPP__
#define __POINTCLOUDMETHOD_HPP__

/* インクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class PointCloudMethod
* @brief 点群処理を行うクラス
*/
class PointCloudMethod{
private:

public:
	PointCloudMethod(); //コンストラクタ
	PointCloudMethod(bool flagPassThrough, bool flagDownsampling, bool flagStatisticalOutlierRemoval, bool flagMLS, bool flagExtractPlane, bool flagIcpRegistration); //コンストラクタ(c64)
	~PointCloudMethod(); //デストラクタ

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
#endif /* __POINTCLOUDMETHOD_HPP__ */
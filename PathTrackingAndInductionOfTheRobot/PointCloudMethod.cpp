﻿/*
* @file PointCloudMethod.cpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL関連の処理を行うクラス
* @date 2015.10.30
* @author H.Shigehara
*/

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "PointCloudMethod.hpp"

/*!
* @brief メソッドPointCloudMethod::PointCloudMethod().コンストラクタ
* @param なし
* @return なし
*/
PointCloudMethod::PointCloudMethod()
{
	//コンストラクタ
}

/*!
 * @brief メソッドPointCloudMethod::PointCloudMethod().コンストラクタ(c64)
 * @param bool flag_removeOutlier, bool flag_downsampling, bool flag_MLS, bool flag_extractPlane
 * @return なし
 */
PointCloudMethod::PointCloudMethod(bool flagPassThrough, bool flagDownsampling, bool flagStatisticalOutlierRemoval, bool flagMLS, bool flagExtractPlane, bool flagIcpRegistration)
{
	//コンストラクタ
	FlagPassThrough = flagPassThrough;
	FlagDownsampling = flagDownsampling;
	FlagStatisticalOutlierRemoval = flagStatisticalOutlierRemoval;
	FlagMLS = flagMLS;
	FlagExtractPlane = flagExtractPlane;
	FlagIcpRegistration = flagIcpRegistration;
}

/*!
* @brief メソッドPointCloudMethod::~PointCloudMethod().デストラクタ
* @param なし
* @return なし
*/
PointCloudMethod::~PointCloudMethod()
{
	//デストラクタ
}


/*!
 * @brief メソッドPointCloudMethod::initializePointCloudViewer().ポイントクラウドビューアーを初期化するメソッド(c57)
 * @param string cloudViewerName
 * @return なし
 */
void PointCloudMethod::initializePointCloudViewer(string cloudViewerName)
{
	viewer = new pcl::visualization::CloudViewer(cloudViewerName);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> __viewer(new pcl::visualization::PCLVisualizer(cloudViewerName)); //仮
	//__viewer(new pcl::visualization::PCLVisualizer(cloudViewerName));
	return;
}

void PointCloudMethod::loadPLY(char* ply_filename)
{
	pcl::io::loadPLYFile(ply_filename, *model);
	return;
}

/*!
 * @brief メソッドPointCloudMethod::flagChecker().PCL処理に関する処理の有無を判定するフラグ変数を反転させるメソッド(c64)
 * @param なし
 * @return なし
 */
void PointCloudMethod::flagChecker()
{
	if (GetAsyncKeyState('X')){ //Xが入力されたので、パススルーフィルターのフラグを反転
		FlagPassThrough = !FlagPassThrough;
	}
	if (GetAsyncKeyState('C')){	//Cが入力されたので、ダウンサンプリング処理のフラグを反転
		FlagDownsampling = !FlagDownsampling;
	}
	if (GetAsyncKeyState('V')){ //Vが入力されたので、統計的な外れ値除去処理のフラグを反転
		FlagStatisticalOutlierRemoval = !FlagStatisticalOutlierRemoval;
	}
	if (GetAsyncKeyState('B')){  //Nが入力されたので、MLS処理のフラグを反転
		FlagMLS = !FlagMLS;
	}
	if (GetAsyncKeyState('N')){	//Mが入力されたので、平面検出のフラグを反転
		FlagExtractPlane = !FlagExtractPlane;
	}
	if (GetAsyncKeyState('M')){
		FlagIcpRegistration = !FlagIcpRegistration;
	}
	cout << "範囲外除去(X) => " << FlagPassThrough << " ダウンサンプリング(C) => " << FlagDownsampling << " 外れ値(V) => " << FlagStatisticalOutlierRemoval << " MLS(B) => " << FlagMLS << " 平面検出(N) => " << FlagExtractPlane << endl;
	return;
}
/*!
 * @brief メソッドPointCloudMethod::passThroughFilter().パススルーフィルター
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before\tPassThroughFilter\t=>\t" << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルター後用のポイントクラウド
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(inputPointCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimitsNegative(true); //setFilterLimits(float min, float max)で指定した範囲以外を削除(c69)
	pass.setFilterLimits(0.3, 40.0);
	pass.filter(*filtered);
	
	cout << "after\tPassThroughFilter\t=>\t" << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::removeOutlier()．外れ値を除去するメソッド(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before\tRemove Outlier\t\t=>\t" << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> fl;
	
	fl.setInputCloud(inputPointCloud);
	fl.setMeanK(10);
	fl.setStddevMulThresh(0.1);
	fl.filter(*filtered);
	fl.setNegative(true);

	cout << "after\tRemove Outlier\t\t=>\t" << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::radiusOutlierRemoval()．外れ値を除去するメソッド(c60)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before\tRadiusOutlierRemoval\t=>\t" << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;

	ror.setInputCloud(inputPointCloud);
	ror.setRadiusSearch(0.8);
	ror.setMinNeighborsInRadius(2);
	ror.filter(*filtered);

	cout << "after\tRadiusOutlierRemoval\t=>\t" << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::downSamplingUsingVoxelGridFilter()．ダウンサンプリング処理を行うメソッド(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ)
{
	cout << "before\tVoxel Grid Filter\t=>\t" << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(inputPointCloud);
	//sor.setLeafSize()でダウンサンプリングの程度を変更
	vg.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
	//vg.setLeafSize(0.003f, 0.003f, 0.003f); //Default
	//sor.setLeafSize(0.03f, 0.03f, 0.03f); //少ない
	vg.filter(*filtered);

	//ポイントクラウドをしっかり保持できているかサイズを確認
	cout << "after\tVoxel Grid Filter\t=>\t" << filtered->size() << endl; //出力されるポイントクラウドのサイズ
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::smoothingUsingMovingLeastSquare()．スムージングを行うメソッド(c60)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius)
{
	cout << "before\tMLS\t\t\t=>\t" << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング処理後用のポイントクラウド
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>()); //KdTreeの作成
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls; //スムージング処理

	mls.setComputeNormals(compute_normals); //法線の計算
	//各パラメータの設定
	mls.setInputCloud(inputPointCloud);
	mls.setPolynomialFit(polynomial_fit);
	mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.003); //Default
	mls.setSearchRadius(radius);
	//mls.process(mls_points); //出力
	mls.process(*filtered); // 出力

	cout << "after\tMLS\t\t\t=>\t" << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::extractPlane().平面を検出するメソッド
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::getExtractPlaneAndClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, int maxIterations, double threshold, bool negative1, bool negative2, double tolerance, int minClusterSize, int maxClusterSize)
{
	cout << "before\tExtract Plane\t\t=>\t" << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	//セグメンテーションオブジェクトの生成
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	//オプション
	seg.setOptimizeCoefficients(optimize);

	//Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);

	//クラスタリング
	seg.setMaxIterations(maxIterations); //Default->100
	//

	seg.setDistanceThreshold(threshold);

	int i = 0, nr_points = (int)inputPointCloud->points.size();
	while (inputPointCloud->points.size() > 0.3*nr_points)
	{
		seg.setInputCloud(inputPointCloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			cout << "Could not estimate a planar model for the given dataset." << endl;
			break;
		}
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(inputPointCloud);
		extract.setIndices(inliers);
		extract.setNegative(negative1); //true:平面以外を残す．false:平面を残す

		extract.filter(*cloud_plane);
		//cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << endl; //平面のサイズ

		extract.setNegative(negative2);
		extract.filter(*filtered);
		pcl::copyPointCloud(*filtered, *inputPointCloud);
	}

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(inputPointCloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(tolerance); //単位[m]
	ec.setMinClusterSize(minClusterSize); //最小クラスタの値
	ec.setMaxClusterSize(maxClusterSize); //最大クラスタの値
	ec.setSearchMethod(tree); //検索手法
	ec.setInputCloud(inputPointCloud); //点群を入力
	ec.extract(cluster_indices); //クラスタ情報を出力

	int j = 0; //クラスタのカウント
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>); //クラスタに色付後の点群用
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr max_cluster(new pcl::PointCloud<pcl::PointXYZRGB>); //最大のクラスタを探す(c76)
	max_cluster = cloud_cluster; //最初のクラスタを最大クラスタとする(c76)

	float colors[6][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 255, 0, 255 } }; //クラスタに色を付ける用
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) //クラスタを1塊ごとに出力
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			inputPointCloud->points[*pit].r = colors[j % 6][0];
			inputPointCloud->points[*pit].g = colors[j % 6][1];
			inputPointCloud->points[*pit].b = colors[j % 6][2];
			cloud_cluster->points.push_back(inputPointCloud->points[*pit]);
		}

		//最大のクラスタを探す．元のmac_clusterより新しいクラスタの方が大きければ新しいクラスタをmax_clusterとする(c76)
		if (max_cluster->size() < cloud_cluster->size()){
			pcl::copyPointCloud(*cloud_cluster, *max_cluster);
		}

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		cout << "Cluster " << j << "\t\t\t=>\t" << cloud_cluster->size() << endl;

		j++;
	}

	//int max = cloud_cluster->size();
	//if (j >= 2){ //クラスタが2個以上あれば一番大きいクラスタを出力
		
	//}
	cout << "cloud_cluster => " << cloud_cluster->size() << ", max_cluster => " << max_cluster->size() << endl;
	//pcl::copyPointCloud(*cloud_cluster, *filtered); //カラーリングしたクラスタ全てを出力
	pcl::copyPointCloud(*max_cluster, *filtered); //最大のクラスタのみ出力(c76)
	//cout << "after\tExtract Plane\t\t=>\t" << filtered->size() << endl;
	return filtered;
}
	

/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputModel)
{
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	double theta = M_PI / 8;
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);
	//A translation on Z axis(0.4m)
	transformation_matrix(2, 3) = 0.4;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final;

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(inputCloud);
	icp.setInputTarget(inputModel);
	icp.align()
	//icp.setTransformationEpsilon(1e-6);
	//icp.setMaxCorrespondenceDistance(5.0);
	//icp.setMaximumIterations(200);
	//icp.setEuclideanFitnessEpsilon(1.0);
	//icp.setRANSACOutlierRejectionThreshold(1.0);
	return final;
}*/

Point3f PointCloudMethod::getCentroidCoordinatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "inputPointCloud => " << inputPointCloud->size() << endl;
	FILE *pointcloud;
	FILE *centroid;
	Point3f centroid_coordinate;
	Point3f sum_pointcloud = 0;

	fopen_s(&pointcloud, "pointcloud.dat", "w");
	fopen_s(&centroid, "centroid.dat", "w");

	//cout << *inputPointCloud << endl;
	//summention coordinate. ※inputPointCloud->width = inputPointCloud->size().
	for (int i = 0; i < inputPointCloud->size(); i++){
		//cout << i << " : " << "[x, y, z] => [ " << inputPointCloud->points[i].x << ", " << inputPointCloud->points[i].y << ", " << inputPointCloud->points[i].z << " ] " << endl;
		sum_pointcloud.x = inputPointCloud->points[i].x;
		sum_pointcloud.y = inputPointCloud->points[i].y;
		sum_pointcloud.z = inputPointCloud->points[i].z;

		fprintf_s(pointcloud, "%f %f %f\n",sum_pointcloud.x,sum_pointcloud.y,sum_pointcloud.z);
	}
	centroid_coordinate.x = sum_pointcloud.x / inputPointCloud->size();
	centroid_coordinate.y = sum_pointcloud.y / inputPointCloud->size();
	centroid_coordinate.z = sum_pointcloud.z / inputPointCloud->size();
	cout << "Centroid" << centroid_coordinate << endl;
	fprintf_s(centroid, "%f %f %f\n",centroid_coordinate.x*1000,centroid_coordinate.y*1000,centroid_coordinate.z*1000);
	fclose(pointcloud);
	fclose(centroid);
	return centroid_coordinate;
}
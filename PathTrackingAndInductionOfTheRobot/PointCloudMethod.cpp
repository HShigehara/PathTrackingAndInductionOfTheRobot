/*
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
PointCloudMethod::PointCloudMethod(bool flag_RO, bool flag_DS, bool flag_MLS, bool flag_EP)
{
	//コンストラクタ
	FlagRemoveOutlier = flag_RO;
	FlagDownsampling = flag_DS;
	FlagMLS = flag_MLS;
	FlagExtractPlane = flag_EP;
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

/*!
 * @brief メソッドPointCloudMethod::flagChecker().PCL処理に関する処理の有無を判定するフラグ変数を反転させるメソッド(c64)
 * @param なし
 * @return なし
 */
void PointCloudMethod::flagChecker()
{
	if (GetAsyncKeyState('V')){
		//cout << "Vが入力されたので、外れ値除去処理のフラグを反転" << endl;
		FlagRemoveOutlier = !FlagRemoveOutlier;
	}
	if (GetAsyncKeyState('B')){
		//cout << "Bが入力されたので、ダウンサンプリング処理のフラグを反転" << endl;
		FlagDownsampling = !FlagDownsampling;
	}
	if (/*flag_downsampling == true && */GetAsyncKeyState('N')){
		//cout << "Nが入力されたので、MLS処理のフラグを反転" << endl;
		FlagMLS = !FlagMLS;
	}
	if (GetAsyncKeyState('M')){
		//cout << "Mが入力されたので、平面検出のフラグを反転" << endl;
		FlagExtractPlane = !FlagExtractPlane;
	}
	cout << "外れ値 => " << FlagRemoveOutlier << " ダウンサンプリング => " << FlagDownsampling << " MLS => " << FlagMLS << " 平面検出 => " << FlagExtractPlane << endl;
	return;
}
/*!
 * @brief メソッドPointCloudMethod::passThroughFilter().パススルーフィルター
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before PassThroughFilter => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルター後用のポイントクラウド
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(inputPointCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimitsNegative(true); //setFilterLimits(float min, float max)で指定した範囲以外を削除(c69)
	pass.setFilterLimits(0.3, 40.0);
	pass.filter(*filtered);
	
	cout << "after PassThroughFilter => " << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::removeOutlier()．外れ値を除去するメソッド(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before Remove Outlier => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> fl;
	
	fl.setInputCloud(inputPointCloud);
	fl.setMeanK(50);
	fl.setStddevMulThresh(0.1);
	fl.setNegative(false);
	cout << "TEST" << endl;
	fl.filter(*filtered);

	cout << "after Remove Outlier => " << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::radiusOutlierRemoval()．外れ値を除去するメソッド(c60)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before RadiusOutlierRemoval => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;

	ror.setInputCloud(inputPointCloud);
	cout << "before RadiusOutlierRemoval => " << inputPointCloud->size() << endl;

	ror.setRadiusSearch(0.8);
	ror.setMinNeighborsInRadius(2);
	ror.filter(*filtered);

	cout << "after RadiusOutlierRemoval => " << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::downSamplingUsingVoxelGridFilter()．ダウンサンプリング処理を行うメソッド(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ)
{
	cout << "before Voxel Grid Filter => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(inputPointCloud);
	//sor.setLeafSize()でダウンサンプリングの程度を変更
	vg.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
	//vg.setLeafSize(0.003f, 0.003f, 0.003f); //Default
	//sor.setLeafSize(0.03f, 0.03f, 0.03f); //少ない
	vg.filter(*filtered);

	//ポイントクラウドをしっかり保持できているかサイズを確認
	//cout << "inputSIZE => " << inputPointCloud->size() << endl; //入力したポイントクラウドのサイズ
	cout << "after Voxel Grid Filter => " << filtered->size() << endl; //出力されるポイントクラウドのサイズ
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::smoothingUsingMovingLeastSquare()．スムージングを行うメソッド(c60)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius)
{
	cout << "before MLS => " << inputPointCloud->size() << endl;

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

	cout << "after MLS => " << filtered->size() << endl;
	return filtered;
}

/*!
 * @brief メソッドPointCloudMethod::extractPlane().平面を検出するメソッド
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, double threshold, bool negative)
{
	cout << "before Extract Plane => " << inputPointCloud->size() << endl;

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
	seg.setDistanceThreshold(threshold);

	seg.setInputCloud(inputPointCloud->makeShared());
	seg.segment(*inliers, *coefficients);


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>());
	//int i = 0, nr_points = (int)inputPointCloud->points.size();
	//while (inputPointCloud->points.size() > 0.3*nr_points)
	//{
	//	seg.setInputCloud(inputPointCloud);
	//	seg.segment(*inliers, *coefficients);
	//	if (inliers->indices.size() == 0)
	//	{
	//		cout << "Could not estimate a planar model for the given dataset." << endl;
	//		break;
	//	}
	//	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	//	extract.setInputCloud(inputPointCloud);
	//	extract.setIndices(inliers);
	//	extract.setNegative(false);
	//	extract.filter(*filtered);
	//	extract.setNegative(true);
	//	extract.filter(*cloud_f);
	//	*inputPointCloud = *cloud_f;
	//}

	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//tree->setInputCloud(filtered);
	//vector<pcl::PointIndices> cluster_indices;
	//pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	//ec.setClusterTolerance(0.02); //cm
	//ec.setMinClusterSize(100);
	//ec.setMaxClusterSize(25000);
	//ec.setSearchMethod(tree);
	//ec.setInputCloud(filtered);
	//ec.extract(cluster_indices);

	//int j = 0;
	//for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
	//	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	//	{
	//		cloud_cluster->points.push_back(filtered->points[*pit]);
	//	}
	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;
	//	filtered = cloud_cluster;
	//	j++;
	//}

	for (size_t i = 0; i < inliers->indices.size(); ++i){
		inputPointCloud->points[inliers->indices[i]].r = 255;
		inputPointCloud->points[inliers->indices[i]].g = 0;
		inputPointCloud->points[inliers->indices[i]].b = 0;
		}
	//filtered = inputPointCloud;

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(inputPointCloud/*filtered*/);
	extract.setIndices(inliers);
	extract.setNegative(negative);
	extract.filter(*filtered);
	

	cout << "after Extract Plane => " << filtered->size() << endl;
	return filtered;
}
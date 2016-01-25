/*
 * @file DrawingMethod.cpp
 * @brief 経路をプロットするためのメソッド群
 * @date 2014.10.24
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "Drawing.hpp"

/*!
* @brief メソッドDrawing::Drawing().コンストラクタ
*/
Drawing::Drawing()
{
	if ((gnuplot = _popen("gnuplot", "w")) == NULL){
		cout << "gnuplotが開けません．\"gnuplot/binary\"へパスが通っているか確認して下さい" << endl; //gnuplot/binary/gnuplot.exeを開く.※wgnuplot.exeは起動するが処理が進まず,gnuplotが起動したままになる
	}

	//(c43)
	if ((gpr = _popen("gnuplot", "w")) == NULL){
		cout << "gnuplotが開けません．\"gnuplot/binary\"へパスが通っているか確認して下さい" << endl; //gnuplot/binary/gnuplot.exeを開く.※wgnuplot.exeは起動するが処理が進まず,gnuplotが起動したままになる
	}

	//save_count = 0;
}

/*!
* @brief メソッドDrawing::Drawing().デストラクタ
*/
Drawing::~Drawing()
{
	_pclose(gnuplot);

	//(c43)
	//fprintf_s(gpr, "unset multiplot\n");
	fprintf_s(gpr, "quit\n");
	_pclose(gpr);
}

/*!
 * @brief メソッドDrawing::gnuplotScriptEV3()．EV3の点群をプロットするスクリプトを生成するメソッド(c78)
 * @param save_count int型．データの保存数のカウント
 * @param original_dirpath char*型．基となるディレクトリ名
 * @param output_filename char*型．出力するファイル名
 * @param coefficient_plane Eigen::Vector3f型．平面の係数
 */
void Drawing::gnuplotScriptEV3(int save_count, char* original_dirpath, char* output_filename, Eigen::Vector3f coefficient_plane)
{
	FILE *fp; //ファイルストリームを開く
	char filepath_splot_ev3[NOC];
	sprintf_s(filepath_splot_ev3, "%s/%s-%02d.plt", original_dirpath, output_filename, save_count);
	fopen_s(&fp, filepath_splot_ev3, "w"); //ファイルを開く

	fprintf_s(fp, "set xlabel \"X-axis\"\n");
	fprintf_s(fp, "set ylabel \"Y-axis\"\n");
	fprintf_s(fp, "set zlabel \"Z-axis\"\n");
	fprintf_s(fp, "set title \"PointCloud Plane(LSM) Centroid\"\n");
	fprintf_s(fp, "splot \"6dof-%02d.dat\" pointsize 5,%f*x+%f*y+%f,\"pointcloud-%02d.dat\" every 5\n",save_count, coefficient_plane.x(),coefficient_plane.y(),coefficient_plane.z(), save_count);

	fclose(fp);

	return;
}

/*!
 * @brief メソッドDrawing::gnuplotScriptEV3Route()．EV3の軌道をプロットするためのスクリプト
 * @param original_dirpath char*型．基となるディレクトリ名
 * @param output_filename char*型．出力するファイル名
 */
void Drawing::gnuplotScriptEV3Route(char* original_dirpath, char* output_filename)
{
	FILE *fp;
	char filepath_splot_ev3route[NOC];
	sprintf_s(filepath_splot_ev3route, "%s/%s.plt", original_dirpath, output_filename);
	fopen_s(&fp, filepath_splot_ev3route, "w");

	fprintf_s(fp, "set xlabel \"X-axis\"\n");
	fprintf_s(fp, "set ylabel \"Y-axis\"\n");
	fprintf_s(fp, "set zlabel \"Z-axis\"\n");
	fprintf_s(fp, "set title \"EV3 Centroid Route\"\n");
	fprintf_s(fp, "splot \"ev3route.dat\" with lp\n");

	fclose(fp);

	return;
}

/*!
 * @brief メソッドDrawing::gnuplotScriptTime2V()．時間と速度の関係をプロットするためのスクリプト
 */
void Drawing::gnuplotScriptTime2V()
{
	FILE *fp;
	char filepath_splot_time2v[NOC];
	sprintf_s(filepath_splot_time2v, "data/%s/plot_time-v.plt", directoryName);
	fopen_s(&fp, filepath_splot_time2v, "w");

	fprintf_s(fp, "set xlabel \"Time[s]\"\n");
	fprintf_s(fp, "set ylabel \"v[mm/frame]\"\n");
	fprintf_s(fp, "plot \"time-averagevandyaw.dat\" u 1:2 with lp\n");

	return;
}

/*!
 * @brief メソッドDrawing::gnuplotScriptTime2Yaw()．時間とヨー角の関係をプロットするためのスクリプト
 */
void Drawing::gnuplotScriptTime2Yaw()
{
	FILE *fp;
	char filepath_splot_time2yaw[NOC];
	sprintf_s(filepath_splot_time2yaw, "data/%s/plot_time-yaw.plt", directoryName);
	fopen_s(&fp, filepath_splot_time2yaw, "w");

	fprintf_s(fp, "set xlabel \"Time[s]\"\n");
	fprintf_s(fp, "set ylabel \"Yaw[deg]\"\n");
	fprintf_s(fp, "plot \"time-averagevandyaw.dat\" u 1:3 with lp\n");

	return;
}
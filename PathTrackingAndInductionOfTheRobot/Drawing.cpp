/*
 * @file DrawingMethod.cpp
 * @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
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
 * @brief メソッドDrawing::plot3D().3D座標をプロットするメソッド(未)
 * @param outputDataName 
 */
void Drawing::plot3D(const string* outputDataName)
{
	//出力ファイル名の定義
	char* plotImageName = "plot.jpeg"; //plotしたときの画像ファイル名(c39)
	char* plotXYImageName = "plot_X-Y.jpeg"; //X-Y平面でプロットした画像ファイルの名前(c39)
	char* plotXZImageName = "plot_X-Z.jpeg"; //X-Z平面でプロットした画像ファイルの名前(c39)
	char* plotYZImageName = "plot_Y-Z.jpeg"; //Y-Z平面でプロットした画像ファイルの名前(c39)

	//fprintf_s(gnuplot,"書式指定子",変数);で書式を指定して，gnuplotへ出力できる
	//fputs("コマンド",gnuplot);でコマンドをそのまま出力できる
	fputs("set xlabel \"X-axis\"\n", gnuplot); //X軸のラベル
	fputs("set ylabel \"Y-axis\"\n", gnuplot); //Y軸のラベル
	fputs("set zlabel \"Z-axis\"\n", gnuplot); //Z軸のラベル

	fprintf_s(gnuplot, "splot \"%s/%s\" using 2:3:4 with lp\n", directoryName, outputDataName); //データをプロット
	fputs("set title \"Path\"\n", gnuplot); //グラフのタイトル
	fputs("set term jpeg size 1280,720\n", gnuplot); //jpegで保存するため
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotImageName); //名前をつけて保存
	fputs("rep\n", gnuplot); //画像の保存の反映

	//fputs("set view 1,360,1,1\n", gnuplot); //(X,Y)平面
	fprintf_s(gnuplot, "plot \"%s/%s\" u 2:3 with lp\n", directoryName, outputDataName); //データをプロット
	fputs("set title \"Path X-Y\"\n", gnuplot); //グラフのタイトル
	fputs("set xlabel \"X-axis\"\n", gnuplot); //グラフのX軸ラベル
	fputs("set ylabel \"Y-axis\"\n", gnuplot); //グラフのY軸ラベル
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotXYImageName); //名前をつけて保存
	fputs("rep\n", gnuplot); //画像の保存の反映
	
	//fputs("set view 90,360,1,1\n", gnuplot); //(X,Z)平面
	fprintf_s(gnuplot, "plot \"%s/%s\" u 2:4 with lp\n", directoryName, outputDataName); //データをプロット
	fputs("set title \"Path X-Z\"\n", gnuplot); //グラフのタイトル
	fputs("set xlabel \"X-axis\"\n", gnuplot); //グラフのX軸ラベル
	fputs("set ylabel \"Z-axis\"\n", gnuplot); //グラフのY軸ラベル
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotXZImageName); //名前をつけて保存
	fputs("rep\n", gnuplot); //画像の保存の反映

	//fputs("set view 90,90,1,1\n", gnuplot); //(Y,Z)平面
	fprintf_s(gnuplot, "plot \"%s/%s\" u 3:4 with lp\n", directoryName,outputDataName); //データをプロット
	fputs("set title \"Path Y-Z\"\n", gnuplot); //グラフのタイトル
	fputs("set xlabel \"Y-axis\"\n", gnuplot); //グラフのX軸ラベル
	fputs("set ylabel \"Z-axis\"\n", gnuplot); //グラフのY軸ラベル
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotYZImageName); //名前をつけて保存
	fputs("rep\n", gnuplot); //画像の保存の反映

	//確認用
	//fprintf_s(gnuplot, "splot \"%s/%s\" with points\n", directoryName, outputDataName); //データをプロット
	//fputs("pause 3\n", gnuplot); //プロットした結果をしばらく表示
	
	fputs("quit\n", gnuplot); //gnuplotを終了
	fflush(gnuplot); //コマンドをgnuplotで実行

	return;
}

/*!
* @brief メソッドDrawing::gnuplotScript().後で，3D座標をプロットして確認するためにgnuplotのスクリプトを出力する
* @param dataFileName
*/
void Drawing::gnuplotScript(const string* dataFileName)
{
	//if (checkNum == 1){ //データを保存する場合にスクリプトを生成する(c32)
		FILE *gp; //!<gnuplotスクリプト用のポインタ
		char filePath[NOC]; //!<ファイルのパスを格納する変数
		char* scriptFileName = "splot.plt"; //出力するスクリプトファイルの名前(c39)

		sprintf_s(filePath, "%s/%s", directoryName, scriptFileName); //出力パスを固定
		fopen_s(&gp, filePath, "w"); //ファイルを書き込みモードで開く

		//共通設定
		fprintf_s(gp, "set multiplot layout 2,2\n"); //1つのウインドウに複数のプロットを表示
		fprintf_s(gp, "set grid xtics ytics ztics\n");

		//範囲設定(自動縮尺)
		if (plotMode == 4){
			fprintf_s(gp, "set autoscale\n"); //自動縮尺(c47)
		}

		//Time-X
		fprintf_s(gp, "set title \"Time-X\"\n");
		fprintf_s(gp, "set xlabel \"Time(ms)\"\n");
		fprintf_s(gp, "set ylabel \"X-axis(mm)\"\n");
		if (plotMode == 1){ //(1)中心固定用
			fprintf_s(gp, "set yrange [-5:5]\n"); //中心固定用
		}
		else if (plotMode == 2){ //(2)縦移動用
			fprintf_s(gp, "set yrange [-15:15]\n"); //縦移動用
		}
		else if(plotMode == 3){ //(3)横移動用
			fprintf_s(gp, "set yrange [-60:60]\n"); //横移動用
		}
		fprintf_s(gp, "plot \"%s\" using 1:2 with linespoints\n", dataFileName);

		//Time-Y
		fprintf_s(gp, "set title \"Time-Y\"\n");
		fprintf_s(gp, "set xlabel \"Time(ms)\"\n");
		fprintf_s(gp, "set ylabel \"Y-axis(mm)\"\n");
		if (plotMode == 1){ //(1)中心固定用
			fprintf_s(gp, "set yrange [-10:0]\n"); //中心固定用
		}
		else if (plotMode == 2){ //(2)縦移動用
			fprintf_s(gp, "set yrange [-15:15]\n"); //縦移動用
		}
		else if(plotMode == 3){ //(3)横移動用
			fprintf_s(gp, "set yrange [-15:15]\n"); //横移動用
		}
		fprintf_s(gp, "plot \"%s\" using 1:3 with linespoints\n", dataFileName);

		//Time-Z
		fprintf_s(gp, "set title \"Time-Z\"\n");
		fprintf_s(gp, "set xlabel \"Time(ms)\"\n");
		fprintf_s(gp, "set ylabel \"Z-axis(mm)\"\n");
		if (plotMode == 1){ //(1)中心固定用
			fprintf_s(gp, "set yrange [70:150]\n"); //中心固定用
		}
		else if (plotMode == 2){ //(2)縦移動用
			fprintf_s(gp, "set yrange [150:250]\n"); //縦移動用
		}
		else if(plotMode == 3){ //(3)横移動用
			fprintf_s(gp, "set yrange [50:250]\n"); //横移動用
		}
		fprintf_s(gp, "plot \"%s\" using 1:4 with linespoints\n", dataFileName);

		fprintf_s(gp, "set xlabel \"X-axis(mm)\"\n");
		fprintf_s(gp, "set ylabel \"Y-axis(mm)\"\n");
		fprintf_s(gp, "set zlabel \"Z-axis(mm)\"\n");
		fprintf_s(gp, "set title \"Path\"\n");
		if (plotMode == 1){ //(1)中心固定用
			fprintf_s(gp, "set xrange [-10:10]\n");
			fprintf_s(gp, "set yrange [-10:0]\n");
			fprintf_s(gp, "set zrange [80:120]\n");
		}
		else if (plotMode == 2){ //(2)縦移動用
			fprintf_s(gp, "set xrange [-10:10]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [50:300]\n");
		}
		else if(plotMode == 3){ //(3)横移動用
			fprintf_s(gp, "set xrange [-60:60]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [150:250]\n");
		}
		fprintf_s(gp, "set view equal xy\n");
		//fprintf_s(gp, "splot \"%s\" every 4 using 2:3:4 with linespoints\n", dataFileName); //点を間引いてプロットするとき(c40)
		fprintf_s(gp, "splot \"%s\" using 2:3:4 with linespoints\n", dataFileName); //データをそのままプロット(c40)

		fprintf_s(gp, "unset multiplot\n");

		fclose(gp);
	//}
	return;
}

void Drawing::plot3DRealTime(int countDataNum, outputData outputData[OUTPUTDATA_MAX])
{
	FILE *realtimeplot; //!<リアルタイムに位置をプロットするために利用(c43)
	char pathName[NOC]; //!<ファイルまでのパス名(c43)
	char* fileName = "realplot.dat"; //!<リアルタイムプロット用のデータファイル(c43)

	sprintf_s(pathName, "%s/%s", directoryName, fileName); //データファイルの保存先を指定
	fopen_s(&realtimeplot, pathName, "a"); //ファイルを追記モードで開く(c43)
	fprintf_s(realtimeplot, "%f %f %f %f\n", outputData[countDataNum].totalTime, outputData[countDataNum].x, outputData[countDataNum].y, outputData[countDataNum].z); //ファイルへ出力(c43)
	fclose(realtimeplot); //ファイルを閉じる(c43)

	//プロット開始
	if (first == true){ //1回目は描画の初期設定をする
		fprintf_s(gpr, "set grid\n"); //グリッドを描画
		//fprintf_s(gpr, "unset key\n"); //凡例を消す
		fprintf_s(gpr, "set xlabel \"X-axis\"\n"); //X軸のラベルを設定
		fprintf_s(gpr, "set ylabel \"Y-axis\"\n"); //Y軸のラベルを設定
		fprintf_s(gpr, "set zlabel \"Z-axis\"\n"); //Z軸のラベルを設定

		if (plotMode == 1){ //(1)中心固定用
			fprintf_s(gpr, "set xrange [-10:10]\n");
			fprintf_s(gpr, "set yrange [-20:0]\n");
			fprintf_s(gpr, "set zrange [50:200]\n");
		}
		else if (plotMode == 2){ //(2)縦移動用
			fprintf_s(gpr, "set xrange [-10:10]\n");
			fprintf_s(gpr, "set yrange [-10:10]\n");
			fprintf_s(gpr, "set zrange [50:300]\n");
		}
		else if(plotMode == 3){ //(3)横移動用
			fprintf_s(gpr, "set xrange [-60:60]\n");
			fprintf_s(gpr, "set yrange [-10:10]\n");
			fprintf_s(gpr, "set zrange [150:250]\n");
		}
		else{ //(4)自動縮尺
			fprintf_s(gpr, "set autoscale\n"); //自動縮尺(c47)
		}

		first = false;
	}
	fprintf_s(gpr, "splot \"%s\" using 2:3:4 with linespoints\n", pathName);
	fflush(gpr);

	return;
}

/*!
* @brief メソッドDrawing::gnuplotScript().後で，球の重心座標をプロットするためにgnuplotのスクリプトを出力する
* @param dataFileName
*/
void Drawing::gnuplotScriptCoG(const string* cogFileName)
{
	//if (checkNum == 1){ //データを保存する場合にスクリプトを生成する(c32)
		FILE *gp; //!<gnuplotスクリプト用のポインタ
		char filePath[NOC]; //!<ファイルのパスを格納する変数
		char* scriptFileName = "cog.plt"; //出力するスクリプトファイルの名前(c39)

		sprintf_s(filePath, "%s/%s", directoryName, scriptFileName); //出力パスを固定
		fopen_s(&gp, filePath, "w"); //ファイルを書き込みモードで開く

		fprintf_s(gp, "set xlabel \"X-axis\"\n");
		fprintf_s(gp, "set ylabel \"Y-axis\"\n");
		fprintf_s(gp, "set zlabel \"Z-axis\"\n");
		fprintf_s(gp, "set title \"Path\"\n");
		if (plotMode == 1){ //(1)中心固定用
			fprintf_s(gp, "set xrange [-60:60]\n");
			fprintf_s(gp, "set yrange [-30:30]\n");
			fprintf_s(gp, "set zrange [50:300]\n");
		}
		else if (plotMode == 2){ //(2)縦移動用
			fprintf_s(gp, "set xrange [-10:10]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [50:300]\n");
		}
		else if (plotMode == 3){ //(3)横移動用
			fprintf_s(gp, "set xrange [-60:60]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [150:250]\n");
		}
		else{
			fprintf_s(gp, "set autoscale\n"); //自動縮尺
		}
		fprintf_s(gp, "set view equal xy\n");
		fprintf_s(gp, "splot \"%s\" using 2:3:4 with linespoints\n", cogFileName); //データをそのままプロット(c40)
		fclose(gp);
	//}
	return;
}

/*!
 * @brief メソッドDrawing::gnuplotScriptEV3Unit()．EV3のユニットに関するデータファイルをプロットするスクリプトを生成するメソッド(c78)
 */
void Drawing::gnuplotScriptEV3Unit(Eigen::Vector3f coefficient_plane)
{
	FILE *fp; //ファイルストリームを開く
	char filepath_splot_ev3[NOC];
	sprintf_s(filepath_splot_ev3, "data/%s/%d/splot_ev3-%02d.plt", directoryName, save_count, save_count);
	fopen_s(&fp, filepath_splot_ev3, "w"); //ファイルを開く

	//(c78)
	//if ((splot_ev3unit = _popen("gnuplot", "w")) == NULL){
	//	cout << "gnuplotが開けません．\"gnuplot/binary\"へパスが通っているか確認して下さい" << endl; //gnuplot/binary/gnuplot.exeを開く.※wgnuplot.exeは起動するが処理が進まず,gnuplotが起動したままになる
	//}

	fprintf_s(fp, "set xlabel \"X-axis\"\n");
	fprintf_s(fp, "set ylabel \"Y-axis\"\n");
	fprintf_s(fp, "set zlabel \"Z-axis\"\n");
	fprintf_s(fp, "set title \"PointCloud Plane(LSM) Centroid\"\n");
	fprintf_s(fp, "splot \"dof6-%02d.dat\" pointsize 5,%f*x+%f*y+%f,\"point-%02d.dat\" every 5\n",save_count, coefficient_plane.x(),coefficient_plane.y(),coefficient_plane.z(), save_count);

	//(c78)
	//_pclose(splot_ev3unit);
	fclose(fp);

	return;
}

void Drawing::gnuplotScriptEV3Route()
{
	FILE *fp;
	char filepath_splot_ev3route[NOC];
	sprintf_s(filepath_splot_ev3route, "data/%s/splot_ev3route.plt", directoryName);
	fopen_s(&fp, filepath_splot_ev3route, "w");

	fprintf_s(fp, "set xlabel \"X-axis\"\n");
	fprintf_s(fp, "set ylabel \"Y-axis\"\n");
	fprintf_s(fp, "set zlabel \"Z-axis\"\n");
	fprintf_s(fp, "set title \"EV3 Centroid Route\"\n");
	fprintf_s(fp, "splot \"ev3route.dat\" with lp\n");

	fclose(fp);

	return;
}
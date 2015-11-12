/*
* @file LeastSquareMethod.cpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief 最小二乗法を行うためのメソッド群
* @date 2015.07.16
* @author H.Shigehara
*/

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp" //ヘッダファイルのインクルード
#include "LeastSquareMethod.hpp"

/*!
 * @brief メソッドLeastSquareMethod::LeastSquareMethod().コンストラクタ
 * @param なし
 * @return なし
 */
LeastSquareMethod::LeastSquareMethod()
{
	//コンストラクタ
}

/*!
* @brief メソッドLeastSquareMethod::~LeastSquareMethod().デストラクタ
* @param なし
* @return なし
*/
LeastSquareMethod::~LeastSquareMethod()
{
	//デストラクタ
}

/*!
* @brief メソッドLeastSquareMethod::getSphereData().コンストラクタ
* @param 座標の3次元データ
* @return 球の情報
*/
void LeastSquareMethod::getSphereData(int actualExtractedNum)
{
	//(c51)
	//確認用
	/*cout << "==========================" << endl;
	cout << "actualExtractedNum => " << actualExtractedNum << endl;
	for (int i = 0; i < actualExtractedNum; i++){
		cout << XYZCoordinate[i].x << "    " << XYZCoordinate[i].y << "    " << XYZCoordinate[i].z << endl;
	}*/

	//変数の宣言
	double S1, S2, S3, S4;
	double Sx, Sy, Sz, Sxx, Syy, Szz, Sxy, Sxz, Syz;
	double a, b, c, r;

	//ファイル
	FILE *fp;
	char filePath[NOC];
	sprintf_s(filePath, "%s/%s", directoryName, "cog.dat");
	fopen_s(&fp, filePath, "a");

	//変数の初期化(c51)
	S1 = 0.0; S2 = 0.0; S3 = 0.0; S4 = 0.0;
	Sx = 0.0; Sy = 0.0; Sz = 0.0; Sxx = 0.0; Syy = 0.0; Szz = 0.0; Sxy = 0.0; Sxz = 0.0; Syz = 0.0;
	a = 0.0; b = 0.0; c = 0.0; r = 0.0;

	//各要素の計算(c51)
	for (int i = 0; i < actualExtractedNum; i++){
		Sx = Sx + XYZCoordinate[i].x;
		Sy = Sy + XYZCoordinate[i].y;
		Sz = Sz + XYZCoordinate[i].z;
		Sxx = Sxx + pow(XYZCoordinate[i].x, 2);
		Syy = Syy + pow(XYZCoordinate[i].y, 2);
		Szz = Szz + pow(XYZCoordinate[i].z, 2);
		Sxy = Sxy + XYZCoordinate[i].x * XYZCoordinate[i].y;
		Sxz = Sxz + XYZCoordinate[i].x * XYZCoordinate[i].z;
		Syz = Syz + XYZCoordinate[i].y * XYZCoordinate[i].z;

		S1 = S1 + ((pow(XYZCoordinate[i].x, 3)) + (XYZCoordinate[i].x * pow(XYZCoordinate[i].y, 2)) + (XYZCoordinate[i].x * pow(XYZCoordinate[i].z, 2)));
		S2 = S2 + ((pow(XYZCoordinate[i].x, 2) * XYZCoordinate[i].y) + (pow(XYZCoordinate[i].y, 3)) + (XYZCoordinate[i].y * pow(XYZCoordinate[i].z, 2)));
		S3 = S3 + ((pow(XYZCoordinate[i].x, 2) * XYZCoordinate[i].z) + (pow(XYZCoordinate[i].y, 2) * XYZCoordinate[i].z) + (pow(XYZCoordinate[i].z, 3)));
		S4 = S4 + ((pow(XYZCoordinate[i].x, 2) + pow(XYZCoordinate[i].y, 2) + pow(XYZCoordinate[i].z, 2)));

	}

	S1 = -S1; S2 = -S2; S3 = -S3; S4 = -S4;
	//cout << S1 << endl; //このへんがあやしい
	//cout << Sx << "\t" << Sy << "\t" << Sz << "\t" << Sxx << "\t" << Syy << "\t" << Szz << "\t" << Sxy << "\t" << Sxz << "\t" << Syz << endl;
	//具体的な行列計算(c51)
	Mat m1 = (Mat_<double>(4, 4) << Sxx, Sxy, Sxz, Sx, Sxy, Syy, Syz, Sy, Sxz, Syz, Szz, Sz, Sx, Sy, Sz, actualExtractedNum); //4x4の行列
	Mat m2 = (Mat_<double>(4, 1) << S1, S2, S3, S4); //4x1の行列
	Mat Ans = m1.inv() * m2;

	//cout << m1 << "\n" << m2 << endl;
	a = -0.5 * Ans.at<double>(0, 0);
	//cout << a;
	b = -0.5 * Ans.at<double>(0, 1);
	//cout << b;
	c = -0.5 * Ans.at<double>(0, 2);
	//cout << c;
	r = sqrt((pow(a, 2) + pow(b, 2) + pow(c, 2) - Ans.at<double>(0, 3)));
	//cout << r;

	//cout << "==================================" << endl;
	cout << "a => " << a << "\t" << "b => " << b << "\t" << "c => " << c/* << "\t" << "r => " << r */<< endl;
	fprintf_s(fp, "%f %f %f %f\n", r, a, b, c);
	fclose(fp);

	return;
}
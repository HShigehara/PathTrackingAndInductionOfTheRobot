#コンソール アプリケーション MastersThesis プロジェクトの概要

修士論文研究用のプログラムです。

ファイル構成は以下のようになっています。

### stdafx.h / stdafx.cpp
  * ヘッダファイルのプリコンパイルに必要なファイル

### 3DPathTrackingUsingtheKINECT.hpp
  * プログラム特有のグローバル変数や構造体の宣言ファイル

### main.cpp
  * プログラムのメインとなるファイル

### Mouse.cpp
  * マウス処理を記述したファイル

### KinectMethod.h / KinectMethod.hpp
  * Kinectの操作に必要な処理が記述されたファイル

### System.hpp / System.cpp
  * システム処理が記述されたファイル

### ImageProcessingMethod.hpp / ImageProcessingMethod.cpp
  * 画像処理に関する処理が記述されたファイル

### RouteDrawing.hpp / RouteDrawingMethod.cpp
  * Gnuplotによる軌道描画に関する処理が記述されたファイル

### LeastSquareMethod.hpp / LeastSquareMethod.cpp
  * 最小二乗法に関する処理が記述されたファイル

### PointCloudMethod.hpp / PointCloudMethod.cpp
  * PCL関連の処理をまとめたメソッド

### ChangeLog.txt
  * 変更点を記述したファイル

### ReadMe.txt
  * 本ファイル。

### MastersThesis.vcxproj
  * これは、アプリケーション ウィザードを使用して生成された VC++ プロジェクトのメイン プロジェクト ファイルです。ファイルを生成した Visual C++ のバージョンに関する情報と、アプリケーション ウィザードで選択されたプラットフォーム、構成、およびプロジェクト機能に関する情報が含まれています。

### MastersThesis.vcxproj.filters
  * これは、アプリケーション ウィザードで生成された VC++ プロジェクトのフィルター ファイルです。このファイルには、プロジェクト内のファイルとフィルターとの間の関連付けに関する情報が含まれています。この関連付けは、特定のノードで同様の拡張子を持つファイルのグループ化を示すために IDE で使用されます (たとえば、".cpp" ファイルは "ソース ファイル" フィルターに関連付けられています)。


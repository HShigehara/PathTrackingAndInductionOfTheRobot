/*
 * @file main.cpp
 * @brief Mouseを扱うためのメソッド群
 * @date 2015.05.11
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
 * @brief メソッドonMouse()．マウス左クリックで座標を取得するメソッド
 * @param event int 型．イベント
 * @param x int型．x座標
 * @param y int型．y座標
 * @param flags int型．フラグ
 * @param param void*型．その他パラメータ
 */
void onMouse(int event, int x, int y, int flags, void* param)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = abs(x - origin.x);
		selection.height = abs(y - origin.y);
		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case CV_EVENT_LBUTTONUP:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
		{
			trackObject = -1;
		}
		break;
	}

	return;
}
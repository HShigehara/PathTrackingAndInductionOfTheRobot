/*
* @file OutputDatafile.hpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief �f�[�^���t�@�C���ɏo�͂��邽�߂̃N���X�̃w�b�_
* @date 2015.12.25
* @author H.Shigehara
*/

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "OutputDatafile.hpp"

/*!
 * @brief ���\�b�hOutputDatafile::OutputDatafile()�D�R���X�g���N�^
 */
OutputDatafile::OutputDatafile()
{
	//�R���X�g���N�^

	save_flag = false;
}

/*!
 * @brief ���\�b�hOutputDatafile::~OutputDatafile()�D�f�X�g���N�^
 */
OutputDatafile::~OutputDatafile()
{
	//�f�X�g���N�^
}

/*!
* @brief ���\�b�hSystem::saveDataEveryEnterKey()�D�A���Ōv�����Ă���ۂ�p�L�[����͂���Ƃ��̎��_�̃f�[�^��V�����f�B���N�g���ɕۑ�����D
* @param cv::Mat& current_image, cv::Mat& bin_image, DoF6d dof6, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud
*/
void OutputDatafile::saveDataEveryEnterKey(Mat& current_image, Mat& bin_image, DoF6i dof6, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, double fps)
{
	//���̓s�x�ۑ����邽�߂̃f�B���N�g�����쐬(c82)
	char filepath_output[NOC];
	sprintf_s(filepath_output, "data/%s/%d", directoryName, save_count);
	_mkdir(filepath_output);

	//���݂̉摜��ۑ�
	char filepath_currentimage[NOC];
	sprintf_s(filepath_currentimage, "data/%s/%d/current_image-%02d.jpg", directoryName, save_count, save_count);
	imwrite(filepath_currentimage, current_image);

	//�������v�Z������l�摜��ۑ�
	char filepath_binimage[NOC];
	sprintf_s(filepath_binimage, "data/%s/%d/background_image-%02d.jpg", directoryName, save_count, save_count);
	imwrite(filepath_binimage, bin_image);

	//�_�Q����ۑ�
	FILE *point_fp; //�ŏI1�t���[�����Dgnuplot�ŕ\�����邽�߂ɓ_�Q���t�@�C���ɏo�͂���p
	char filepath_point[NOC];
	sprintf_s(filepath_point, "data/%s/%d/point-%02d.dat", directoryName, save_count, save_count);
	fopen_s(&point_fp, filepath_point, "w"); //
	for (int i = 1; i < inputPointCloud->size(); i++){
		fprintf_s(point_fp, "%f %f %f\n", inputPointCloud->points[i].x, inputPointCloud->points[i].y, inputPointCloud->points[i].z); //�t�@�C���ɏo��
	}
	fclose(point_fp);

	//6DoF����ۑ�
	FILE *dof6_fp; //�ŏI1�t���[�����Dgnuplot�ŕ\�����邽�߂ɕ��ύ��W(�d�S)���t�@�C���ɏo�͂���p
	char filepath_dof6[NOC];
	sprintf_s(filepath_dof6, "data/%s/%d/dof6-%02d.dat", directoryName, save_count, save_count);
	fopen_s(&dof6_fp, filepath_dof6, "w");
	fprintf_s(dof6_fp, "%d %d %d %d %d %d %d\n", dof6.x, dof6.y, dof6.z, dof6.yaw, dof6.roll, dof6.pitch, inputPointCloud->size());
	fclose(dof6_fp);

	//6DoF���𑱂��ĕۑ�����
	FILE *dof6con_fp;
	char filepath_dof6con[NOC];
	sprintf_s(filepath_dof6con, "data/%s/dof6con.csv", directoryName);
	fopen_s(&dof6con_fp, filepath_dof6con, "a");
	if (save_flag == false){
		fprintf(dof6_fp, "x,y,z,Yaw,Roll,Pitch,Data Size,Frame Rate\n");
	}
	fprintf_s(dof6_fp, "%d,%d,%d,%d,%d,%d,%d,%f\n", dof6.x, dof6.y, dof6.z, dof6.yaw, dof6.roll, dof6.pitch, inputPointCloud->size(), fps);
	fclose(dof6con_fp);

	//PointCloud��ۑ�����
	char filepath_pointcloud[NOC];
	sprintf_s(filepath_pointcloud, "data/%s/%d/pointcloud-%02d.ply", directoryName, save_count, save_count);
	pcl::io::savePLYFileASCII(filepath_pointcloud, *inputPointCloud);

	//�������Ԃ�ۑ�����
	//FILE *framerate;
	//char filepath_framerate[NOC];
	//sprintf_s(filepath_framerate, "data/%s/%d/framerate-%02d.dat", directoryName, save_count, save_count);
	//fopen_s(&framerate, filepath_framerate, "w");
	//fprintf_s(framerate, "%f\n", fps);
	//fclose(framerate);

	return;
}

/*!
* @brief ���\�b�hOutputData::saveDataContinuously()�Dp�L�[�����͂��ꂽ��C���ύ��W���o��(c82)
* @@param DoF6d centroid
*/
void OutputDatafile::saveDataContinuously(double sum_time, DoF6i centroid, ControlParamd current)
{
	//�ۑ��p�̃t�@�C���쐬
	FILE *ev3route;
	char filepath_ev3route[NOC];
	sprintf_s(filepath_ev3route, "data/%s/ev3route.dat", directoryName);
	fopen_s(&ev3route, filepath_ev3route, "a"); //�t�@�C���I�[�v��
	fprintf_s(ev3route, "%d %d %d\n", centroid.x, centroid.y, centroid.z); //�f�[�^���t�@�C���ɏ�������
	fclose(ev3route);

	FILE *time_averagevandyaw;
	char filepath_timeaveragevandyaw[NOC];
	sprintf_s(filepath_timeaveragevandyaw, "data/%s/time-averagevandyaw.dat", directoryName);
	fopen_s(&time_averagevandyaw, filepath_timeaveragevandyaw, "a");
	fprintf_s(time_averagevandyaw, "%f %f %f\n", sum_time/1000.0, current.velocity, current.yaw);
	fclose(time_averagevandyaw);

	return;
}


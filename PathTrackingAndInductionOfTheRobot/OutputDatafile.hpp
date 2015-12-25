/*
* @file OutputDatafile.hpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief �f�[�^���t�@�C���ɏo�͂��邽�߂̃N���X�̃w�b�_
* @date 2015.12.25
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __OUTPUTDATAFILE_HPP__
#define __OUTPUTDATAFILE_HPP__

/* �C���N���[�h */
#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class OutputDatafile
* @brief �f�[�^���t�@�C���ɏo�͂��邽�߂̃N���X
*/
class OutputDatafile
{
private:

public:
	OutputDatafile(); //!<�R���X�g���N�^
	~OutputDatafile(); //!<�f�X�g���N�^


	void saveDataEveryEnterKey(Mat& current_image, Mat& bin_image, DoF6i dof6, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, double fps);
	bool save_flag; //!<6DoF�����o�͂���t���O
	void saveDataContinuously(double sum_time, DoF6i centroid, ControlParamd current); //���ύ��W��A���ŕۑ�����
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __OUTPUTDATAFILE_HPP__ */
/*
* @file EV3Control.hpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief EV3�𐧌䂷�邽�߂̃N���X�̃w�b�_
* @date 2015.12.15
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __EV3CONTROL_HPP__
#define __EV3CONTROL_HPP__

#include "PathTrackingAndInductionOfTheRobot.hpp"

/*!
* @class EV3Control
* @brief EV3�𐧌䂷�邽�߂̃N���X
*/
class EV3Control
{
private:
	ControlParamd before1_average; //!<1�t���[���O�̕��ϑ��x�ƕ��σ��[�p
	ControlParamd before2_average; //!<2�t���[���O�̕��ϑ��x�ƕ��σ��[�p
	ControlParamd before3_average; //!<3�t���[���O�̕��ϑ��x�ƕ��σ��[�p
	ControlParamd before4_average; //!<4�t���[���O�̕��ϑ��x�ƕ��σ��[�p
	ControlParamd before5_average; //!<5�t���[���O�̕��ϑ��x�ƕ��σ��[�p

public:
	EV3Control(); //!<�R���X�g���N�^
	~EV3Control(); //!<�f�X�g���N�^

	void set6DoFEV3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle); //!<�ŏ����@�ɂ���ċ��߂����ύ��W�ƈʒu��EV3�̐���̂��߂ɍ\���̂Ɋi�[����(c80)
	DoF6i ev3_6dof; //!<EV3��6���R�x(c80)

	void getVelocityinSec(double time_ms); //!<EV3�̑��x���v�Z����(c85)
	DoF6i before; //!<�O�t���[����6DoF���
	DoF6i current; //!<���t���[����6DoF���
	double velocity; //!<���xv(c85)
	bool flag_velocity; //!<�ŏ���1�t���[���̂��߂̃t���O

	void getAverageVelocityAndYaw(); //!<���ς̑��x�ƃ��[�p���v�Z����
	ControlParamd current_average; //!<���݂̕��ς̑��x�ƃ��[�p
	int count_average; //!<���x�ƃ��[�p�̉ߋ�5�t���[�����̕��ς���邽�߂ɍŏ���5�t���[�����J�E���g���邽�߂̕ϐ�
	bool flag_average; //!<�n�߂�5�t���[�����p

	//�f�[�^�o�̓��\�b�h
	void output6DoF(int save_count, char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud); //!<���t���[����6DoF�����t�@�C���ɏo�͂���
	void output6DoFContinuous(char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud); //!<�L�[����͂����Ƃ���6DoF����A������csv�`���ŕۑ�����
	bool save_flag; //!<6DoF�����o�͂��邩�`�F�b�N���邽�߂̃t���O
	void outputEV3RouteContinuous(char* original_dirpath, char* output_filename); //!<EV3�̑��s�O����ۑ�����
	void outputControlInformation(double sumtime_ms, char* original_dirpath, char* output_filename); //!<EV3�̐�������o�͂���
	void outputControlInformation(); //!<EV3�ɕK�v�ȑ��x�ƃ��[�p���t�@�C���ɏo��
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __EV3CONTROL_HPP__ */

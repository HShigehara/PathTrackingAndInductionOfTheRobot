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

public:
	EV3Control(); //!<�R���X�g���N�^
	~EV3Control(); //!<�f�X�g���N�^

	void set6DoFEV3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle); //!<�ŏ����@�ɂ���ċ��߂����ύ��W�ƈʒu��EV3�̐���̂��߂ɍ\���̂Ɋi�[����(c80)
	DoF6d ev3_6dof; //!<EV3��6���R�x(c80)

	void getVelocity(); //!<EV3�̑��x���v�Z����(c85)
	DoF6d before; //�O�t���[���̏��
	DoF6d current; //���t���[���̏��
	double velocity; //!<���x(c85)
	bool flag_velocity; //�ŏ���1�t���[���̂��߂̃t���O
};
/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __EV3CONTROL_HPP__ */

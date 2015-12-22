/*
* @file EV3Control.cpp
* @link https://github.com/HShigehara/PathTrackingAndInductionOfTheRobot.git
* @brief EV3�𐧌䂷�邽�߂̃��\�b�h�Q
* @date 2015.12.15
* @author H.Shigehara
*/

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "PathTrackingAndInductionOfTheRobot.hpp"
#include "EV3Control.hpp"

/*!
* @brief ���\�b�hEV3Control::EV3Control().�R���X�g���N�^
*/
EV3Control::EV3Control()
{
	//�R���X�g���N�^�͂Ȃ�
	current = { 0 };
	before = { 0 };
	flag_velocity = false; //1�t���[���ڂ̂Ƃ��͏��������Ȃ��悤�ɂ��邽�߂̃t���O(c85)
}

/*!
* @brief ���\�b�hEV3Control::EV3Control().�R���X�g���N�^
*/
EV3Control::~EV3Control()
{
	//�f�X�g���N�^�͂Ȃ�
}

/*!
 * @brief ���\�b�hEV3Control::set6DoFEV3()�D�ŏ����@�ɂ���ċ��߂����ύ��W�ƈʒu��EV3�̐���̂��߂ɍ\���̂Ɋi�[����
 * @param pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle
 */
void EV3Control::set6DoFEV3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Point3d centroid, AttitudeAngle attitude_angle)
{
	//FILE *fp;
	//char filepath_measuredata[NOC];
	//sprintf_s(filepath_measuredata, "data/%s/measuredata.dat", directoryName);
	//fopen_s(&fp, filepath_measuredata, "w");

	ev3_6dof.x = centroid.x;
	ev3_6dof.y = centroid.y;
	ev3_6dof.z = centroid.z;
	ev3_6dof.yaw = attitude_angle.yaw;
	ev3_6dof.roll = attitude_angle.roll;
	ev3_6dof.pitch = attitude_angle.pitch;
	cout << "[X, Y, Z, Yaw, Roll, Pitch, PointCloudNum]" << endl;
	cout << "[ " << ev3_6dof.x << ", " << ev3_6dof.y << ", " << ev3_6dof.z << ", " << ev3_6dof.yaw << ", " << ev3_6dof.roll << ", " << ev3_6dof.pitch << ", " << inputPointCloud->size() << " ]" << endl;

	//fprintf_s(fp, "[\tX\tY\tZ\tYaw\tRoll\tPitch\tPointCloudNum\t]\n");
	//fprintf_s(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z, ev3_6dof.yaw, ev3_6dof.roll, ev3_6dof.pitch, inputPointCloud->size());

	//fclose(fp);
	return;
}

/*!
 * @brief ���\�b�hEV3Control::getVelocity()�DEV3�̑��x���v�Z���郁�\�b�h
 */
void EV3Control::getVelocity()
{
	if (flag_velocity == false){ //1�t���[���ڂ̏���
		before.x = ev3_6dof.x; //���x�̌v�Z�ɕK�v��x���W
		before.z = ev3_6dof.z; //���x�̌v�Z�ɕK�v��z���W
		flag_velocity = true; //�t���O��true�ɂ���
	}
	else{ //2�t���[���ڈȍ~�̏���
		cout << "c_x " << current.x*1000 << ", c_z " << current.z*1000 << ", b_x " << before.x*1000 << ", b_z " << before.z*1000 << endl;
		current.x = ev3_6dof.x; //���݂�x�̒l���i�[
		current.z = ev3_6dof.z; //���݂�z�̒l���i�[
		velocity = sqrt(pow((current.x-before.x),2)+pow((current.z-before.z),2));
		before.x = current.x;
		before.z = current.z;
	}

	cout << "Velocity => " << velocity/1000 << "[mm/frame]" << endl;
	return;
}
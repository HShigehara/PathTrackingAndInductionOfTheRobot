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

	current_average = { 0 };
	before1_average = { 0 };
	before2_average = { 0 };
	before3_average = { 0 };
	before4_average = { 0 };
	before5_average = { 0 };
	count_average = 0;
	flag_average = false;
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

	ev3_6dof.x = round(centroid.x);
	ev3_6dof.y = round(centroid.y);
	ev3_6dof.z = round(centroid.z);
	ev3_6dof.yaw = round(attitude_angle.yaw);
	ev3_6dof.roll = round(attitude_angle.roll);
	ev3_6dof.pitch = round(attitude_angle.pitch);
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
		cout << "c_x " << current.x << ", c_z " << current.z << ", b_x " << before.x << ", b_z " << before.z << endl;
		current.x = ev3_6dof.x; //���݂�x�̒l���i�[
		current.z = ev3_6dof.z; //���݂�z�̒l���i�[
		velocity = sqrt(pow((current.x-before.x),2)+pow((current.z-before.z),2));
		before.x = current.x;
		before.z = current.z;
	}

	cout << "Velocity => " << velocity << "[mm/frame]" << endl;
	return;
}

ControlParamd EV3Control::getAverageVelocityAndYaw()
{
	//�ŏ���5�t���[���̓f�[�^��ۑ����邾��
	if (flag_average == false){ //5�t���[���̊Ԃ̏���
		if (count_average == 0){
			cout << "1�t���[���ڂȂ̂ŕۑ�" << endl;
			before5_average.velocity = velocity;
			before5_average.yaw = ev3_6dof.yaw;
			//current_average.velocity = before5_average.velocity;
			//current_average.yaw = before5_average.yaw;
			count_average++;
		}
		else if (count_average == 1){
			cout << "2�t���[���ڂȂ̂ŕۑ�" << endl;
			before4_average.velocity = velocity;
			before4_average.yaw = ev3_6dof.yaw;
			//current_average.velocity = (before5_average.velocity+before4_average.velocity)/2;
			//current_average.yaw = (before5_average.yaw+before4_average.yaw)/2;
			
			count_average++;
		}
		else if (count_average == 2){
			cout << "3�t���[���ڂȂ̂ŕۑ�" << endl;
			before3_average.velocity = velocity;
			before3_average.yaw = ev3_6dof.yaw;
			//current_average.velocity = (before5_average.velocity+before4_average.velocity+before3_average.velocity)/3;
			//current_average.yaw = (before5_average.yaw+before4_average.yaw+before3_average.yaw)/3;
			count_average++;
		}
		else if (count_average == 3){
			cout << "4�t���[���ڂȂ̂ŕۑ�" << endl;
			before2_average.velocity = velocity;
			before2_average.yaw = ev3_6dof.yaw;
			//current_average.velocity = (before5_average.velocity+before4_average.velocity+before3_average.velocity+before2_average.velocity)/4;
			//current_average.yaw = (before5_average.yaw+before4_average.yaw+before3_average.yaw+before2_average.yaw)/4;
			count_average++;
			//flag_average = true;
		}
		else if (count_average == 4){
			cout << "5�t���[���ڂȂ̂ŕۑ�" << endl;
			before1_average.velocity = velocity;
			before1_average.yaw = ev3_6dof.yaw;
			//current_average.velocity = (before5_average.velocity+before4_average.velocity+before3_average.velocity+before2_average.velocity+before1_average.velocity)/5;
			//current_average.yaw = (before5_average.yaw+before4_average.yaw+before3_average.yaw+before2_average.yaw+before1_average.yaw)/5;
			
			count_average++;
			flag_average = true;
		}
	}
	else{

		//���݂̑��x�̕��ς��v�Z���C1�t���[�������ɂ��炷
		current_average.velocity = (velocity + before1_average.velocity + before2_average.velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 6.0; //���݂Ɖߋ�5�t���[�����̑��x���猻�݂̑��x���v�Z
		before1_average.velocity = current_average.velocity;
		before2_average.velocity = before1_average.velocity;
		before3_average.velocity = before2_average.velocity;
		before4_average.velocity = before3_average.velocity;
		before5_average.velocity = before4_average.velocity;

		//���݂̂�`�p�̕��ς��v�Z���C1�t���[�������ɂ��炷
		current_average.yaw = (ev3_6dof.yaw + before1_average.yaw + before2_average.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 6.0;
		before1_average.yaw = current_average.yaw;
		before2_average.yaw = before1_average.yaw;
		before3_average.yaw = before2_average.yaw;
		before4_average.yaw = before3_average.yaw;
		before5_average.yaw = before4_average.yaw;

		cout << "[ " << current_average.velocity << " , " << current_average.yaw << " ]" << endl;
	}
	return current_average;
}
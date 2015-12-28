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
	current = { 0 }; //�O�t���[����6DoF��������
	before = { 0 }; //���t���[����6DoF��������
	flag_velocity = false; //1�t���[���ڂ̂Ƃ��͏��������Ȃ��悤�ɂ��邽�߂̃t���O(c85)

	current_average = { 0 }; //���t���[���O�̕��ϑ��x�ƕ��σ��[�p�̏�����
	before1_average = { 0 }; //1�t���[���O�̕��ϑ��x�ƕ��σ��[�p�̏�����
	before2_average = { 0 }; //2�t���[���O�̕��ϑ��x�ƕ��σ��[�p�̏�����
	before3_average = { 0 }; //3�t���[���O�̕��ϑ��x�ƕ��σ��[�p�̏�����
	before4_average = { 0 }; //4�t���[���O�̕��ϑ��x�ƕ��σ��[�p�̏�����
	before5_average = { 0 }; //5�t���[���O�̕��ϑ��x�ƕ��σ��[�p�̏�����
	count_average = 0; //���x�ƃ��[�p�̉ߋ�5�t���[�����̕��ς���邽�߂ɍŏ���5�t���[�����J�E���g���邽�߂̕ϐ�
	flag_average = false; //�n�߂�5�t���[�������`�F�b�N���邽�߂̃t���O

	save_flag = false; //6DoF�����o�͂��邩�`�F�b�N���邽�߂̃t���O

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
	ev3_6dof.x = round(centroid.x);
	ev3_6dof.y = round(centroid.y);
	ev3_6dof.z = round(centroid.z);
	ev3_6dof.yaw = round(attitude_angle.yaw);
	ev3_6dof.roll = round(attitude_angle.roll);
	ev3_6dof.pitch = round(attitude_angle.pitch);
	cout << "[X, Y, Z, Yaw, Roll, Pitch, PointCloudNum]" << endl;
	cout << "[ " << ev3_6dof.x << ", " << ev3_6dof.y << ", " << ev3_6dof.z << ", " << ev3_6dof.yaw << ", " << ev3_6dof.roll << ", " << ev3_6dof.pitch << ", " << inputPointCloud->size() << " ]" << endl;

	return;
}

/*!
 * @brief ���\�b�hEV3Control::getVelocity()�DEV3�̑��x���v�Z���郁�\�b�h
 */
void EV3Control::getVelocity()
{
	if (flag_velocity == false){ //1�t���[���ڂ̏���
		before = ev3_6dof; //���x�̌v�Z�ɕK�v�ȍ��W��ۑ�
		flag_velocity = true; //�t���O��true�ɂ���
	}
	else{ //2�t���[���ڈȍ~�̏���
		current = ev3_6dof; //���݂̒l���i�[
		velocity = sqrt(pow((current.x-before.x),2)+pow((current.y-before.y),2)+pow((current.z-before.z),2));
		before = current; //���݂̏����ߋ��̏��ɍX�V����
	}
	cout << "Velocity => " << velocity << "[mm/frame]" << endl; //�m�F�p
	return;
}

void EV3Control::getAverageVelocityAndYaw()
{
	//�ŏ���5�t���[���̓f�[�^��ۑ����邾��
	if (flag_average == false){ //5�t���[���̊Ԃ̏���
		if (count_average == 0){ //1�t���[���ڂ̏���
			current_average.velocity = velocity;
			current_average.yaw = ev3_6dof.yaw;
			before5_average = current_average;
			count_average++;
		}
		else if (count_average == 1){ //2�t���[���ڂ̏���
			current_average.velocity = (velocity + before5_average.velocity) / 2.0;
			current_average.yaw = (ev3_6dof.yaw + before5_average.yaw) / 2.0;
			before4_average = current_average;
			count_average++;
		}
		else if (count_average == 2){ //3�t���[���ڂ̏���
			current_average.velocity = (velocity + before4_average.velocity + before5_average.velocity) / 3.0;
			current_average.yaw = (ev3_6dof.yaw + before4_average.yaw + before5_average.yaw) / 3.0;
			before3_average = current_average;
			count_average++;
		}
		else if (count_average == 3){ //4�t���[���ڂ̏���
			current_average.velocity = (velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 4.0;
			current_average.yaw = (ev3_6dof.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 4.0;
			before2_average = current_average;
			count_average++;
		}
		else if (count_average == 4){ //5�t���[���ڂ̏���
			current_average.velocity = (velocity + before2_average.velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 5.0;
			current_average.yaw = (ev3_6dof.yaw + before2_average.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 5.0;
			before1_average = current_average;
			flag_average = true;
		}
	}
	else{ //6�t���[���ȍ~
		//���݂̑��x�̕��ς��v�Z����
		current_average.velocity = (velocity + before1_average.velocity + before2_average.velocity + before3_average.velocity + before4_average.velocity + before5_average.velocity) / 6.0; //���݂Ɖߋ�5�t���[�����̑��x���猻�݂̑��x���v�Z
		//���݂̃��[�p�̕��ς��v�Z����
		current_average.yaw = (ev3_6dof.yaw + before1_average.yaw + before2_average.yaw + before3_average.yaw + before4_average.yaw + before5_average.yaw) / 6.0;

		//���݂̑��x�ƃ��[�p�̕��ς�1�t���[�������炷
		before1_average = current_average;
		before2_average = before1_average;
		before3_average = before2_average;
		before4_average = before3_average;
		before5_average = before4_average;
	}
	cout << "5 Frame Average [Velocity, Yaw] => " << endl;// << "[ " << round(current_average.velocity) << " , " << round(current_average.yaw) << " ]" << endl;
	cout << current_average.velocity << endl;
	return;
}

/*!
 * @brief ���\�b�hEV3Control::output6Dof()�D���t���[����6DoF�����t�@�C���ɏo�͂���
 */
void EV3Control::output6DoF(int save_count, char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputPointCloud)
{
	//6DoF����ۑ�
	FILE *dof6_fp; //�ŏI1�t���[�����Dgnuplot�ŕ\�����邽�߂ɕ��ύ��W(�d�S)���t�@�C���ɏo�͂���p
	char filepath_dof6[NOC];
	sprintf_s(filepath_dof6, "%s/%s-%02d.dat", original_dirpath, output_filename, save_count);
	fopen_s(&dof6_fp, filepath_dof6, "w");
	fprintf_s(dof6_fp, "%d %d %d %d %d %d %d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z, ev3_6dof.yaw, ev3_6dof.roll, ev3_6dof.pitch, outputPointCloud->size());
	fclose(dof6_fp);

	return;
}

/*!
 * @brief ���\�b�hEV3Control::output6DoFContinuous()�D�L�[����͂����Ƃ���6DoF����A������csv�`���ŕۑ�����
 */
void EV3Control::output6DoFContinuous(char* original_dirpath, char* output_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputPointCloud)
{
	FILE *dof6con_fp;
	char filepath_dof6con[NOC];
	sprintf_s(filepath_dof6con, "%s/%s.csv", original_dirpath, output_filename);
	fopen_s(&dof6con_fp, filepath_dof6con, "a");
	if (save_flag == false){
		fprintf(dof6con_fp, "x,y,z,Yaw,Roll,Pitch,Data Size\n");
	}
	fprintf_s(dof6con_fp, "%d,%d,%d,%d,%d,%d,%d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z, ev3_6dof.yaw, ev3_6dof.roll, ev3_6dof.pitch, outputPointCloud->size());
	fclose(dof6con_fp);

	return;
}

/*!
 * @brief ���\�b�hEV3Control::outputEV3RouteContinuous()�DEV3�̑��s�O����ۑ�����
 */
void EV3Control::outputEV3RouteContinuous(char* original_dirpath, char* output_filename)
{
	//�ۑ��p�̃t�@�C���쐬
	FILE *ev3route;
	char filepath_ev3route[NOC];
	sprintf_s(filepath_ev3route, "%s/%s.dat", original_dirpath, output_filename);
	fopen_s(&ev3route, filepath_ev3route, "a"); //�t�@�C���I�[�v��
	fprintf_s(ev3route, "%d %d %d\n", ev3_6dof.x, ev3_6dof.y, ev3_6dof.z); //�f�[�^���t�@�C���ɏ�������
	fclose(ev3route);

	return;
}

/*!
 * @brief ���\�b�hEV3Control::outputControlInformation()�DEV3�̐�������o�͂���
 */
void EV3Control::outputControlInformation(double sumtime_ms, char* original_dirpath, char* output_filename)
{
	FILE *time_averagevandyaw;
	char filepath_timeaveragevandyaw[NOC];
	sprintf_s(filepath_timeaveragevandyaw, "%s/%s.dat", original_dirpath, output_filename);
	fopen_s(&time_averagevandyaw, filepath_timeaveragevandyaw, "a");
	fprintf_s(time_averagevandyaw, "%f %f %f\n", sumtime_ms / 1000.0, current_average.velocity, current_average.yaw);
	fclose(time_averagevandyaw);

	return;
}
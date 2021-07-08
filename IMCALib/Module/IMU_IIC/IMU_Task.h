#ifndef __IMU_TASK_
#define __IMU_TASK_

#include "mytype.h"


typedef struct{
    float angle; //������yaw���ʵʱ�Ƕ�
    float last_angle; //��һ�νǶ�
    float angle_diff; //ǰ������ֱ������õ��Ĳ�ֵ
    float angle_diff_process; //���������ĽǶȲ�ֵ
    float total_angle; //ת�����ܽǶ�
    int32_t round_cnt; //Ȧ��
    
}IMU_t;


extern IMU_t IMU_Yaw;

void GetImuAngle(IMU_t *IMU);



#endif

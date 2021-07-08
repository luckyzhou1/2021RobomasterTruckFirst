#include "IMU_Task.h"
#include "IOI2C.h"


IMU_t IMU_Yaw; /*������yaw��ṹ��*/


void GetImuAngle(IMU_t *IMU)
{
    IICreadBytes(0x50, AX, 24,&chrTemp[0]); /*ģ��IIC��ȡ����������*/
    
    IMU->last_angle = IMU->angle;
    IMU->angle = (float)CharToShort(&chrTemp[22])/32768*180; /*���IMU��yaw�Ƕ�ֵ����Ӧ�����ǵ�Z��*/
    IMU->angle_diff = IMU->angle - IMU->last_angle; /*ֱ������*/
    if(IMU->angle_diff < -240) /*˳ʱ�룬��180�ȴ���*/
    {
        IMU->round_cnt++;
//        IMU->angle_diff_process = 360 + IMU->angle - IMU->last_angle;   //(angle + 180)+(180 - last_angle)
    }
    else if(IMU->angle_diff > 240)  /*��ʱ�룬��180�ȴ���*/
    {
        IMU->round_cnt--;
//        IMU->angle_diff_process = 360 + IMU->last_angle - IMU->angle;   //(180 - angle)+(180 + last_angle)
    }
//    else
//    {
//        IMU->angle_diff_process = IMU->angle_diff;
//    }
    
    IMU->total_angle = IMU->angle + IMU->round_cnt*360.0f;
//    IMU->total_angle += IMU->angle_diff_process; /*�õ�IMUת���ĽǶ�ֵ*/
    
}


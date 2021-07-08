#include "PrintfInfo.h"
#include "CanBus_Task.h"
#include "RC_Task.h"
#include "stdio.h"
#include "IOI2C.h"


uint8_t Printf_Info;

void PrintfInfo(void)
{
    
    #if defined (REMOTE_MEASURE)    /*��ӡң������ͨ����Ϣ*/
        printf("\n\r The RC ch1.ch2.ch3.ch4 is:%d  %d  %d  %d\n\r",remote_control.ch1,remote_control.ch2,remote_control.ch3,remote_control.ch4);
        printf("\n\r The RC switch_left and switch_right is:%d  %d\n\r",remote_control.switch_left,remote_control.switch_right);
    
    #elif defined (M3508_MOTOR_MEASURE)  /*��ӡ����3508�������Ϣ*/
        printf("\n\r The motor angle and speed_rpm is:%d  %d\n\r", Chassis_Motor[0].angle, Chassis_Motor[0].speed_rpm);  //��ӡ�����ת�ӻ�е�ǶȺ�ת��ת��
        printf("\n\r The motor current is:%f\n\r", Chassis_Motor[0].real_current);  //��ӡ���ת�ص���
        printf("\n\r The motor hall is:%d\n\r", Chassis_Motor[0].hall);  //��ӡ����¶�
        
    #elif defined (M6020_MOTOR_MEASURE)  /*��ӡ��̨6020�������Ϣ*/
        printf("\n\r The motor angle and total_angle is:%d  %d\n\r",Gimbal_Motor[0].angle, Gimbal_Motor[0].total_angle);  //��ӡ�����ת�ӻ�е�ǶȺ�ת��ת��
//        printf("\n\r Motor speed_rpm: %d \n\r", Gimbal_Motor[0].speed_rpm); //ת�����ܽǶ�
//        printf("\n\r The motor current is:%f\n\r",Gimbal_Motor[0].real_current);  //��ӡ���ת�ص���
    
    #elif defined (IMU_MEASURE)
        printf("0x50:   Angle:%.3f %.3f %.3f \r\n",ImuData.Angle[0],ImuData.Angle[1],ImuData.Angle[2]);
    
    #else
        printf("\n\r The offset_angle is:%d\n\r",moto_chassis[0].offset_angle );
        
        printf("\n\r The total_angle is:%d\n\r",moto_chassis[0].total_angle );
    #endif
		
	
	/*ע�����ת�ӻ�е�Ƕ�ֵ��Χ��0 ~ 8192
	      ת��ת��ֵ�ĵ�λΪ��rpm
	      ����¶ȵĵ�λΪ�����϶�
	*/

}


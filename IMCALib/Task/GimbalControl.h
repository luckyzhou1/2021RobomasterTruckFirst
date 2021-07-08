#ifndef __GIMBAL_CTRL
#define __GIMBAL_CTRL

#include "mytype.h"
#include "pid.h"
#include "ramp.h"
#define CHASSIS_YAW_MID_FRONT        7890   //����ǰ��������yaw���������Ӧ����ֵ
#define CHASSIS_YAW_MID_BACK         3794   //���̺���������yaw���������Ӧ����ֵ
#define FAULT_TOLERANT_OFFSET_ANGLE  23     //�ϵ�ʱyaw����̨����ʱ���ݴ�Ƕȣ��������ٶ�

/*��̨������*/
enum{
    YAW   = 0,
    PITCH = 1,
    
};

typedef struct{
    int32_t set_angle;              //�趨�Ƕ�
    int32_t offset_relative_angle;  //�ϵ�ʱyaw������ڵ���ǰ�������ĵ���Ի�е�Ƕ�ֵ
    int32_t relative_angle;         //����ڵ���ǰ�������ĵ���Ի�е�Ƕ�ֵ
    float relative_angle_rad;       //��̨����̵���ԽǶȣ���λ��rad
    float can_send;                 //CAN���͵��ٶȻ�PID���ֵ
    float set_angle_imu;
    
}gimbal_t;
extern char gimbal_view_chasiis_flag;//���ڵ��̷���ı��־λ
extern uint16_t gimbal_delay;//������ʱ
extern int32_t yaw_angle;
extern int32_t pitch_angle;
extern int32_t jscope_relative_angle;
extern uint8_t Gimbal_Init_Flag;  //��̨��ʼ����ʶ
extern int32_t yaw_reset_speed;
extern int32_t pitch_reset_speed;
extern SinRampState  sin_ramp_state; 
extern pid_t Gimbal_Motor_Pid_Pos[2]; //��̨λ�û�PID�ṹ��
extern pid_t Gimbal_Motor_Pid_Spd[2]; //��̨�ٶȻ�PID�ṹ��
extern gimbal_t Gimbal[2]; //��̨�ṹ��

void GimbalControl(void);
/*��̨��ʼ��������*/
void GimbalInit(void);
/*�õ���̨����̵���ԽǶȣ��Ľ���*/
int16_t GetEncoderRelativeAngle_2(void);
#endif

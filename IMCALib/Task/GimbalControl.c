#include "GimbalControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "user_lib.h"
#include "ramp.h"
#include "CatchingTask.h"
#include "ResetTask.h"

uint16_t gimbal_delay = 510;//������ʱ

int32_t yaw_reset_speed;
int32_t pitch_reset_speed;

char yaw_angle_flag;
char pitch_angle_flag;

int32_t yaw_angle;
int32_t pitch_angle;
int32_t jscope_relative_angle;

char pitch_move_flag;
char gimbal_view_chasiis_flag;//���ڵ��̷���ı��־λ

SinRampState  sin_ramp_state; 
gimbal_t Gimbal[2];  //��̨�ṹ��
uint8_t Gimbal_Init_Flag;  //��̨��ʼ����ʶ
int sinsign_yaw=1;//б�º����Ƿ��Ѿ���ʼ���ı�־λ
pid_t Gimbal_Motor_Pid_Pos[2];  //��̨λ�û�PID�ṹ��
pid_t Gimbal_Motor_Pid_Spd[2];  //��̨�ٶȻ�PID�ṹ��

void GimbalControl(void)
{
//    if(RC_UPPER_RIGHT_SW_UP)
//    {
//        yaw_angle = 30000;
//    }
//    else if(RC_UPPER_RIGHT_SW_MID)
//    {
//        yaw_angle = 0;
//    }
//    else if(RC_UPPER_RIGHT_SW_DOWN)
//    {
//        yaw_angle = -30000;
//    }
  
    if(KEY_E && KEY_CTRL && (gimbal_delay > 500))//ͼ��������ǰ���ӽ�
    {
        pitch_angle = reset_pitch_offset_angle -50000;
        yaw_angle = reset_yaw_offset_angle -75000;
        pitch_move_flag = 1;//PITCH�Ƿ��������y����Ʊ�־
        gimbal_view_chasiis_flag = 0;//�����ƶ������־
        gimbal_delay = 0;
    }
    else if(KEY_E && (gimbal_delay > 500))//ͼ������С�����ӽ�
    {
        yaw_angle = reset_yaw_offset_angle - 1000;
        pitch_angle = reset_pitch_offset_angle - 60000;
        pitch_move_flag = 0;
        gimbal_view_chasiis_flag = 1;
        gimbal_delay = 0;//������ʱ
    }
    
    if(pitch_move_flag == 1)//ͼ��������ǰ���ӽ�ʱ��������PITCH��
    {
        pitch_angle += remote_control.mouse.y*5;
    }
    
    /*PITCH����λ*/
    if(pitch_angle > reset_pitch_offset_angle)
    {
        pitch_angle = reset_pitch_offset_angle;
    }
    else if(pitch_angle < reset_pitch_offset_angle - 70000)
    {
        pitch_angle = reset_pitch_offset_angle - 70000;
    }

    if(yaw_reset_flag ==  1)//YAW��λ״̬
    {
         pid_calc(&Gimbal_Motor_Pid_Spd[YAW], Gimbal_Motor[YAW].speed_rpm, yaw_reset_speed);  //YAW���ٶȻ�
    }
    else
    {
        pid_calc(&Gimbal_Motor_Pid_Pos[YAW], Gimbal_Motor[YAW].total_angle, yaw_angle);  //YAW��λ�û�
        pid_calc(&Gimbal_Motor_Pid_Spd[YAW], Gimbal_Motor[YAW].speed_rpm, Gimbal_Motor_Pid_Pos[YAW].pos_out);  //YAW���ٶȻ�
    }
        
    if(pitch_reset_flag ==  1)//PITCH��λ״̬
    {
         pid_calc(&Gimbal_Motor_Pid_Spd[PITCH], Gimbal_Motor[PITCH].speed_rpm, pitch_reset_speed);  //PITCH���ٶȻ�
    }
    else//�Ǹ�λ״̬
    {  
        pid_calc(&Gimbal_Motor_Pid_Pos[PITCH], Gimbal_Motor[PITCH].total_angle, pitch_angle);  //PITCH��λ�û�
        pid_calc(&Gimbal_Motor_Pid_Spd[PITCH], Gimbal_Motor[PITCH].speed_rpm, Gimbal_Motor_Pid_Pos[PITCH].pos_out);  //PITCH���ٶȻ�
    }
    
    Gimbal[YAW].can_send = Gimbal_Motor_Pid_Spd[YAW].pos_out; //YAW��PID��������ֵ��CAN����
    Gimbal[PITCH].can_send = Gimbal_Motor_Pid_Spd[PITCH].pos_out; //YAW��PID��������ֵ��CAN����
    
    SetMotorCurrent1(&hcan2, Moto_Catch_Island_Pid_Spd[0].pos_out, Moto_Catch_Island_Pid_Spd[1].pos_out,  Gimbal[YAW].can_send , Gimbal[PITCH].can_send ); 
     
}

#include "GimbalControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "user_lib.h"
#include "ramp.h"
#include "CatchingTask.h"
#include "ResetTask.h"

uint16_t gimbal_delay = 510;//防误触延时

int32_t yaw_reset_speed;
int32_t pitch_reset_speed;

char yaw_angle_flag;
char pitch_angle_flag;

int32_t yaw_angle;
int32_t pitch_angle;
int32_t jscope_relative_angle;

char pitch_move_flag;
char gimbal_view_chasiis_flag;//用于底盘方向改变标志位

SinRampState  sin_ramp_state; 
gimbal_t Gimbal[2];  //云台结构体
uint8_t Gimbal_Init_Flag;  //云台初始化标识
int sinsign_yaw=1;//斜坡函数是否已经初始化的标志位
pid_t Gimbal_Motor_Pid_Pos[2];  //云台位置环PID结构体
pid_t Gimbal_Motor_Pid_Spd[2];  //云台速度环PID结构体

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
  
    if(KEY_E && KEY_CTRL && (gimbal_delay > 500))//图传处于正前方视角
    {
        pitch_angle = reset_pitch_offset_angle -50000;
        yaw_angle = reset_yaw_offset_angle -75000;
        pitch_move_flag = 1;//PITCH是否能用鼠标y轴控制标志
        gimbal_view_chasiis_flag = 0;//底盘移动方向标志
        gimbal_delay = 0;
    }
    else if(KEY_E && (gimbal_delay > 500))//图传处于小电脑视角
    {
        yaw_angle = reset_yaw_offset_angle - 1000;
        pitch_angle = reset_pitch_offset_angle - 60000;
        pitch_move_flag = 0;
        gimbal_view_chasiis_flag = 1;
        gimbal_delay = 0;//按键延时
    }
    
    if(pitch_move_flag == 1)//图传处于正前方视角时，鼠标控制PITCH轴
    {
        pitch_angle += remote_control.mouse.y*5;
    }
    
    /*PITCH轴限位*/
    if(pitch_angle > reset_pitch_offset_angle)
    {
        pitch_angle = reset_pitch_offset_angle;
    }
    else if(pitch_angle < reset_pitch_offset_angle - 70000)
    {
        pitch_angle = reset_pitch_offset_angle - 70000;
    }

    if(yaw_reset_flag ==  1)//YAW复位状态
    {
         pid_calc(&Gimbal_Motor_Pid_Spd[YAW], Gimbal_Motor[YAW].speed_rpm, yaw_reset_speed);  //YAW轴速度环
    }
    else
    {
        pid_calc(&Gimbal_Motor_Pid_Pos[YAW], Gimbal_Motor[YAW].total_angle, yaw_angle);  //YAW轴位置环
        pid_calc(&Gimbal_Motor_Pid_Spd[YAW], Gimbal_Motor[YAW].speed_rpm, Gimbal_Motor_Pid_Pos[YAW].pos_out);  //YAW轴速度环
    }
        
    if(pitch_reset_flag ==  1)//PITCH复位状态
    {
         pid_calc(&Gimbal_Motor_Pid_Spd[PITCH], Gimbal_Motor[PITCH].speed_rpm, pitch_reset_speed);  //PITCH轴速度环
    }
    else//非复位状态
    {  
        pid_calc(&Gimbal_Motor_Pid_Pos[PITCH], Gimbal_Motor[PITCH].total_angle, pitch_angle);  //PITCH轴位置环
        pid_calc(&Gimbal_Motor_Pid_Spd[PITCH], Gimbal_Motor[PITCH].speed_rpm, Gimbal_Motor_Pid_Pos[PITCH].pos_out);  //PITCH轴速度环
    }
    
    Gimbal[YAW].can_send = Gimbal_Motor_Pid_Spd[YAW].pos_out; //YAW轴PID计算结果赋值给CAN发送
    Gimbal[PITCH].can_send = Gimbal_Motor_Pid_Spd[PITCH].pos_out; //YAW轴PID计算结果赋值给CAN发送
    
    SetMotorCurrent1(&hcan2, Moto_Catch_Island_Pid_Spd[0].pos_out, Moto_Catch_Island_Pid_Spd[1].pos_out,  Gimbal[YAW].can_send , Gimbal[PITCH].can_send ); 
     
}

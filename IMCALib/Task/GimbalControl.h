#ifndef __GIMBAL_CTRL
#define __GIMBAL_CTRL

#include "mytype.h"
#include "pid.h"
#include "ramp.h"
#define CHASSIS_YAW_MID_FRONT        7890   //底盘前面正中央yaw轴编码器对应的数值
#define CHASSIS_YAW_MID_BACK         3794   //底盘后面正中央yaw轴编码器对应的数值
#define FAULT_TOLERANT_OFFSET_ANGLE  23     //上电时yaw轴云台回中时的容错角度，正负多少度

/*云台电机序号*/
enum{
    YAW   = 0,
    PITCH = 1,
    
};

typedef struct{
    int32_t set_angle;              //设定角度
    int32_t offset_relative_angle;  //上电时yaw轴相对于底盘前面正中心的相对机械角度值
    int32_t relative_angle;         //相对于底盘前面正中心的相对机械角度值
    float relative_angle_rad;       //云台与底盘的相对角度，单位：rad
    float can_send;                 //CAN发送的速度环PID输出值
    float set_angle_imu;
    
}gimbal_t;
extern char gimbal_view_chasiis_flag;//用于底盘方向改变标志位
extern uint16_t gimbal_delay;//防误触延时
extern int32_t yaw_angle;
extern int32_t pitch_angle;
extern int32_t jscope_relative_angle;
extern uint8_t Gimbal_Init_Flag;  //云台初始化标识
extern int32_t yaw_reset_speed;
extern int32_t pitch_reset_speed;
extern SinRampState  sin_ramp_state; 
extern pid_t Gimbal_Motor_Pid_Pos[2]; //云台位置环PID结构体
extern pid_t Gimbal_Motor_Pid_Spd[2]; //云台速度环PID结构体
extern gimbal_t Gimbal[2]; //云台结构体

void GimbalControl(void);
/*云台初始化，居中*/
void GimbalInit(void);
/*得到云台与底盘的相对角度，改进版*/
int16_t GetEncoderRelativeAngle_2(void);
#endif

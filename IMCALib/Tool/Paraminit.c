#include "Paraminit.h"
#include "ChassisControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "Ano_Dt.h"
#include "arm_math.h"
#include "stdio.h"
#include "ramp.h"
#include "PrintfInfo.h"
#include "SW_Wave.h"
#include "CatchingTask.h"
#include "GimbalControl.h"

void ParamInit(void)
{
    memset(&Chassis, 0, sizeof(chassis_t));
//    memset(&remote_control, 0, sizeof(RC_Type));
    
    ChassisRampInit(); //底盘斜坡初始化
    
    /*底盘四个电机速度环初始化PID参数*/
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Pos[i], POSITION_PID, 400, 10, 0.02f, 0.0f, 0.0f);  //麦轮悬空时，I = 0.002
    }
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);  //麦轮悬空时，I = 0.002
    }
//    
     /*云台yaw电机PID参数初始化，由电机编码器反馈信息*/
    PID_struct_init(&Gimbal_Motor_Pid_Pos[YAW], POSITION_PID, 8000, 10000, 0.1f, 0.0f, 0.00f); //位置环 
    PID_struct_init(&Gimbal_Motor_Pid_Spd[YAW], POSITION_PID, 30000, 15000, 20.0f, 0.01f, 2.0f); //速度环

    
     /*云台pitch电机PID参数初始化，由电机编码器反馈信息*/
    PID_struct_init(&Gimbal_Motor_Pid_Pos[PITCH], POSITION_PID, 3628, 100, 0.4f, 0.0f, 0.0f); //位置环 
    PID_struct_init(&Gimbal_Motor_Pid_Spd[PITCH], POSITION_PID, 6000, 10000, 13.5f, 0.01f, 0.0f); //速度环
    
//    PID_struct_init(&Moto_Chassis_Pid_Pos[4], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
//    PID_struct_init(&Moto_Chassis_Pid_Spd[4], POSITION_PID,12000, 500, 12.0f, 0.15f, 2.0f);//12000, 5000, 20.0f, 0.f, 0.0f
//    
//		PID_struct_init(&Moto_Chassis_Pid_Pos[5], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
//    PID_struct_init(&Moto_Chassis_Pid_Spd[5], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
//		
//		PID_struct_init(&Moto_Chassis_Pid_Pos[6], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
//    PID_struct_init(&Moto_Chassis_Pid_Spd[6], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
//		
//		PID_struct_init(&Moto_Chassis_Pid_Pos[7], POSITION_PID, 400, 10, 0.01f, 0.0f, 0.0f);
//    PID_struct_init(&Moto_Chassis_Pid_Spd[7], POSITION_PID, 12000, 500, 12.0f, 0.15f, 2.0f);
    
    PID_struct_init(&Moto_Catch_Island_Pid_Pos[0], POSITION_PID, 400, 10, 0.02f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Catch_Island_Pid_Spd[0], POSITION_PID, 12000, 500, 15.0f, 0.15f, 2.0f);
    
    PID_struct_init(&Moto_Catch_Island_Pid_Pos[1], POSITION_PID, 400, 10, 0.02f, 0.0f, 0.0f);
    PID_struct_init(&Moto_Catch_Island_Pid_Spd[1], POSITION_PID, 12000, 500, 15.0f, 0.15f, 2.0f);
    
    PID_struct_init(&Moto_Lift_Island_Pid_Pos[0], POSITION_PID, Maxout_Lift_Island_pos, MaxIlimit_Lift_Island_pos, KP_Lift_Island_pos, KI_Lift_Island_pos, KD_Lift_Island_pos);//资源岛夹取
    PID_struct_init(&Moto_Lift_Island_Pid_Spd[0], POSITION_PID, Maxout_Lift_Island_spd, MaxIlimit_Lift_Island_spd, KP_Lift_Island_spd, KI_Lift_Island_spd, KD_Lift_Island_spd);
    
    PID_struct_init(&Moto_Lift_Island_Pid_Pos[1], POSITION_PID, Maxout_Lift_Island_pos, MaxIlimit_Lift_Island_pos, KP_Lift_Island_pos, KI_Lift_Island_pos, KD_Lift_Island_pos);//资源岛夹取
    PID_struct_init(&Moto_Lift_Island_Pid_Spd[1], POSITION_PID, Maxout_Lift_Island_spd, MaxIlimit_Lift_Island_spd, KP_Lift_Island_spd, KI_Lift_Island_spd, KD_Lift_Island_spd);
    
}

#include "ControlTask.h"
#include "ChassisControl.h"
#include "Paraminit.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "Ano_Dt.h"
#include "arm_math.h"
#include "stdio.h"
#include "ramp.h"
#include "PrintfInfo.h"
#include "SW_Wave.h"
#include "CatchingTask.h"
#include "DataScope_DP.h"
#include "Rescue_Task.h"
#include "GimbalControl.h"
#include "ResetTask.h"
#include "Bigisland_Task.h"
#include "Vision_interact.h"
#include "CatchingCurrent_Task.h"
uint16_t motor_angle;
uint8_t Control_Ctrl;
uint8_t Remote_Control_Mode = 0;

void AllTask(void)
{
    if(Control_Ctrl) //频率1kHz
    {
        Control_Ctrl = 0;
        if(RC_UPPER_LEFT_SW_UP)
        {
            Remote_Control_Mode = 0;//RC控制
        }
        else 
        {
            Remote_Control_Mode = 1;//PC控制
        }
        
        ResetControl();//复位任务  
        GimbalControl();//云台任务
        LiftingIsland();//资源岛抬升任务
        ChassisTask();//底盘任务
//        SwipingCard();//刷卡救援任务
        RescueingControl();//救援任务
        if(catch_mode == 1)
            CatchingIslandControl();//小资源岛夹取任务和兑换任务
        else if(catch_mode == 2)
            BigislandControl();//大资源岛夹取任务
        CatchingCurrent();//夹取机构电流发送
    }
    
    if(vision_send_flag)//发送数据给视觉
    {
        vision_send_flag = 0;
        if(RC_UPPER_RIGHT_SW_UP)
            vision_view_angle_flag = 1;//小屏幕显示1号摄像头视角
        else
            vision_view_angle_flag = 2;//小屏幕显示2号摄像头视角      
        sendVisionData(vision_view_angle_flag);//发送函数
    }
    
    if(Data_Send_ANO_DT) //频率500Hz
    {
        Data_Send_ANO_DT = 0;
        //        motor_angle = Chassis_Motor[4].angle;
        //        ANO_DT_DataUpdate(); /*发送数据到匿名地面站*/
        //        SwDataWaveUpdate(); /*发送数据到山外多功能调试助手进行波形显示*/
        //        PrintfInfo();
        //        printf("\n\r The sin and cos value:%f  , %f  \n\r",arm_sin_f32(PI/6), arm_cos_f32(PI/6));
        //        printf("\n\r %d \n\r", remote_control.ch1);
        //        printf("\n\r Hello Robomaster! \n\r");

        //        sin_test = arm_sin_f32(PI/3);

    }
    
    if(Data_Scope_DP_Send)//MiniBalance无线波形软件
    {
        //		    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
        Data_Scope_DP_Send = 0;
        DataScope();//MiniBalance波形显示
    }

}
/*************************************************PID参数调试函数***********************************************************/
//在线调参时上位机发过来的参数是已经乘以1000的，这里要除以1000
/*速度环PID参数重置*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Spd[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Spd[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Spd[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Spd[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Spd[0].d = kd/1000.0f;
    
}

/*位置环PID参数重置*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Pos[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Pos[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Pos[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Pos[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Pos[0].d = kd/1000.0f;
    
}

/********************************************************END**********************************************************/

//测试
void LEDTest(int flag)
{
    if(flag == 1)
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
}



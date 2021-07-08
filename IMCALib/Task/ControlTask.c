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
    if(Control_Ctrl) //Ƶ��1kHz
    {
        Control_Ctrl = 0;
        if(RC_UPPER_LEFT_SW_UP)
        {
            Remote_Control_Mode = 0;//RC����
        }
        else 
        {
            Remote_Control_Mode = 1;//PC����
        }
        
        ResetControl();//��λ����  
        GimbalControl();//��̨����
        LiftingIsland();//��Դ��̧������
        ChassisTask();//��������
//        SwipingCard();//ˢ����Ԯ����
        RescueingControl();//��Ԯ����
        if(catch_mode == 1)
            CatchingIslandControl();//С��Դ����ȡ����Ͷһ�����
        else if(catch_mode == 2)
            BigislandControl();//����Դ����ȡ����
        CatchingCurrent();//��ȡ������������
    }
    
    if(vision_send_flag)//�������ݸ��Ӿ�
    {
        vision_send_flag = 0;
        if(RC_UPPER_RIGHT_SW_UP)
            vision_view_angle_flag = 1;//С��Ļ��ʾ1������ͷ�ӽ�
        else
            vision_view_angle_flag = 2;//С��Ļ��ʾ2������ͷ�ӽ�      
        sendVisionData(vision_view_angle_flag);//���ͺ���
    }
    
    if(Data_Send_ANO_DT) //Ƶ��500Hz
    {
        Data_Send_ANO_DT = 0;
        //        motor_angle = Chassis_Motor[4].angle;
        //        ANO_DT_DataUpdate(); /*�������ݵ���������վ*/
        //        SwDataWaveUpdate(); /*�������ݵ�ɽ��๦�ܵ������ֽ��в�����ʾ*/
        //        PrintfInfo();
        //        printf("\n\r The sin and cos value:%f  , %f  \n\r",arm_sin_f32(PI/6), arm_cos_f32(PI/6));
        //        printf("\n\r %d \n\r", remote_control.ch1);
        //        printf("\n\r Hello Robomaster! \n\r");

        //        sin_test = arm_sin_f32(PI/3);

    }
    
    if(Data_Scope_DP_Send)//MiniBalance���߲������
    {
        //		    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
        Data_Scope_DP_Send = 0;
        DataScope();//MiniBalance������ʾ
    }

}
/*************************************************PID�������Ժ���***********************************************************/
//���ߵ���ʱ��λ���������Ĳ������Ѿ�����1000�ģ�����Ҫ����1000
/*�ٶȻ�PID��������*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Spd[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Spd[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Spd[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Spd[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Spd[0].d = kd/1000.0f;
    
}

/*λ�û�PID��������*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Moto_Chassis_Pid_Pos[0].MaxOutput = maxout/1000.0f;
    Moto_Chassis_Pid_Pos[0].IntegralLimit = intergral_limit/1000.0f;
    Moto_Chassis_Pid_Pos[0].p = kp/1000.0f;
    Moto_Chassis_Pid_Pos[0].i = ki/1000.0f;
    Moto_Chassis_Pid_Pos[0].d = kd/1000.0f;
    
}

/********************************************************END**********************************************************/

//����
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



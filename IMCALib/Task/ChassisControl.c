/*************************************************************************************************************************
 * @File   ChassisControl.c        
 * @Brief  �����˵��̿��ƣ��ӵ������Ͻǵ����ʱ�뿪ʼ�������̶�Ӧ�����IDΪ1 ~ 4�������ĸ�����Ŀ����ǹ���CAN1�ϣ�
 *         PID��Ӧ�Ĳ������ڴ������Ӹ��ص�����µ��Եõ���
 *
*************************************************************************************************************************/

#include "ChassisControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ramp.h"
#include "stdio.h"
#include "ControlTask.h"
#include "GimbalControl.h"

char chassis_pid_pos = 0;//�Ƿ�ʹ�õ���λ�û���־λ��1ʹ�ã�0��ʹ��
char chassis_angle_init_flag = 0;//����λ�û���ʼ����־
int16_t chassis_pos_delay;//����λ���ƶ���ʱ

int32_t chassis_motor_1_pos_offset_total_angle = 0;
int32_t chassis_motor_2_pos_offset_total_angle = 0;
int32_t chassis_motor_3_pos_offset_total_angle = 0;
int32_t chassis_motor_4_pos_offset_total_angle = 0;

pid_t  Moto_Chassis_Pid_Pos[4];  //CAN1λ�û�PID�ṹ��
pid_t  Moto_Chassis_Pid_Spd[4];  //CAN1�ٶȻ�PID�ṹ��
chassis_t  Chassis;  /*���ڵ���*/

/**********************************J_Scope������ʾ*********************************************/
float J_Scope_rcinput_y;
float J_Scope_rcinput_x;
float Jscope_set_speed;
float Jscope_get_speed;
float Jscope_pid_out;

/***************************************END****************************************************/


void ChassisTask(void)
{
//    ChassisSpeedTest();  //������
    ChassisDataUpdate(); 
    ChassisPidCalc(); 
    ChassisDataCanSend();
    
}



void ChassisDataUpdate(void)
{
    if(Remote_Control_Mode == 1)//PC����
    {
        int16_t chassis_vx_channel = 0, chassis_vy_channel = 0;
        if(gimbal_view_chasiis_flag == 1)//��С��Ļ����
        {
            if(KEY_W)    //��ǰ
            {   
                chassis_vx_channel = CHASSIS_MAXSPEED_RPM;
            }

            else if(KEY_S)    //�˺�
            {
                chassis_vx_channel = -CHASSIS_MAXSPEED_RPM;
            }
            if(KEY_A)    //����
            {
                chassis_vy_channel = CHASSIS_MAXSPEED_RPM;
            }
            else if(KEY_D)    //����
            {
                chassis_vy_channel = -CHASSIS_MAXSPEED_RPM;
            }
            
            if(KEY_W && KEY_CTRL)    //��ǰ
            {   
                chassis_vx_channel = CHASSIS_CATCH_MAXSPEED_RPM;
            }

            else if(KEY_S && KEY_CTRL)    //�˺�
            {
                chassis_vx_channel = -CHASSIS_CATCH_MAXSPEED_RPM;
            }
            if(KEY_A && KEY_CTRL)    //����
            {
                chassis_vy_channel = -CHASSIS_CATCH_MAXSPEED_RPM;
            }
            else if(KEY_D && KEY_CTRL)    //����
            {
                chassis_vy_channel = CHASSIS_CATCH_MAXSPEED_RPM;
            }
            if(KEY_W && KEY_SHIFT)    //��ǰ
            {   
                chassis_vx_channel = CHASSIS_SHIFT_MAXSPEED_RPM;
            }

            else if(KEY_S && KEY_SHIFT)    //�˺�
            {
                chassis_vx_channel = -CHASSIS_SHIFT_MAXSPEED_RPM;
            }
            if(KEY_A && KEY_SHIFT)    //����
            {
                chassis_vy_channel = -CHASSIS_SHIFT_MAXSPEED_RPM;
            }
            else if(KEY_D && KEY_SHIFT)    //����
            {
                chassis_vy_channel = CHASSIS_SHIFT_MAXSPEED_RPM;
            }
        }
        else 
        {
            if(KEY_W)    //��ǰ
            {   
                chassis_vy_channel = CHASSIS_MAXSPEED_RPM;
            }

            else if(KEY_S)    //�˺�
            {
                chassis_vy_channel = -CHASSIS_MAXSPEED_RPM;
            }
            if(KEY_A)    //����
            {
                chassis_vx_channel = -CHASSIS_MAXSPEED_RPM;
            }
            else if(KEY_D)    //����
            {
                chassis_vx_channel = CHASSIS_MAXSPEED_RPM;
            }
            
            if(KEY_W && KEY_CTRL)    //��ǰ
            {   
                chassis_vy_channel = CHASSIS_CATCH_MAXSPEED_RPM;
            }

            else if(KEY_S && KEY_CTRL)    //�˺�
            {
                chassis_vy_channel = -CHASSIS_CATCH_MAXSPEED_RPM;
            }
            if(KEY_A && KEY_CTRL)    //����
            {
                chassis_vx_channel = -CHASSIS_CATCH_MAXSPEED_RPM;
            }
            else if(KEY_D && KEY_CTRL)    //����
            {
                chassis_vx_channel = CHASSIS_CATCH_MAXSPEED_RPM;
            }
            if(KEY_W && KEY_SHIFT)    //��ǰ
            {   
                chassis_vy_channel = CHASSIS_SHIFT_MAXSPEED_RPM;
            }

            else if(KEY_S && KEY_SHIFT)    //�˺�
            {
                chassis_vy_channel = -CHASSIS_SHIFT_MAXSPEED_RPM;
            }
            if(KEY_A && KEY_SHIFT)    //����
            {
                chassis_vx_channel = -CHASSIS_SHIFT_MAXSPEED_RPM;
            }
            else if(KEY_D && KEY_SHIFT)    //����
            {
                chassis_vx_channel = CHASSIS_SHIFT_MAXSPEED_RPM;
            }
        }
    
        FirstOrderFilterCali(&chassis_cmd_slow_set_vx, chassis_vx_channel);
        FirstOrderFilterCali(&chassis_cmd_slow_set_vy, chassis_vy_channel);
        
        if(chassis_vx_channel<0.006&&chassis_vx_channel>(-0.006))
        {
           chassis_cmd_slow_set_vx.out = 0; //����ƽ��
        }
        if(chassis_vy_channel<0.006&&chassis_vy_channel>(-0.006))
        {
           chassis_cmd_slow_set_vy.out = 0; //ǰ���ƶ�
        }
        
        Chassis.vx = chassis_cmd_slow_set_vx.out; //����ƽ��
        Chassis.vy = chassis_cmd_slow_set_vy.out; //ǰ���˶�
  
        Chassis.vw = remote_control.mouse.x;//�����Ƶ�����ת
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
    }
    else if(Remote_Control_Mode == 0)//ң�ؿ���
    {
        int16_t chassis_vx_channel = 0, chassis_vy_channel = 0, chassis_vw_channel = 0;
        /*ң������������*/
        chassis_vx_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;
        chassis_vy_channel = RcDeadlineLimit(remote_control.ch2, 10)*CHASSIS_MAXSPEED_RPM/660;
        chassis_vw_channel = RcDeadlineLimit(remote_control.ch3, 10)*CHASSIS_MAXSPEED_RPM/660;


        //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
        FirstOrderFilterCali(&chassis_cmd_slow_set_vx, chassis_vx_channel);
        FirstOrderFilterCali(&chassis_cmd_slow_set_vy, chassis_vy_channel);

        Chassis.vx = chassis_cmd_slow_set_vx.out; //����ƽ��
        Chassis.vy = chassis_cmd_slow_set_vy.out; //ǰ���˶�
        Chassis.vw = chassis_vw_channel; //��ת
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
        
    }

}

void ChassisPosDataUpdate(int32_t chassis_vx_angle_channel, int32_t chassis_vy_angle_channel, int32_t chassis_vw_angle_channel)
{
       
    if(chassis_angle_init_flag == 0)
    {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        chassis_pos_delay = 0;//λ����ʱ��ʼ
        
        chassis_motor_1_pos_offset_total_angle = Chassis_Motor[FRON_RIGH_201].total_angle;
        chassis_motor_2_pos_offset_total_angle = Chassis_Motor[FRON_LEFT_202].total_angle;
        chassis_motor_3_pos_offset_total_angle = Chassis_Motor[REAR_LEFT_203].total_angle;
        chassis_motor_4_pos_offset_total_angle = Chassis_Motor[REAR_RIGH_204].total_angle;
        
        Chassis.car_vx = chassis_vx_angle_channel; //����ƽ��
        Chassis.car_vy = chassis_vy_angle_channel; //ǰ���˶�
        Chassis.car_vw = chassis_vw_angle_channel; //��ת

        Chassis.fr_motor_angle_201 = +Chassis.car_vx - Chassis.car_vy + Chassis.car_vw + chassis_motor_1_pos_offset_total_angle;
        Chassis.fl_motor_angle_202 = +Chassis.car_vx + Chassis.car_vy + Chassis.car_vw + chassis_motor_2_pos_offset_total_angle;
        Chassis.rl_motor_angle_203 = -Chassis.car_vx + Chassis.car_vy + Chassis.car_vw + chassis_motor_3_pos_offset_total_angle;
        Chassis.rr_motor_angle_204 = -Chassis.car_vx - Chassis.car_vy + Chassis.car_vw + chassis_motor_4_pos_offset_total_angle;
        
        chassis_angle_init_flag = 1;//����λ�ó�ʼ�����
        chassis_pid_pos = 1;//λ�û�ģʽ
    }
   
//    /*�Ƕȼ���*/
//    if(chassis_pid_pos == 1)
//    {
//        Chassis.vx = chassis_vx_angle_channel; //����ƽ��
//        Chassis.vy = chassis_vy_angle_channel; //ǰ���˶�
//        Chassis.vw = chassis_vw_angle_channel; //��ת

//        Chassis.fr_motor_angle_201 = +Chassis.car_vx - Chassis.car_vy + Chassis.car_vw + chassis_motor_1_pos_offset_total_angle;
//        Chassis.fl_motor_angle_202 = +Chassis.car_vx + Chassis.car_vy + Chassis.car_vw + chassis_motor_2_pos_offset_total_angle;
//        Chassis.rl_motor_angle_203 = -Chassis.car_vx + Chassis.car_vy + Chassis.car_vw + chassis_motor_3_pos_offset_total_angle;
//        Chassis.rr_motor_angle_204 = -Chassis.car_vx - Chassis.car_vy + Chassis.car_vw + chassis_motor_4_pos_offset_total_angle;
//        
////        ChassisPidCalc();
////        ChassisDataCanSend();
////        
//    }

}

void ChassisPidCalc(void)
{
    Jscope_set_speed = Chassis.fr_motor_rpm_201;
    Jscope_get_speed = Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2;
    if(chassis_pid_pos == 1)//����˫��PID����
    {
//        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        pid_calc(&Moto_Chassis_Pid_Pos[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].total_angle, Chassis.fr_motor_angle_201);
        pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].speed_rpm, Moto_Chassis_Pid_Pos[FRON_RIGH_201].pos_out*REDUCTION_RATIO_3508);
        
        pid_calc(&Moto_Chassis_Pid_Pos[FRON_LEFT_202], Chassis_Motor[FRON_LEFT_202].total_angle, Chassis.fl_motor_angle_202);
        pid_calc(&Moto_Chassis_Pid_Spd[FRON_LEFT_202], Chassis_Motor[FRON_LEFT_202].speed_rpm, Moto_Chassis_Pid_Pos[FRON_LEFT_202].pos_out*REDUCTION_RATIO_3508);
        
        pid_calc(&Moto_Chassis_Pid_Pos[REAR_LEFT_203], Chassis_Motor[REAR_LEFT_203].total_angle, Chassis.rl_motor_angle_203);
        pid_calc(&Moto_Chassis_Pid_Spd[REAR_LEFT_203], Chassis_Motor[REAR_LEFT_203].speed_rpm, Moto_Chassis_Pid_Pos[REAR_LEFT_203].pos_out*REDUCTION_RATIO_3508);
        
        pid_calc(&Moto_Chassis_Pid_Pos[REAR_RIGH_204], Chassis_Motor[REAR_RIGH_204].total_angle, Chassis.rr_motor_angle_204);
        pid_calc(&Moto_Chassis_Pid_Spd[REAR_RIGH_204], Chassis_Motor[REAR_RIGH_204].speed_rpm, Moto_Chassis_Pid_Pos[REAR_RIGH_204].pos_out*REDUCTION_RATIO_3508);
        
    }
    else//�����ٶȻ�PID����
    {
        pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].speed_rpm, Chassis.fr_motor_rpm_201*REDUCTION_RATIO_3508);
        pid_calc(&Moto_Chassis_Pid_Spd[FRON_LEFT_202], Chassis_Motor[FRON_LEFT_202].speed_rpm, Chassis.fl_motor_rpm_202*REDUCTION_RATIO_3508);
        pid_calc(&Moto_Chassis_Pid_Spd[REAR_LEFT_203], Chassis_Motor[REAR_LEFT_203].speed_rpm, Chassis.rl_motor_rpm_203*REDUCTION_RATIO_3508);
        pid_calc(&Moto_Chassis_Pid_Spd[REAR_RIGH_204], Chassis_Motor[REAR_RIGH_204].speed_rpm, Chassis.rr_motor_rpm_204*REDUCTION_RATIO_3508);
    }
    Jscope_pid_out = Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out;
    
}


void ChassisDataCanSend(void)
{
   
        SetMotorCurrent1(&hcan1, Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out, Moto_Chassis_Pid_Spd[FRON_LEFT_202].pos_out, 
                     Moto_Chassis_Pid_Spd[REAR_LEFT_203].pos_out, Moto_Chassis_Pid_Spd[REAR_RIGH_204].pos_out);
    
}


/************************************************������*************************************************************/
int32_t set_speed_test;

void ChassisSpeedTest(void)
{
    if(RC_UPPER_RIGHT_SW_MID)
    {
        set_speed_test =0;
        
    }
    else if(RC_UPPER_RIGHT_SW_UP)
    {
        set_speed_test = 400;
        
    }
    else if(RC_UPPER_RIGHT_SW_DOWN)
    {
        set_speed_test = -150;
        
    }
//    Jscope_set_speed = set_speed_test;
//    Jscope_get_speed = Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2;
    
     pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].speed_rpm, set_speed_test*REDUCTION_RATIO_3508);
    
//    Jscope_pid_out = Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out;
}






/*************************************************END***************************************************************/




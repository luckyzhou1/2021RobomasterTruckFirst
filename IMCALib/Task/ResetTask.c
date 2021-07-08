#include "ResetTask.h"
#include "RC_Task.h"
#include "CatchingTask.h"
#include "CanBus_Task.h"
#include "GimbalControl.h"
#include "main.h"


char pitch_reset_flag = 0;//PITCH���ٶȻ���λ��־λ
char yaw_reset_flag = 0;//PITCH���ٶȻ���λ��־λ
int32_t reset_pitch_offset_angle;//��λ��PITCH���ʼ�Ƕ�
int32_t reset_yaw_offset_angle;//��λ��YAW���ʼ�Ƕ�

int32_t reset_lift_island_offset_angle;//��λ��̧����ʼ�Ƕ�
char lift_island_reset_success;//��λ�ɹ���־
char lift_island_reset_success_assist;//�˱�־λĿ���ǣ���֤����������δ��λ�ɹ�ʱ������һֱ���븴λ�Ƿ�ɹ����ж�

/*������λ����*/
void ResetControl(void)
{
    if(KEY_CTRL && KEY_R && (catching_island_doing_flag == 1))//��λ
    {
//        catching_island_doing_flag = 0;//�����Ѿ������˶�״̬��������;���½��루�Ƿ���Խ����ȡ��־λ��0���ɽ���1�ɽ���
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//��
        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);          
        //catching_count = 1;
        r_flag = 1;//״̬��־λ��1
//        ialand_catch_reset = 1;//��ȡ����Դ����������λ��ʼ
        lift_island_reset_flag = 1;//̧������Դ����������λ��ʼ
        pitch_reset_flag = 1;//PITCH���ٶȻ���λ��־λ
        yaw_reset_flag = 1;//YAW���ٶȻ���λ��־λ
        lift_island_reset_success = 0;//̧����ʼ��λ����δ�ɹ�
        lift_island_reset_success_assist = 1;
    }
    else if(r_flag == 1)
    {
        /*����������λ�ٶȸ�ֵ*/
        catch_island_reset_speed = -800;
        lift_island_reset_speed = 800;
        pitch_reset_speed = 200;
        yaw_reset_speed = 200;
        
       //��Ϊ��������һ�Σ��൱�ڰ�2�Σ�������ʵ�ָ�λ�����������ж��ǿ��Ե� 
        if((lift_island_reset_speed - Island_Lift_Motor[0].speed_rpm > 398)&&(lift_island_reset_success_assist == 1))
        {       
            change_angle2 = Island_Lift_Motor[0].total_angle;      
            reset_lift_island_offset_angle =  Island_Lift_Motor[0].total_angle;           
            lift_island_reset_flag = 0;//̧������Դ����������λ��������б��
            lift_island_reset_success = 1;//̧����λ�ѳɹ�
            lift_island_reset_success_assist = 0;
        }
         if((pitch_reset_speed - Gimbal_Motor[PITCH].speed_rpm > 190))
        {  
            pitch_angle = Gimbal_Motor[PITCH].total_angle;  
            reset_pitch_offset_angle = Gimbal_Motor[PITCH].total_angle;
            pitch_reset_flag = 0;
        }    
         if((yaw_reset_speed - Gimbal_Motor[YAW].speed_rpm > 190))
        {
            
            yaw_angle = Gimbal_Motor[YAW].total_angle;  
            reset_yaw_offset_angle = Gimbal_Motor[YAW].total_angle;
            yaw_reset_flag = 0;

        }
//         if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)))
//        {          
//            ialand_catch_reset = 0;//��ȡ����Դ����������λ������б��
//            catch_island_reset_speed = 0;
//        }
         if((lift_island_reset_flag == 0)&&(pitch_reset_flag == 0)&&(yaw_reset_flag == 0))//��̨��δװ����ʹ��(yaw_reset_flag == 0)&&(pitch_reset_flag == 0)&&(ialand_catch_reset == 0
        {
//            catching_island_doing_flag = 1;
            r_flag = 0;
        }
    }
}

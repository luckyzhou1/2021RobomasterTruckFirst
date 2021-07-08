#include "Bigisland_Task.h"
#include "CatchingTask.h"
#include "RC_Task.h"
#include "ramp.h"
#include "CanBus_Task.h"
#include "ResetTask.h"
#include "ChassisControl.h"
#include "ControlTask.h"
#include "gpio.h"
#include "GimbalControl.h"
void BigislandControl(void)
{
     ClipControl();//���ӿ���
    
    /*��ȡ���ϵ�һ����ʯ*/
    if(KEY_Z && (catching_island_doing_flag == 1)) //������else if,������if,������CTRL_Zʱ�ᴥ��KEY_Z
    {
        catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
        catching_count = 0;//��ȡ�������
        swap_speed_flag = 1;//���ٶȻ���λ
        z_flag = 1;//�����µ���z��
        catching_count++;
    }
    else if(z_flag == 1)//�ڰ���Z��ȡ״̬
    {
        if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//�ƶ�������ǰ
             catching_count++;
        }
        else if(catching_count == 2)
        {
            /*��ȡ�����ٶȻ���λ*/
            catch_island_reset_speed = -800;
            /*���ж������ӣ�����ܺܿ�����ж�A�����¸�λ���ɹ�c'c'r'r*/
            if(catching_reset_flag == 1)//�ж��Ƿ�ڶ��ν��룬����Ϊ�ڶ���
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//��Ϊ�ڶ��ν��룬���˱�־λ��0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//ȡ���жϵĴ���Ϊ���ж�A
            {       
                in_flag++;
                swap_speed_flag = 0;
//                catching_count++;
                if(in_flag == 1)//��һ�ν������ж�A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//��ȡ��λ��Ҫ�õ���ʱ����֤����ܿ�ڶ��ν��������Ǹ�if���ж�A��
                    catching_reset_flag = 1;//�˱�־λ��1����ʾ�Ѿ������һ�ν��룬�����ڶ��ν����ж�A
                    catching_reset_delay_flag = 1;//�˱�־λ��1���𵽱�����������ʱ�жϵ�����
                }
                else //�ڶ��ν������ж�A
                {
                    in_flag = 0;
                    catching_count++;
                }
                //          r_flag = 0;
                //          catching_island_doing_flag = 1;
            }

        }
        else if(catching_count == 3) //&& (down_up_sign == 0)
        {
            if(sinsign==1)
            {
                SinRampInit(&Sin_Test);
                sinsign = 0;
            }
            Sin_Test.sin_ramp_switch = 1;
            flip_angle = 80000;//�оߴ��ȥ
            
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
             change_angle2 = reset_lift_island_offset_angle - 900000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
             catching_count++;
            catching_island_delay = 0;
        }
        else if(catching_count == 6)
        {
            if((catching_island_delay >=1)&&(catching_island_delay <=4))
            {
                if(sinsign==1)
                { 
                    SinRampInit(&Sin_Test);
                    sinsign=0;
                }
               Sin_Test.sin_ramp_switch = 1;
               flip_angle = -80000;//��ʱ�룬����ת�ӿ�
            }
        }
        else if(catching_count == 7)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
            if((catching_island_delay >=1)&&(catching_island_delay <=3))
            {
                catching_count++;
            }
        }
        else if(catching_count == 8)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);//�ƶ���������˶�
            change_angle2 = reset_lift_island_offset_angle - 700000;
            catching_count++;
            catching_island_delay = 0;
        }
        else if(catching_count == 9)
        {
            
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
                catching_count = 0;
                catching_island_doing_flag = 1;
                z_flag = 0; 
             }
        }
    }
    
    
    
    
/*��ȡ���ϵڶ�����ʯ*/
    else if(KEY_X && (catching_island_doing_flag == 1)) 
    {
        catching_count = 0;
        catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
        swap_speed_flag = 1;//���ٶȻ���λ
        x_flag = 1;//�����µ���x��
        catching_count++;//��ȡ�������
        
    }
    else if(x_flag == 1)
    {
        if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�           
             catching_count++;
        }
        else if(catching_count == 2)
        {
            /*��ȡ�����ٶȻ���λ*/
            catch_island_reset_speed = -800;
            /*���ж������ӣ�����ܺܿ�����ж�A�����¸�λ���ɹ�c'c'r'r*/
            if(catching_reset_flag == 1)//�ж��Ƿ�ڶ��ν��룬����Ϊ�ڶ���
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//��Ϊ�ڶ��ν��룬���˱�־λ��0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//ȡ���жϵĴ���Ϊ���ж�A
            {       
                in_flag++;
                swap_speed_flag = 0;
                if(in_flag == 1)//��һ�ν������ж�A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//��ȡ��λ��Ҫ�õ���ʱ����֤����ܿ�ڶ��ν��������Ǹ�if���ж�A��
                    catching_reset_flag = 1;//�˱�־λ��1����ʾ�Ѿ������һ�ν��룬�����ڶ��ν����ж�A
                    catching_reset_delay_flag = 1;//�˱�־λ��1���𵽱�����������ʱ�жϵ�����
                }
                else //�ڶ��ν������ж�A
                {
                    in_flag = 0;
                    catching_count++;
                }
            }

        }
        else if(catching_count == 3) //&& (down_up_sign == 0)
        {
            catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
            if(sinsign==1)
            {
                SinRampInit(&Sin_Test);
                sinsign = 0;
            }
            Sin_Test.sin_ramp_switch = 1;
            flip_angle = 80000;//˳ʱ�룬����ת�ӿ�
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
                change_angle2 = reset_lift_island_offset_angle - 900000;//̧���������ڴ���Դ����λ�ĵ���Ƕ� 
                catching_count++;
                catching_island_delay = 0;   
        }
        else if(catching_count == 6)
        {
            if((catching_island_delay >=2)&&(catching_island_delay <=4))
            {
                if(sinsign==1)
                { 
                    SinRampInit(&Sin_Test);
                    sinsign=0;
                }
               Sin_Test.sin_ramp_switch = 1;
               flip_angle = -80000;//��ʱ�룬����ת�ӿ�
            }
        }
        else if(catching_count == 7)
        {
            change_angle2 = reset_lift_island_offset_angle - 700000;//̧���������ڴ���Դ����λ�ĵ���Ƕ�
            catching_count = 0;
            catching_island_doing_flag = 1;
            x_flag = 0;
        }
    }
    
    
    
    
/*��ȡ���ϵ�������ʯ*/
    else if(KEY_C && (catching_island_doing_flag == 1))
    {
         catching_count = 0;
         catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
         swap_speed_flag = 1;//���ٶȻ���λ
         c_flag = 1;//�����µ���x��
         catching_count++;//��ȡ�������
    } 
    else if(c_flag == 1)
    {
       if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
//             catching_island_delay = 0;
//             if((catching_island_delay >=1)&&(catching_island_delay <=3))
//             {
             catching_count++;//ע�⣬�������մ˴�ע�͵�������д��������Զ����ִ�д�����䣬��Ϊcatching_island_delay һֱС��1
//             }
        }
        else if(catching_count == 2)
        {
            /*��ȡ�����ٶȻ���λ*/
            catch_island_reset_speed = -800;
            /*���ж������ӣ�����ܺܿ�����ж�A�����¸�λ���ɹ�c'c'r'r*/
            if(catching_reset_flag == 1)//�ж��Ƿ�ڶ��ν��룬����Ϊ�ڶ���
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//��Ϊ�ڶ��ν��룬���˱�־λ��0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//ȡ���жϵĴ���Ϊ���ж�A
            {       
                in_flag++;
                swap_speed_flag = 0;
//                catching_count++;
                if(in_flag == 1)//��һ�ν������ж�A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//��ȡ��λ��Ҫ�õ���ʱ����֤����ܿ�ڶ��ν��������Ǹ�if���ж�A��
                    catching_reset_flag = 1;//�˱�־λ��1����ʾ�Ѿ������һ�ν��룬�����ڶ��ν����ж�A
                    catching_reset_delay_flag = 1;//�˱�־λ��1���𵽱�����������ʱ�жϵ�����
                }
                else //�ڶ��ν������ж�A
                {
                    in_flag = 0;
                    catching_count++;
                }
            }

        }
        if(catching_count == 3) //&& (down_up_sign == 0)
        {
           
            if(sinsign==1)
            {
                SinRampInit(&Sin_Test);
                sinsign = 0;
            }
            Sin_Test.sin_ramp_switch = 1;
            flip_angle = 80000;//˳ʱ�룬����ת�ӿ�      
        
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
            change_angle2 = reset_lift_island_offset_angle - 900000;//̧���������ڴ���Դ����λ�ĵ���Ƕ� 
            catching_count++;
            catching_island_delay = 0;   
        }
        else if(catching_count == 6)
        {
            if((catching_island_delay >=2)&&(catching_island_delay <=4))
            {
                if(sinsign==1)
                { 
                    SinRampInit(&Sin_Test);
                    sinsign=0;
                }
               Sin_Test.sin_ramp_switch = 1;
               flip_angle = -40000;//��ʱ�룬����ת�ӿ�
            }
        }
        else if(catching_count == 7)
        {
            change_angle2 = reset_lift_island_offset_angle - 700000;//̧���������ڴ���Դ����λ�ĵ���Ƕ� 
            catching_island_doing_flag = 1;
            catching_count = 0;
            c_flag = 0;
        }
    }
    
    
   
}

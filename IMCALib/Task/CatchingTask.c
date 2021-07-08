#include "CatchingTask.h"
#include "ramp.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ChassisControl.h"
#include "ControlTask.h"
#include "gpio.h"
#include "GimbalControl.h"
#include "ResetTask.h"
#include "CatchingCurrent_Task.h"

int32_t speed_cha;
int sign;
int sign2;
char a_flag = 0;

SinRampState Sin_Test;
SinRampState Sin_Island_Lift;
int32_t rotate_angle0;//3508��ת�ĽǶ�
int32_t rotate_angle1;
int32_t set_speed; 
int delay_100ms; //��ʱ100ms
int delay_3000ms; //��ʱ3000ms
int sinsign=1;//б�º����Ƿ��Ѿ���ʼ���ı�־λ
int catching_sign=1;//�ж��Ƿ���м�ȡ�ı�־��

int flip_box_delay;//��������ʱ
int flip_box_sign;//�����ӱ�־λ
int Even_Take_Sign=0;//�����ж�����һ���ֽ⶯��

int rotational_delay;
int Rotational_Sign;

/*�¼ӵ�*/
char catch_mode;
int32_t conversion_delay;
char conversion_delay_flag = 0;
int32_t lift_island_reset_speed;
int32_t catch_island_reset_speed;
int32_t change_angle1;//����Դ����ʯ���ü�צ�ĵ�����ƽǶ�
int32_t change_angle2;//����Դ����ʯ����̧�������ĵ�����ƽǶ�
int32_t ccc;
int32_t island_lift_angle;
int32_t flip_angle;//��צ��ת�Ƕ�(��Դ��)
char sin_end_flag = 0;
char island_lifting_angle_flag = 0;//��ʾ̧��������ʱλ�ã�0ʱΪ��λ��1ʱΪ��λ��2Ϊ��λ
char in_flag = 0;//��һ�����Ӹ�λ��Ҫ�õ��ı�־λ
char clamp_level_flag;//����(��Դ��)��ƽ��־λ
char down_up_sign;//̧������λ�ñ�־��downΪ0��upΪ1
char catching_count = 0;
char swap_speed_flag = 0;//�һ�ʱ��צ�Ƿ�ʹ���ٶȻ���־λ
char lift_island_reset_flag = 0;//̧����������Դ������λ��־
char ialand_catch_reset = 0;//��ȡ����(��Դ��)��λ��־
char r_flag = 0;//������־λ 
char f_flag = 0;
char z_flag = 0;
char x_flag = 0;
char c_flag = 0;
char g_flag = 0;
char q_flag = 0;
char ctrl_g_flag = 0;
char ctrl_z_flag = 0;
char ctrl_x_flag = 0;
char ctrl_c_flag = 0;
char ctrl_f_flag = 0;
char ctrl_v_flag = 0;
char shift_z_flag = 0;
char mouse_right_flag = 0;
char big_island_down = 0;//����Դ���½���־λ

uint16_t z_group_set_delay;//z��ϼ�����������ʱ
uint16_t x_group_set_delay;//x��ϼ�����������ʱ
uint16_t c_group_set_delay;//c��ϼ�����������ʱ

uint16_t catching_reset_delay;//��ȡ��λ������ʱ
char catching_reset_flag = 0;//��ȡ��λ������ʱ��־λ
char catching_reset_delay_flag = 0;//��ȡ��λ������ʱ��־λ
uint16_t catching_island_delay;//�е��Ͽ�ʯʱ����������ʱ
char catching_island_doing_flag = 1;//0ʱ�������½����ȡ��1ʱ�������½����ȡ

pid_t  Moto_Lift_Island_Pid_Pos[2];  //��Դ��̧������PIDλ�û��ṹ��
pid_t  Moto_Lift_Island_Pid_Spd[2];  //��Դ��̧������PID�ٶȻ��ṹ��
pid_t  Moto_Catch_Island_Pid_Pos[2];  //��Դ����ȡ����PIDλ�û��ṹ��
pid_t  Moto_Catch_Island_Pid_Spd[2];  //��Դ����ȡ����PID�ٶȻ��ṹ��
/************************J-Scope������ʾ����*************************/
float Jscope_set_angle;
float Jscope_get_angle;
float Jscope_real_speed;

/******************************END***********************************/
/*���Ƽ���*/
void ClipControl(void)
{
    /*���ɼ��Ӻ��ս�����*/
    if(KEY_Q && (catching_island_doing_flag == 1))
    {  
        catching_island_doing_flag = 0;//�Ѿ������ȡ��һ�״̬��������;���½���
        catching_count = 0;//��ȡ��һ��������
        q_flag = 1;//�����µ���ctrl_z��
        catching_count++;
    }
    else if(q_flag == 1)
    {
        if(catching_count == 1) //&& (down_up_sign == 0)
        {
            if(clamp_level_flag == 0)
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);
            else if(clamp_level_flag == 1)
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);
            catching_island_delay = 0;
            catching_count++;
        }     
        else if(catching_count == 2)
        {
            if((catching_island_delay >=2)&&(catching_island_delay <=4))
            {
                catching_count = 0;
                clamp_level_flag = !clamp_level_flag; //�ߵ͵�ƽ��־λȡ��
                catching_island_doing_flag = 1;
                q_flag = 0; 
            }        
       }
    }  
}

void LiftingIsland(void)
{
    if(lift_island_reset_success == 1)
    {
        change_angle2 = reset_lift_island_offset_angle - 100000;
        lift_island_reset_success = 0;
    }    
    if(KEY_CTRL && KEY_F &&  (delay_3000ms == 1))//delay_3000ms����������С��Դ���½�
    {

        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
        change_angle2 = reset_lift_island_offset_angle - 100000;
        catch_mode = 0;
        delay_3000ms = 0;
    }
    
    else if(KEY_F && (delay_3000ms == 1))//С��Դ������
    {
        change_angle2 = reset_lift_island_offset_angle - 2150000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000  ��λ-2200000
        catch_mode = 1;//��ȡС��Դ��ģʽ
        delay_3000ms = 0;
    }
    
    
    if(KEY_G && KEY_CTRL &&  (delay_3000ms == 1))//����Դ���½�
    {  
        catching_count = 0;//�������
        ctrl_g_flag = 1;//�����µ���g��
        catching_count++;
    }
    else if(ctrl_g_flag == 1)
    {
        if(catching_count == 1) 
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//��
             HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
             catching_count++;
             catching_island_delay = 0; 
        }     
        else if(catching_count == 2)
        {
            if((catching_island_delay >=2)&&(catching_island_delay <=5))
            {
                change_angle2 = reset_lift_island_offset_angle - 100000;
                catching_count = 0;
                ctrl_g_flag = 0;
                delay_3000ms = 0;
            }        
       }
    }
    
    else if(KEY_G && (delay_3000ms == 1))//����Դ��̧��
    {  
        catching_count = 0;//�������
        g_flag = 1;//�����µ���g��
        catching_count++;
    }
    else if(g_flag == 1)
    {
        if(catching_count == 1) 
        {
            change_angle2 = reset_lift_island_offset_angle - 700000;//̧���������ڴ���Դ����λ�ĵ���Ƕ� 
            catching_island_delay = 0;
            catching_count++;
        }     
        else if(catching_count == 2)
        {
            if((catching_island_delay >=2)&&(catching_island_delay <=5))
            {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//��
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);  
                catching_count++;
            }        
       }
       else if(catching_count == 3)
       {
            catch_mode = 2;//��ȡ����Դ��ģʽ
            catching_count = 0;
            g_flag = 0;
            delay_3000ms = 0;
       }
    }

    if(lift_island_reset_flag == 1) 
    {
        pid_calc(&Moto_Lift_Island_Pid_Spd[0], Island_Lift_Motor[0].speed_rpm, lift_island_reset_speed);
        pid_calc(&Moto_Lift_Island_Pid_Spd[1], Island_Lift_Motor[1].speed_rpm, lift_island_reset_speed);
    }
    else
    {       
        pid_calc(&Moto_Lift_Island_Pid_Pos[0], Island_Lift_Motor[0].total_angle, change_angle2);
        pid_calc(&Moto_Lift_Island_Pid_Spd[0], Island_Lift_Motor[0].speed_rpm, Moto_Lift_Island_Pid_Pos[0].pos_out*REDUCTION_RATIO_3508);
        
        pid_calc(&Moto_Lift_Island_Pid_Pos[1], Island_Lift_Motor[1].total_angle, change_angle2);
        pid_calc(&Moto_Lift_Island_Pid_Spd[1], Island_Lift_Motor[1].speed_rpm, Moto_Lift_Island_Pid_Pos[1].pos_out*REDUCTION_RATIO_3508);
    }
    SetMotorCurrent2(&hcan1, Moto_Lift_Island_Pid_Spd[0].pos_out, Moto_Lift_Island_Pid_Spd[1].pos_out, 0, 0); 
     
}

void CatchingIslandControl(void)
{
  
    ClipControl();//���ӿ���
    
    /*С��Դ������ȡ*/
    if(KEY_SHIFT && KEY_CTRL && (catching_island_doing_flag == 1)&&(z_group_set_delay > 500))
    {
         catching_island_doing_flag = 0;//�Ѿ������ȡ��һ�״̬��������;���½���
         catching_count = 0;//��ȡ��һ��������
         swap_speed_flag = 1;//�����ٶȻ���λ
         shift_z_flag = 1;
         catching_count++;
        z_group_set_delay = 0;
    }
    else if(shift_z_flag == 1)
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
        else if(catching_count == 3) 
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
             change_angle2 = reset_lift_island_offset_angle - 2300000;//̧����������С��Դ����λ�ĵ���Ƕ� 
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
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
            if((catching_island_delay >=1)&&(catching_island_delay <=3))
            {
                catching_count++;
            }
        }
        else if(catching_count == 8)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);//�ƶ���������˶�
            change_angle2 = reset_lift_island_offset_angle - 2150000;
            catching_count++;          
        }
        else if(catching_count == 9)
        {    
            ChassisPosDataUpdate(8192,0,0);            
            catching_count++;
        }
        else if(catching_count == 10)
        {
            if(chassis_pos_delay > 5)
            {
                chassis_angle_init_flag = 0;//��ʼ����ԭ
                chassis_pid_pos = 0;//�ٶȻ�ģʽ
                catching_count++;
            }
        }
        else if(catching_count == 11) //&& (down_up_sign == 0)
        {         
            if(sinsign==1)
            {
                SinRampInit(&Sin_Test);
                sinsign = 0;
            }
            Sin_Test.sin_ramp_switch = 1;
            flip_angle = 80000;//˳ʱ�룬����ת�ӿ�
        }
        else if(catching_count == 12)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 13)
        {
                change_angle2 = reset_lift_island_offset_angle - 2300000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
                catching_count++;
                catching_island_delay = 0;   
        }
        else if(catching_count == 14)
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
        else if(catching_count == 15)
        {
            change_angle2 = reset_lift_island_offset_angle - 2150000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
            catching_count++;

        }
        else if(catching_count == 16)
        {
            ChassisPosDataUpdate(8192,0,0);
//            ChassisPidCalc();
//            ChassisDataCanSend();
            catching_count++;
        }
        else if(catching_count == 17)
        {
            if(chassis_pos_delay > 2)
            {
                chassis_angle_init_flag = 0;//��ʼ����ԭ
                chassis_pid_pos = 0;//�ٶȻ�ģʽ
                catching_count++;
            }
        }
        else if(catching_count == 18)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
             catching_island_delay = 0;
//             if((catching_island_delay >=1)&&(catching_island_delay <=3))
//             {
             catching_count++;//ע�⣬�������մ˴�ע�͵�������д��������Զ����ִ�д�����䣬��Ϊcatching_island_delay һֱС��1
//             }
        }
        if(catching_count == 19) //&& (down_up_sign == 0)
        {
            if((catching_island_delay >=1)&&(catching_island_delay <=3))
            {
                if(sinsign==1)
                {
                    SinRampInit(&Sin_Test);
                    sinsign = 0;
                }
                Sin_Test.sin_ramp_switch = 1;
                flip_angle = 80000;//˳ʱ�룬����ת�ӿ�      
            }
        }
        else if(catching_count == 20)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
       else if(catching_count == 21)
       {
           change_angle2 = reset_lift_island_offset_angle - 2300000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
           catching_count++;
           catching_island_delay = 0;
       }
        else if(catching_count == 22)
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
        else if(catching_count == 23)
        {
            change_angle2 = reset_lift_island_offset_angle - 2150000;//̧����������С��Դ����λ�ĵ���Ƕ� 
            catching_island_doing_flag = 1;
            shift_z_flag = 0;
            catching_count = 0;
        }
    }
    
    
    
    
    
/*�һ����ϵ�һ����ʯ*/
    if(KEY_CTRL && KEY_Z && (catching_island_doing_flag == 1))
    {
        catching_island_doing_flag = 0;//�Ѿ������ȡ��һ�״̬��������;���½���
        catching_count = 0;//��ȡ��һ��������
        swap_speed_flag = 1;//�����ٶȻ���λ
        ctrl_z_flag = 1;//�����µ���ctrl_z��
        catching_count++;
    }
    else if(ctrl_z_flag == 1)//�ڰ���CTRL_Z״̬
    {
         if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//�ƶ�������ǰ
             catching_count++;
        }
 
        if(catching_count == 2) //&& (down_up_sign == 0)
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
                    catch_island_reset_speed = 0;
                    catching_count++;  
                }
            }
        }
        else if(catching_count == 3)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             conversion_delay = 0;
             catching_count++;//Ҫд�����������д����һ������
        }
        else if(catching_count == 4)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);          
            conversion_delay_flag = 1;
           if((conversion_delay > 1000)&&(conversion_delay_flag == 1))
           {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
               conversion_delay_flag = 0;
                catching_count++;
            }
     
        }
        if(catching_count == 5) //&& (down_up_sign == 0)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//�Ƹ˻���
            catching_island_delay = 0;
            catching_count++;
        }
        if(catching_count == 6) //&& (down_up_sign == 0)
        {  
            if((catching_island_delay >=2)&&(catching_island_delay <=4))
            {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�,�ߵ�ƽ�˶�
                catching_island_delay = 0;
                catching_count++;
            }
        }
        else if(catching_count == 7)
        {
                if(sinsign==1)
                { 
                    SinRampInit(&Sin_Test);
                    sinsign=0;
                }
                Sin_Test.sin_ramp_switch = 1;
                flip_angle = 80000;//���Ӵ��ȥ;
        }
        else if(catching_count == 8)
        {
             conversion_delay = 0;
             catching_count++;//Ҫд�����������д����һ������
        }
        else if(catching_count == 9)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);          
            conversion_delay_flag = 1;
           if((conversion_delay > 300)&&(conversion_delay_flag == 1))
           {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
                conversion_delay_flag = 0;
                catching_count++;
            }
     
        }
        else if(catching_count == 10)
        {
            catching_count = 0;
            catching_island_doing_flag = 1;
            ctrl_z_flag = 0; 
        }
    }
    
    
    
/*��ȡ���ϵ�һ����ʯ*/
    else if( KEY_Z && (catching_island_doing_flag == 1) ) //������else if,������if,������CTRL_Zʱ�ᴥ��KEY_Z
    {
        catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
        catching_count = 0;//��ȡ�������
        swap_speed_flag = 1;//���ٶȻ���λ
        z_flag = 1;//�����µ���z��
        catching_count++;
    }
    else if(z_flag == 1)//�ڰ���Z��ȡ״̬
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
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
        else if(catching_count == 3) 
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
             change_angle2 = reset_lift_island_offset_angle - 2300000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
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
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
            if((catching_island_delay >=1)&&(catching_island_delay <=3))
            {
                catching_count++;
            }
        }
        else if(catching_count == 8)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);//�ƶ���������˶�
            change_angle2 = reset_lift_island_offset_angle - 2150000;
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
  
    
/*�һ���ʯ�ڶ�����*/    
    else if(remote_control.mouse.press_right && (catching_island_doing_flag == 1))//remote_control.mouse.press_left
    {
         catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
         catching_count = 0;//��ȡ�������
         mouse_right_flag = 1;
         catching_count++;
    }
    else if(mouse_right_flag == 1)
    {
        if(catching_count == 1)
        {
            conversion_delay = 0;
            catching_count++;//Ҫд�����������д����һ������

        }
        else if(catching_count == 2)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);          
            conversion_delay_flag = 1;
           if((conversion_delay > 1000)&&(conversion_delay_flag == 1))
           {
                conversion_delay_flag = 0;
                conversion_delay = 0;
                catching_count++;
           }              
        }
              
        else if(catching_count == 3)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
             catching_island_delay = 0;     
             catching_count++;            
        }
        else if(catching_count == 4)
        {
             if((catching_island_delay >=2)&&(catching_island_delay <=5))
             {
                 HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
                 conversion_delay = 0;
                 catching_count++;  
             }
        }
        else if(catching_count == 5)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);          
            conversion_delay_flag = 1;
            if((conversion_delay > 1000)&&(conversion_delay_flag == 1))
            {
                conversion_delay_flag = 0;
                conversion_delay = 0;
                catching_count++;
            }            
        }
        else if(catching_count == 6)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
            catching_island_delay = 0;
            catching_count++;
        }
        else if(catching_count == 7)
        {
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
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
        else if(catching_count == 8)
        {         
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
            catching_count = 0;
            catching_island_doing_flag = 1;
            mouse_right_flag = 0; 
        }
        
    }
    
    
    
/*�һ����ϵڶ�����ʯ*/
    if(KEY_CTRL && KEY_X && (catching_island_doing_flag == 1))
    {
       catching_island_doing_flag = 0;//�Ѿ������ȡ��һ�״̬��������;���½���
       catching_count = 0;//��ȡ��һ��������
       ctrl_x_flag = 1;//�����µ���ctrl_x��
       swap_speed_flag = 1;//�����ٶȻ���λ
       catching_count++;
    }
    else if(ctrl_x_flag == 1)//�ڰ���CTRL_X״̬
    {
        
        if(catching_count == 1)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//���ɼо�
            catching_count++;
        }
        if(catching_count == 2)
        {
            catch_island_reset_speed = -800;
            /*���ж������ӣ�����ܺܿ�����ж�A�����¸�λ���ɹ�c'c'r'r*/
            if(catching_reset_flag == 1)//�ж��Ƿ�ڶ��ν��룬����Ϊ�ڶ���
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//��Ϊ�ڶ��ν��룬���˱�־λ��0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//���ţ��ж�A
            {
               
                speed_cha = catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm;
                in_flag++;
                swap_speed_flag = 0;
                if(in_flag == 1)//��һ�ν����ж�A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//��ȡ��λ��Ҫ�õ���ʱ����֤����ܿ�ڶ��ν��������Ǹ�if���ж�A��
                    catching_reset_flag = 1;//�˱�־λ��1����ʾ�Ѿ������һ�ν��룬�����ڶ��ν����ж�A
                    catching_reset_delay_flag = 1;//�˱�־λ��1���𵽱�����������ʱ�жϵ�����
                }
                else //�ڶ��ν����ж�A
                {
                    in_flag = 0;
                    catching_count++;
                }
            }
         }        
        else if(catching_count == 3)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
             conversion_delay = 0;
             catching_count++;//Ҫд�����������д����һ������
        }
        else if(catching_count == 4)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);          
            conversion_delay_flag = 1;
           if((conversion_delay > 1000)&&(conversion_delay_flag == 1))
           {
//                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
               conversion_delay_flag = 0;
                catching_count++;
            }
     
        }
        if(catching_count == 5) //&& (down_up_sign == 0)
        {
            catching_island_delay = 0;
            catching_count++;
        }
        if(catching_count == 6) //&& (down_up_sign == 0)
        {  
            if((catching_island_delay >=2)&&(catching_island_delay <=4))
            {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//�н��о�
                catching_island_delay = 0;
                catching_count++;
            }
        }
        else if(catching_count == 7)
        {
                if(sinsign==1)
                { 
                    SinRampInit(&Sin_Test);
                    sinsign=0;
                }
                Sin_Test.sin_ramp_switch = 1;
                flip_angle = 80000;//���Ӵ��ȥ;
        }
        else if(catching_count == 8)
        {
             conversion_delay = 0;
             catching_count++;//Ҫд�����������д����һ������
        }
        else if(catching_count == 9)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);          
            conversion_delay_flag = 1;
           if((conversion_delay > 300)&&(conversion_delay_flag == 1))
           {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
                conversion_delay_flag = 0;
                catching_count++;
           }
     
        }
        else if(catching_count == 10)
        {
 
            catching_count = 0;
            catching_island_doing_flag = 1;
            ctrl_x_flag = 0; 
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
                change_angle2 = reset_lift_island_offset_angle - 2300000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
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
            change_angle2 = reset_lift_island_offset_angle - 2150000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
            catching_count = 0;
            catching_island_doing_flag = 1;
            x_flag = 0;
        }
    }

    
    
    
    
/*�һ����ϼ�ȡ�ĵ�������ʯ*/
    else if(KEY_CTRL && KEY_C && (catching_island_doing_flag == 1))
    {
         catching_count = 0;
         catching_island_doing_flag = 0;//�Ѿ������ȡ״̬��������;���½���
         ctrl_c_flag = 1;//�����µ���x��
         catching_count++;//��ȡ�������
    } 
    else if(ctrl_c_flag == 1)
    {
        if(catching_count == 1) //&& (down_up_sign == 0)
        {
            if(sinsign==1)
            {
                SinRampInit(&Sin_Test);
                sinsign = 0;
            }
            Sin_Test.sin_ramp_switch = 1;
            flip_angle = 40000;//�������ȥ
        }
        else if(catching_count == 2)
        {
             conversion_delay = 0;
             catching_count++;//Ҫд�����������д����һ������
        }
        else if(catching_count == 3)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//��
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);          
            conversion_delay_flag = 1;
           if((conversion_delay > 300)&&(conversion_delay_flag == 1))
           {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
                conversion_delay_flag = 0;
                catching_count++;
            }
     
        }
        else if(catching_count == 4)
        {
            catching_island_doing_flag = 1;
            ctrl_c_flag = 0;
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
            change_angle2 = reset_lift_island_offset_angle - 2300000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
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
            change_angle2 = reset_lift_island_offset_angle - 2150000;//̧����������С��Դ����λ�ĵ���Ƕ� -1900000
            catching_island_doing_flag = 1;
            catching_count = 0;
            c_flag = 0;
        }
    }
   
}


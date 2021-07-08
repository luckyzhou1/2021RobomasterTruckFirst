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
int32_t rotate_angle0;//3508翻转的角度
int32_t rotate_angle1;
int32_t set_speed; 
int delay_100ms; //延时100ms
int delay_3000ms; //延时3000ms
int sinsign=1;//斜坡函数是否已经初始化的标志位
int catching_sign=1;//判断是否进行夹取的标志卫

int flip_box_delay;//弹箱子延时
int flip_box_sign;//弹箱子标志位
int Even_Take_Sign=0;//用于判断是哪一步分解动作

int rotational_delay;
int Rotational_Sign;

/*新加的*/
char catch_mode;
int32_t conversion_delay;
char conversion_delay_flag = 0;
int32_t lift_island_reset_speed;
int32_t catch_island_reset_speed;
int32_t change_angle1;//夹资源岛矿石所用夹爪的电机控制角度
int32_t change_angle2;//夹资源岛矿石所用抬升机构的电机控制角度
int32_t ccc;
int32_t island_lift_angle;
int32_t flip_angle;//夹爪翻转角度(资源岛)
char sin_end_flag = 0;
char island_lifting_angle_flag = 0;//表示抬升机构此时位置，0时为低位，1时为中位，2为高位
char in_flag = 0;//第一步夹子复位需要用到的标志位
char clamp_level_flag;//夹子(资源岛)电平标志位
char down_up_sign;//抬升机构位置标志，down为0，up为1
char catching_count = 0;
char swap_speed_flag = 0;//兑换时夹爪是否使用速度环标志位
char lift_island_reset_flag = 0;//抬升机构（资源岛）复位标志
char ialand_catch_reset = 0;//夹取机构(资源岛)复位标志
char r_flag = 0;//按键标志位 
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
char big_island_down = 0;//大资源岛下降标志位

uint16_t z_group_set_delay;//z组合键消抖所用延时
uint16_t x_group_set_delay;//x组合键消抖所用延时
uint16_t c_group_set_delay;//c组合键消抖所用延时

uint16_t catching_reset_delay;//夹取复位所用延时
char catching_reset_flag = 0;//夹取复位所用延时标志位
char catching_reset_delay_flag = 0;//夹取复位所用延时标志位
uint16_t catching_island_delay;//夹岛上矿石时夹子气动延时
char catching_island_doing_flag = 1;//0时不可重新进入夹取，1时可以重新进入夹取

pid_t  Moto_Lift_Island_Pid_Pos[2];  //资源岛抬升机构PID位置环结构体
pid_t  Moto_Lift_Island_Pid_Spd[2];  //资源岛抬升机构PID速度环结构体
pid_t  Moto_Catch_Island_Pid_Pos[2];  //资源岛夹取机构PID位置环结构体
pid_t  Moto_Catch_Island_Pid_Spd[2];  //资源岛夹取机构PID速度环结构体
/************************J-Scope波形显示定义*************************/
float Jscope_set_angle;
float Jscope_get_angle;
float Jscope_real_speed;

/******************************END***********************************/
/*控制夹子*/
void ClipControl(void)
{
    /*放松夹子和收紧夹子*/
    if(KEY_Q && (catching_island_doing_flag == 1))
    {  
        catching_island_doing_flag = 0;//已经进入夹取或兑换状态，不可中途重新进入
        catching_count = 0;//夹取或兑换步骤计数
        q_flag = 1;//代表按下的是ctrl_z键
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
                clamp_level_flag = !clamp_level_flag; //高低电平标志位取反
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
    if(KEY_CTRL && KEY_F &&  (delay_3000ms == 1))//delay_3000ms键盘消抖，小资源岛下降
    {

        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
        change_angle2 = reset_lift_island_offset_angle - 100000;
        catch_mode = 0;
        delay_3000ms = 0;
    }
    
    else if(KEY_F && (delay_3000ms == 1))//小资源岛上升
    {
        change_angle2 = reset_lift_island_offset_angle - 2150000;//抬升机构处于小资源岛中位的电机角度 -1900000  高位-2200000
        catch_mode = 1;//夹取小资源岛模式
        delay_3000ms = 0;
    }
    
    
    if(KEY_G && KEY_CTRL &&  (delay_3000ms == 1))//大资源岛下降
    {  
        catching_count = 0;//步骤计数
        ctrl_g_flag = 1;//代表按下的是g键
        catching_count++;
    }
    else if(ctrl_g_flag == 1)
    {
        if(catching_count == 1) 
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//进
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
    
    else if(KEY_G && (delay_3000ms == 1))//大资源岛抬升
    {  
        catching_count = 0;//步骤计数
        g_flag = 1;//代表按下的是g键
        catching_count++;
    }
    else if(g_flag == 1)
    {
        if(catching_count == 1) 
        {
            change_angle2 = reset_lift_island_offset_angle - 700000;//抬升机构处于大资源岛中位的电机角度 
            catching_island_delay = 0;
            catching_count++;
        }     
        else if(catching_count == 2)
        {
            if((catching_island_delay >=2)&&(catching_island_delay <=5))
            {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//出
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);  
                catching_count++;
            }        
       }
       else if(catching_count == 3)
       {
            catch_mode = 2;//夹取大资源岛模式
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
  
    ClipControl();//夹子控制
    
    /*小资源岛三连取*/
    if(KEY_SHIFT && KEY_CTRL && (catching_island_doing_flag == 1)&&(z_group_set_delay > 500))
    {
         catching_island_doing_flag = 0;//已经进入夹取或兑换状态，不可中途重新进入
         catching_count = 0;//夹取或兑换步骤计数
         swap_speed_flag = 1;//开启速度环复位
         shift_z_flag = 1;
         catching_count++;
        z_group_set_delay = 0;
    }
    else if(shift_z_flag == 1)
    {
        if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//推动机构向前
             catching_count++;
        }
        else if(catching_count == 2)
        {
            /*夹取机构速度环复位*/
            catch_island_reset_speed = -800;
            /*此判断若不加，则可能很快进入判断A，导致复位不成功c'c'r'r*/
            if(catching_reset_flag == 1)//判断是否第二次进入，成立为第二次
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//若为第二次进入，将此标志位置0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//取此判断的代号为：判断A
            {       
                in_flag++;
                swap_speed_flag = 0;
                if(in_flag == 1)//第一次进入了判断A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//夹取复位所要用的延时，保证不会很快第二次进入上面那个if（判断A）
                    catching_reset_flag = 1;//此标志位置1，表示已经走完第一次进入，即将第二次进入判断A
                    catching_reset_delay_flag = 1;//此标志位置1，起到必须走上面延时判断的作用
                }
                else //第二次进入了判断A
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
            flip_angle = 80000;//夹具打出去
            
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
             change_angle2 = reset_lift_island_offset_angle - 2300000;//抬升机构处于小资源岛高位的电机角度 
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
               flip_angle = -80000;//逆时针，对着转子看
            }
        }
        else if(catching_count == 7)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
            if((catching_island_delay >=1)&&(catching_island_delay <=3))
            {
                catching_count++;
            }
        }
        else if(catching_count == 8)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);//推动机构向后运动
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
                chassis_angle_init_flag = 0;//初始化复原
                chassis_pid_pos = 0;//速度环模式
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
            flip_angle = 80000;//顺时针，对着转子看
        }
        else if(catching_count == 12)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 13)
        {
                change_angle2 = reset_lift_island_offset_angle - 2300000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
               flip_angle = -80000;//逆时针，对着转子看
            }
        }
        else if(catching_count == 15)
        {
            change_angle2 = reset_lift_island_offset_angle - 2150000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
                chassis_angle_init_flag = 0;//初始化复原
                chassis_pid_pos = 0;//速度环模式
                catching_count++;
            }
        }
        else if(catching_count == 18)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
             catching_island_delay = 0;
//             if((catching_island_delay >=1)&&(catching_island_delay <=3))
//             {
             catching_count++;//注意，不可以照此处注释掉的那样写，否则永远不会执行此条语句，因为catching_island_delay 一直小于1
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
                flip_angle = 80000;//顺时针，对着转子看      
            }
        }
        else if(catching_count == 20)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
       else if(catching_count == 21)
       {
           change_angle2 = reset_lift_island_offset_angle - 2300000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
               flip_angle = -40000;//逆时针，对着转子看
            }
        }
        else if(catching_count == 23)
        {
            change_angle2 = reset_lift_island_offset_angle - 2150000;//抬升机构处于小资源岛高位的电机角度 
            catching_island_doing_flag = 1;
            shift_z_flag = 0;
            catching_count = 0;
        }
    }
    
    
    
    
    
/*兑换岛上第一个矿石*/
    if(KEY_CTRL && KEY_Z && (catching_island_doing_flag == 1))
    {
        catching_island_doing_flag = 0;//已经进入夹取或兑换状态，不可中途重新进入
        catching_count = 0;//夹取或兑换步骤计数
        swap_speed_flag = 1;//开启速度环复位
        ctrl_z_flag = 1;//代表按下的是ctrl_z键
        catching_count++;
    }
    else if(ctrl_z_flag == 1)//在按键CTRL_Z状态
    {
         if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//推动机构向前
             catching_count++;
        }
 
        if(catching_count == 2) //&& (down_up_sign == 0)
        {
            /*夹取机构速度环复位*/
            catch_island_reset_speed = -800;
            /*此判断若不加，则可能很快进入判断A，导致复位不成功c'c'r'r*/
            if(catching_reset_flag == 1)//判断是否第二次进入，成立为第二次
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//若为第二次进入，将此标志位置0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//取此判断的代号为：判断A
            {       
                in_flag++;
                swap_speed_flag = 0;
                if(in_flag == 1)//第一次进入了判断A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//夹取复位所要用的延时，保证不会很快第二次进入上面那个if（判断A）
                    catching_reset_flag = 1;//此标志位置1，表示已经走完第一次进入，即将第二次进入判断A
                    catching_reset_delay_flag = 1;//此标志位置1，起到必须走上面延时判断的作用
                }
                else //第二次进入了判断A
                {
                    in_flag = 0;
                    catch_island_reset_speed = 0;
                    catching_count++;  
                }
            }
        }
        else if(catching_count == 3)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             conversion_delay = 0;
             catching_count++;//要写在这里，不可以写在下一个步骤
        }
        else if(catching_count == 4)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//进
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
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//推杆回来
            catching_island_delay = 0;
            catching_count++;
        }
        if(catching_count == 6) //&& (down_up_sign == 0)
        {  
            if((catching_island_delay >=2)&&(catching_island_delay <=4))
            {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具,高电平运动
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
                flip_angle = 80000;//夹子打出去;
        }
        else if(catching_count == 8)
        {
             conversion_delay = 0;
             catching_count++;//要写在这里，不可以写在下一个步骤
        }
        else if(catching_count == 9)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//出
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
    
    
    
/*夹取岛上第一个矿石*/
    else if( KEY_Z && (catching_island_doing_flag == 1) ) //必须是else if,不能是if,否则按下CTRL_Z时会触发KEY_Z
    {
        catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
        catching_count = 0;//夹取步骤计数
        swap_speed_flag = 1;//开速度环复位
        z_flag = 1;//代表按下的是z键
        catching_count++;
    }
    else if(z_flag == 1)//在按键Z夹取状态
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);//推动机构向前
             catching_count++;
        }
        else if(catching_count == 2)
        {
            /*夹取机构速度环复位*/
            catch_island_reset_speed = -800;
            /*此判断若不加，则可能很快进入判断A，导致复位不成功c'c'r'r*/
            if(catching_reset_flag == 1)//判断是否第二次进入，成立为第二次
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//若为第二次进入，将此标志位置0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//取此判断的代号为：判断A
            {       
                in_flag++;
                swap_speed_flag = 0;
                if(in_flag == 1)//第一次进入了判断A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//夹取复位所要用的延时，保证不会很快第二次进入上面那个if（判断A）
                    catching_reset_flag = 1;//此标志位置1，表示已经走完第一次进入，即将第二次进入判断A
                    catching_reset_delay_flag = 1;//此标志位置1，起到必须走上面延时判断的作用
                }
                else //第二次进入了判断A
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
            flip_angle = 80000;//夹具打出去
            
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
             change_angle2 = reset_lift_island_offset_angle - 2300000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
               flip_angle = -80000;//逆时针，对着转子看
            }
        }
        else if(catching_count == 7)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
            if((catching_island_delay >=1)&&(catching_island_delay <=3))
            {
                catching_count++;
            }
        }
        else if(catching_count == 8)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);//推动机构向后运动
            change_angle2 = reset_lift_island_offset_angle - 2150000;
            catching_count++;
            catching_island_delay = 0;
        }
        else if(catching_count == 9)
        {
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
                catching_count = 0;
                catching_island_doing_flag = 1;
                z_flag = 0; 
             }
        }
    }
  
    
/*兑换矿石第二部分*/    
    else if(remote_control.mouse.press_right && (catching_island_doing_flag == 1))//remote_control.mouse.press_left
    {
         catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
         catching_count = 0;//夹取步骤计数
         mouse_right_flag = 1;
         catching_count++;
    }
    else if(mouse_right_flag == 1)
    {
        if(catching_count == 1)
        {
            conversion_delay = 0;
            catching_count++;//要写在这里，不可以写在下一个步骤

        }
        else if(catching_count == 2)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//出
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
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
             catching_island_delay = 0;     
             catching_count++;            
        }
        else if(catching_count == 4)
        {
             if((catching_island_delay >=2)&&(catching_island_delay <=5))
             {
                 HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
                 conversion_delay = 0;
                 catching_count++;  
             }
        }
        else if(catching_count == 5)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//进
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
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
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
               flip_angle = -80000;//逆时针，对着转子看
            }
          
        }
        else if(catching_count == 8)
        {         
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
            catching_count = 0;
            catching_island_doing_flag = 1;
            mouse_right_flag = 0; 
        }
        
    }
    
    
    
/*兑换岛上第二个矿石*/
    if(KEY_CTRL && KEY_X && (catching_island_doing_flag == 1))
    {
       catching_island_doing_flag = 0;//已经进入夹取或兑换状态，不可中途重新进入
       catching_count = 0;//夹取或兑换步骤计数
       ctrl_x_flag = 1;//代表按下的是ctrl_x键
       swap_speed_flag = 1;//开启速度环复位
       catching_count++;
    }
    else if(ctrl_x_flag == 1)//在按键CTRL_X状态
    {
        
        if(catching_count == 1)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
            catching_count++;
        }
        if(catching_count == 2)
        {
            catch_island_reset_speed = -800;
            /*此判断若不加，则可能很快进入判断A，导致复位不成功c'c'r'r*/
            if(catching_reset_flag == 1)//判断是否第二次进入，成立为第二次
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//若为第二次进入，将此标志位置0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//代号：判断A
            {
               
                speed_cha = catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm;
                in_flag++;
                swap_speed_flag = 0;
                if(in_flag == 1)//第一次进入判断A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//夹取复位所要用的延时，保证不会很快第二次进入上面那个if（判断A）
                    catching_reset_flag = 1;//此标志位置1，表示已经走完第一次进入，即将第二次进入判断A
                    catching_reset_delay_flag = 1;//此标志位置1，起到必须走上面延时判断的作用
                }
                else //第二次进入判断A
                {
                    in_flag = 0;
                    catching_count++;
                }
            }
         }        
        else if(catching_count == 3)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             conversion_delay = 0;
             catching_count++;//要写在这里，不可以写在下一个步骤
        }
        else if(catching_count == 4)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//进
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
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
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
                flip_angle = 80000;//夹子打出去;
        }
        else if(catching_count == 8)
        {
             conversion_delay = 0;
             catching_count++;//要写在这里，不可以写在下一个步骤
        }
        else if(catching_count == 9)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//出
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
    
    
    
/*夹取岛上第二个矿石*/
    else if(KEY_X && (catching_island_doing_flag == 1)) 
    {
        catching_count = 0;
        catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
        swap_speed_flag = 1;//开速度环复位
        x_flag = 1;//代表按下的是x键
        catching_count++;//夹取步骤计数
        
    }
    else if(x_flag == 1)
    {
       
        if(catching_count == 1)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具           
             catching_count++;
        }
        else if(catching_count == 2)
        {
            /*夹取机构速度环复位*/
            catch_island_reset_speed = -800;
            /*此判断若不加，则可能很快进入判断A，导致复位不成功c'c'r'r*/
            if(catching_reset_flag == 1)//判断是否第二次进入，成立为第二次
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//若为第二次进入，将此标志位置0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//取此判断的代号为：判断A
            {       
                in_flag++;
                swap_speed_flag = 0;
//                catching_count++;
                if(in_flag == 1)//第一次进入了判断A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//夹取复位所要用的延时，保证不会很快第二次进入上面那个if（判断A）
                    catching_reset_flag = 1;//此标志位置1，表示已经走完第一次进入，即将第二次进入判断A
                    catching_reset_delay_flag = 1;//此标志位置1，起到必须走上面延时判断的作用
                }
                else //第二次进入了判断A
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
            catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
            if(sinsign==1)
            {
                SinRampInit(&Sin_Test);
                sinsign = 0;
            }
            Sin_Test.sin_ramp_switch = 1;
            flip_angle = 80000;//顺时针，对着转子看
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
                change_angle2 = reset_lift_island_offset_angle - 2300000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
               flip_angle = -80000;//逆时针，对着转子看
            }
        }
        else if(catching_count == 7)
        {
            change_angle2 = reset_lift_island_offset_angle - 2150000;//抬升机构处于小资源岛高位的电机角度 -1900000
            catching_count = 0;
            catching_island_doing_flag = 1;
            x_flag = 0;
        }
    }

    
    
    
    
/*兑换岛上夹取的第三个矿石*/
    else if(KEY_CTRL && KEY_C && (catching_island_doing_flag == 1))
    {
         catching_count = 0;
         catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
         ctrl_c_flag = 1;//代表按下的是x键
         catching_count++;//夹取步骤计数
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
            flip_angle = 40000;//夹子伸出去
        }
        else if(catching_count == 2)
        {
             conversion_delay = 0;
             catching_count++;//要写在这里，不可以写在下一个步骤
        }
        else if(catching_count == 3)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET);//出
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
    
    

    
/*夹取岛上第三个矿石*/
    else if(KEY_C && (catching_island_doing_flag == 1))
    {
         catching_count = 0;
         catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
         swap_speed_flag = 1;//开速度环复位
         c_flag = 1;//代表按下的是x键
         catching_count++;//夹取步骤计数
    } 
    else if(c_flag == 1)
    {
       if(catching_count == 1)
        {
             
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);//放松夹具
//             catching_island_delay = 0;
//             if((catching_island_delay >=1)&&(catching_island_delay <=3))
//             {
             catching_count++;//注意，不可以照此处注释掉的那样写，否则永远不会执行此条语句，因为catching_island_delay 一直小于1
//             }
        }
        else if(catching_count == 2)
        {
            /*夹取机构速度环复位*/
            catch_island_reset_speed = -800;
            /*此判断若不加，则可能很快进入判断A，导致复位不成功c'c'r'r*/
            if(catching_reset_flag == 1)//判断是否第二次进入，成立为第二次
            {
                if(catching_reset_delay > 1000)
                {
                    catching_reset_delay_flag = 0;//若为第二次进入，将此标志位置0
                }
            }
            if((catch_island_reset_speed - Island_Catch_Motor[0].speed_rpm < (-798)) && (catching_reset_delay_flag == 0))//取此判断的代号为：判断A
            {       
                in_flag++;
                swap_speed_flag = 0;
//                catching_count++;
                if(in_flag == 1)//第一次进入了判断A
                {
                    catching_island_doing_flag = 1;
                    catching_reset_delay = 0;//夹取复位所要用的延时，保证不会很快第二次进入上面那个if（判断A）
                    catching_reset_flag = 1;//此标志位置1，表示已经走完第一次进入，即将第二次进入判断A
                    catching_reset_delay_flag = 1;//此标志位置1，起到必须走上面延时判断的作用
                }
                else //第二次进入了判断A
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
            flip_angle = 80000;//顺时针，对着转子看      
        
        }
        else if(catching_count == 4)
        {
             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
             if((catching_island_delay >=1)&&(catching_island_delay <=3))
             {
                catching_count++;
             }
        }
        else if(catching_count == 5)
        {
            change_angle2 = reset_lift_island_offset_angle - 2300000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
               flip_angle = -40000;//逆时针，对着转子看
            }
        }
        else if(catching_count == 7)
        {
            change_angle2 = reset_lift_island_offset_angle - 2150000;//抬升机构处于小资源岛高位的电机角度 -1900000
            catching_island_doing_flag = 1;
            catching_count = 0;
            c_flag = 0;
        }
    }
   
}


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
     ClipControl();//夹子控制
    
    /*夹取岛上第一个矿石*/
    if(KEY_Z && (catching_island_doing_flag == 1)) //必须是else if,不能是if,否则按下CTRL_Z时会触发KEY_Z
    {
        catching_island_doing_flag = 0;//已经进入夹取状态，不可中途重新进入
        catching_count = 0;//夹取步骤计数
        swap_speed_flag = 1;//开速度环复位
        z_flag = 1;//代表按下的是z键
        catching_count++;
    }
    else if(z_flag == 1)//在按键Z夹取状态
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
             change_angle2 = reset_lift_island_offset_angle - 900000;//抬升机构处于小资源岛高位的电机角度 -1900000
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
            change_angle2 = reset_lift_island_offset_angle - 700000;
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
                change_angle2 = reset_lift_island_offset_angle - 900000;//抬升机构处于大资源岛高位的电机角度 
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
            change_angle2 = reset_lift_island_offset_angle - 700000;//抬升机构处于大资源岛中位的电机角度
            catching_count = 0;
            catching_island_doing_flag = 1;
            x_flag = 0;
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
            change_angle2 = reset_lift_island_offset_angle - 900000;//抬升机构处于大资源岛高位的电机角度 
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
            change_angle2 = reset_lift_island_offset_angle - 700000;//抬升机构处于大资源岛中位的电机角度 
            catching_island_doing_flag = 1;
            catching_count = 0;
            c_flag = 0;
        }
    }
    
    
   
}

#include "ResetTask.h"
#include "RC_Task.h"
#include "CatchingTask.h"
#include "CanBus_Task.h"
#include "GimbalControl.h"
#include "main.h"


char pitch_reset_flag = 0;//PITCH轴速度环复位标志位
char yaw_reset_flag = 0;//PITCH轴速度环复位标志位
int32_t reset_pitch_offset_angle;//复位后PITCH轴初始角度
int32_t reset_yaw_offset_angle;//复位后YAW轴初始角度

int32_t reset_lift_island_offset_angle;//复位后抬升初始角度
char lift_island_reset_success;//复位成功标志
char lift_island_reset_success_assist;//此标志位目的是：保证其他机构还未复位成功时，不会一直进入复位是否成功的判断

/*机构复位函数*/
void ResetControl(void)
{
    if(KEY_CTRL && KEY_R && (catching_island_doing_flag == 1))//复位
    {
//        catching_island_doing_flag = 0;//机构已经进入运动状态，不可中途重新进入（是否可以进入夹取标志位，0不可进，1可进）
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);//夹紧夹具
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET);//进
        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);          
        //catching_count = 1;
        r_flag = 1;//状态标志位置1
//        ialand_catch_reset = 1;//夹取（资源岛）机构复位开始
        lift_island_reset_flag = 1;//抬升（资源岛）机构复位开始
        pitch_reset_flag = 1;//PITCH轴速度环复位标志位
        yaw_reset_flag = 1;//YAW轴速度环复位标志位
        lift_island_reset_success = 0;//抬升开始复位，还未成功
        lift_island_reset_success_assist = 1;
    }
    else if(r_flag == 1)
    {
        /*各个机构复位速度赋值*/
        catch_island_reset_speed = -800;
        lift_island_reset_speed = 800;
        pitch_reset_speed = 200;
        yaw_reset_speed = 200;
        
       //因为按键按下一次，相当于按2次，所以能实现复位，所以这样判断是可以的 
        if((lift_island_reset_speed - Island_Lift_Motor[0].speed_rpm > 398)&&(lift_island_reset_success_assist == 1))
        {       
            change_angle2 = Island_Lift_Motor[0].total_angle;      
            reset_lift_island_offset_angle =  Island_Lift_Motor[0].total_angle;           
            lift_island_reset_flag = 0;//抬升（资源岛）机构复位结束，无斜坡
            lift_island_reset_success = 1;//抬升复位已成功
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
//            ialand_catch_reset = 0;//夹取（资源岛）机构复位结束，斜坡
//            catch_island_reset_speed = 0;
//        }
         if((lift_island_reset_flag == 0)&&(pitch_reset_flag == 0)&&(yaw_reset_flag == 0))//云台若未装，不使用(yaw_reset_flag == 0)&&(pitch_reset_flag == 0)&&(ialand_catch_reset == 0
        {
//            catching_island_doing_flag = 1;
            r_flag = 0;
        }
    }
}

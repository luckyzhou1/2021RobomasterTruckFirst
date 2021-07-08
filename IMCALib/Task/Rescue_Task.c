#include "Rescue_Task.h"
#include "gpio.h"
#include "RC_Task.h"
#include "CatchingTask.h"

int swip_card_ctrl_delay;//防止松开CTRL_A时，误按A键
int rescue_ctrl_delay;//防止松开CTRL_Q时，误按Q键
int rescue_out_delay;//刷卡步骤延时(伸出)
int rescue_in_delay;//刷卡步骤延时(缩入)
int KEY_CTRL_A_count = 4;//消抖
int KEY_A_count = 4;//消抖
char rescuing_flag = 0;//刷卡运动进行中，标志位
char mouse_left_flag = 0;//按键标志位
char rescue_level_flag = 0;//救援电平标志位
char rescue_can_flag = 1;//救援是否能使用标志
uint16_t rescue_delay;//救援按键延时
/*刷卡救援*/
void SwipingCard(void)
{
    if(KEY_A && KEY_CTRL &&(rescuing_flag == 0) && (KEY_CTRL_A_count > 2))//缩入
    {
        KEY_CTRL_A_count = 0;//消抖
        rescuing_flag = 1;//刷卡运动进行中
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
        rescue_out_delay = 0;//步骤延时开始
        swip_card_ctrl_delay = 0;//防止松开CTRL_A时，误按A键
    }
    else if((rescue_out_delay > 200)&&(rescue_out_delay < 400)&&(rescuing_flag == 1))
    {
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
         rescuing_flag = 0;
          
    }
    
    else if(KEY_A  && (swip_card_ctrl_delay >= 4)&&(KEY_A_count > 2) &&(rescuing_flag == 0))//伸出
    {
        KEY_A_count = 0;
        rescuing_flag = 1; 
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
        rescue_in_delay = 0;
    }
    else if((rescue_in_delay > 200)&&(rescue_in_delay < 400)&&(rescuing_flag == 1))
    {
         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
         rescuing_flag = 0;
         
    }
}
    
/*救援*/
void RescueingControl(void)  
{
    /*打下救援和收回救援*/
    if(remote_control.mouse.press_left && (rescue_can_flag == 1))
    {  
        catching_count = 0;
        rescue_can_flag = 0;
        mouse_left_flag = 1;//代表按下的是鼠标左键
        catching_count++;
    }
    else if(mouse_left_flag == 1)
    {
        if(catching_count == 1)
        {
            if(rescue_level_flag == 0)
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_SET);
            else if(rescue_level_flag == 1)
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_RESET);
            rescue_delay = 0;
            catching_count++;
        }     
        else if(catching_count == 2)
        {
            if((rescue_delay >=700)&&(rescue_delay <=7000))
            {
                catching_count = 0;
                rescue_level_flag = !rescue_level_flag; //高低电平标志位取反
                rescue_can_flag = 1;
                mouse_left_flag = 0; 
            }        
       }
    }
//    if(KEY_Q && KEY_CTRL )
//    {
//        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_RESET);
//        rescue_ctrl_delay = 0;//防误触延时开始
//    }
//    else if(KEY_Q && (rescue_ctrl_delay >= 1))
//    {
//         HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_SET);
//    }
}    
    
//    if(KEY_A && KEY_CTRL)
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
//    else if(KEY_A)
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
//    
//    if(KEY_S && KEY_CTRL)
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
//    else if(KEY_S)
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);



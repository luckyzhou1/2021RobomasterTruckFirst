#include "Rescue_Task.h"
#include "gpio.h"
#include "RC_Task.h"
#include "CatchingTask.h"

int swip_card_ctrl_delay;//��ֹ�ɿ�CTRL_Aʱ����A��
int rescue_ctrl_delay;//��ֹ�ɿ�CTRL_Qʱ����Q��
int rescue_out_delay;//ˢ��������ʱ(���)
int rescue_in_delay;//ˢ��������ʱ(����)
int KEY_CTRL_A_count = 4;//����
int KEY_A_count = 4;//����
char rescuing_flag = 0;//ˢ���˶������У���־λ
char mouse_left_flag = 0;//������־λ
char rescue_level_flag = 0;//��Ԯ��ƽ��־λ
char rescue_can_flag = 1;//��Ԯ�Ƿ���ʹ�ñ�־
uint16_t rescue_delay;//��Ԯ������ʱ
/*ˢ����Ԯ*/
void SwipingCard(void)
{
    if(KEY_A && KEY_CTRL &&(rescuing_flag == 0) && (KEY_CTRL_A_count > 2))//����
    {
        KEY_CTRL_A_count = 0;//����
        rescuing_flag = 1;//ˢ���˶�������
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
        rescue_out_delay = 0;//������ʱ��ʼ
        swip_card_ctrl_delay = 0;//��ֹ�ɿ�CTRL_Aʱ����A��
    }
    else if((rescue_out_delay > 200)&&(rescue_out_delay < 400)&&(rescuing_flag == 1))
    {
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
         rescuing_flag = 0;
          
    }
    
    else if(KEY_A  && (swip_card_ctrl_delay >= 4)&&(KEY_A_count > 2) &&(rescuing_flag == 0))//���
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
    
/*��Ԯ*/
void RescueingControl(void)  
{
    /*���¾�Ԯ���ջؾ�Ԯ*/
    if(remote_control.mouse.press_left && (rescue_can_flag == 1))
    {  
        catching_count = 0;
        rescue_can_flag = 0;
        mouse_left_flag = 1;//�����µ���������
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
                rescue_level_flag = !rescue_level_flag; //�ߵ͵�ƽ��־λȡ��
                rescue_can_flag = 1;
                mouse_left_flag = 0; 
            }        
       }
    }
//    if(KEY_Q && KEY_CTRL )
//    {
//        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_RESET);
//        rescue_ctrl_delay = 0;//������ʱ��ʼ
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



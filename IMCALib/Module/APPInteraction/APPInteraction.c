/**********************************************************************************************************************
 * �ļ�     ��APPInteraction.c
 * ���     ��ͨ��USMART�������ʵ�����ߵ��Σ���Ҫ����PID�����ߵ��Σ���Ҫ���XCOM V2.0���ڵ�������ʹ�á�
 * ʹ�÷��� ������USART1���жϣ�������Ϊ115200��ͬʱҪ��ͨһ����ʱ��������Ƶ��Ϊ10hz��100ms�ж�һ�Σ�����ִ��usmartɨ�衣
 *            ��ΪUSMART��������ǲ�֧��С�������ߵ��Σ��������ô��ڵ����������ߵ���ʱӦ����������1000��Ȼ��Ҫ����Ӧ�ĺ�
 *            ���õ��������ٳ���1000����Reset_Usart1_Receive_IT()��ӵ�USART1_IRQHandler()�б����ã���UsmartScan()��ӵ�
 *            ��TIM7_IRQHandler()�б����ã�ʵ��100ms����һ�Ρ�
**********************************************************************************************************************/

#include "APPInteraction.h"
#include "usmart.h"
#include "usart.h"
#include "tim.h"


u8  USART_RX_BUF[USART_REC_LEN];  //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
u16 USART_RX_STA = 0;  //����״̬���	
u8 aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer


/**********************************************************************************************************************
 * @brief  USMART���������ʼ������Ҫ�Ƕ��õ��Ĵ��ںͶ�ʱ����ʼ����USMART��ʼ��
 * @param  None
 * @retval None
 * @others �õ��Ĵ��ںͶ�ʱ�����ж����ȼ�Ҫ���ó�һ����CANͨ�ŵ��ж����ȼ����Ҳһ������һ����ʱ��������ݳ���
**********************************************************************************************************************/
void APPInteractionInit(void)
{
    
    HAL_UART_Receive_IT(&huart1, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ������1�����жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
    HAL_TIM_Base_Start_IT(&TIMX);  //Ƶ��Ϊ10hz��100ms�ж�һ��
    
    usmart_dev.init(); 		//��ʼ��USMART

}    


/**********************************************************************************************************************
 * @brief   �����жϺ����������Լ����壬�������ݵĽ���
 * @param   UART_HandleTypeDef *huart     UART handle Structure
 * @retval  None
**********************************************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1
	{
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		
                
			}
            
		}

	}
    
}



/**********************************************************************************************************************
 * @brief   ��ʱ������������1�����жϣ���USART1_IRQHandler()�б�����
 * @param   None
 * @retval  None
 * @others  USART1���ж����ȼ��Ͷ�ʱ��7���ж����ȼ�������ó�һ��
**********************************************************************************************************************/
void Reset_Usart1_Receive_IT(void)
{
    u32 timeout=0;
    timeout=0;
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//�ȴ�����
    {
         timeout++;////��ʱ����
         if(timeout>HAL_MAX_DELAY) break;		
    }
     
    timeout=0;
    while(HAL_UART_Receive_IT(&huart1, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
    {
         timeout++; //��ʱ����
         if(timeout>HAL_MAX_DELAY) break;	
    }
    
}


/**********************************************************************************************************************
 * @brief    USMARTɨ�裬100msִ��һ�Σ���TIM7_IRQHandler()�б�����
 * @param    None
 * @retval   None
 * @others   ���Ի��������Ķ�ʱ��������ʱ����Ƶ��Ϊ10Hz����100ms�ж�һ�Σ������жϺ���
 *           ���˺���������Ӧ�Ķ�ʱ���жϺ���������
**********************************************************************************************************************/
void UsmartScan(void)
{
    if(__HAL_TIM_GET_IT_SOURCE(&TIMX,TIM_IT_UPDATE)==SET)//����ж�
    {
        usmart_dev.scan();	//ִ��usmartɨ��
        __HAL_TIM_SET_COUNTER(&TIMX,0);    //��ն�ʱ����CNT
        __HAL_TIM_SET_AUTORELOAD(&TIMX,100);//�ָ�ԭ��������
    }
    __HAL_TIM_CLEAR_IT(&TIMX, TIM_IT_UPDATE);//����жϱ�־λ
  
}


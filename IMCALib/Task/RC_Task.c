/**********************************************************************************************************************
 * @file   RC_Task.c
 * @brief  DR16ң�ؽ������ݵĽ������ô���2��DMA����ң���������������ݣ�ʹ��ʱ�����ļ���ͷ�ļ����
 *         ��stm32f4xx_it.c�У���USART2_IRQHandler()���������HAL_UART_IDLE_IRQHandler(&huart2)
 *         �����жϿ��к���������USART2_IRQHandler()������ԭ����HAL_UART_IRQHandler(&huart2)ע�͵�
**********************************************************************************************************************/

#include "RC_Task.h"
#include "usart.h"
#include "CanBus_Task.h"


RC_Type remote_control;
uint8_t UART_Buffer[100];


/**********************************************************************************************************************
  * @Func	 void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief   DR16���ջ�Э��������
  * @Param	 RC_Type* rc��  �洢ң�������ݵĽṹ��
  *          uint8_t* buff�����ڽ���Ļ��桡��
  * @Retval	 None  
 *********************************************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
    sen_can_comu_1(&hcan2,buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7]);
  sen_can_comu_2(&hcan2,buff[8],buff[9],buff[10],buff[11],buff[12],buff[13],buff[14],buff[15]);
    
//	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
	
}


/**********************************************************************************************************************
  * @brief  ���ô���û��DMA�жϵĽ���,����ʹ���˿����жϣ���������û�б�Ҫ�ٿ���DMA�жϣ�����HAL����
  *			������UART_Receive_DMA����Ĭ�Ͽ�����DMA�жϣ�����Ҫ�Լ�ʵ��һ��û���жϵ�DMA���պ�����	
  * @param  UART_HandleTypeDef *huart   UART handle Structure
  *         uint8_t *pData              ���ջ�����
  *         uint16_t Size               ���ݴ�С
  * @retval HAL status
**********************************************************************************************************************/
HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
 uint32_t *tmp;
  
  /* Check that a Rx process is not already ongoing */
  if(huart->RxState == HAL_UART_STATE_READY) 
  {
    if((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    /* Set the DMA abort callback */
    huart->hdmarx->XferAbortCallback = NULL;

    /* Enable the DMA channel */
    tmp = (uint32_t*)&pData;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);

    /* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART Parity Error Interrupt */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


/**********************************************************************************************************************
 * @brief  �������ڿ����ж�,�ڳ�ʼ��ʱ����
 * @param  UART_HandleTypeDef *huart   UART handle Structure
 *         uint8_t *pData              ���ջ�����
 *         uint16_t Size               ���ݴ�С
 * @retval HAL status
**********************************************************************************************************************/
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  /*�������ڿ����ж�*/
	
		return UART_Receive_DMA_NoIT(huart,pData,Size);  /*����û��DMA�жϵ�DMA���պ���*/

}


/**********************************************************************************************************************
 * @brief   ���ڿ����жϣ���USART2�ж��е���
 * @param   UART_HandleTypeDef *huart    UART handle Structure
 *
 * @retval  None
**********************************************************************************************************************/
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
		
		uint32_t DMA_FLAGS,tmp;
	
			
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
			
		tmp = huart->Instance->SR;
		tmp = huart->Instance->DR;
		tmp++;  
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
		
		//ʧ��DMA
	    __HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
		
		huart->hdmarx->Instance->NDTR = huart->RxXferSize;
		__HAL_DMA_ENABLE(huart->hdmarx);
		
	}
	Callback_RC_Handle(&remote_control,huart->pRxBuffPtr);  /*ң�ؽ������������ݽ���*/
	
}


//ң�����������ƣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024����
int16_t RcDeadlineLimit(int16_t input, uint8_t dealine)
{
    int16_t output;
    if((input > dealine)||(input < -dealine))
    {
        output = input;
    }
    else
    {
        output = 0;
    }
    return output;

}



typedef struct __FILE FILE;
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);  /*ʹ�ô���1��������*/
 
  return ch;
}


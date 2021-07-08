#ifndef __APPINERACTION_
#define __APPINERACTION_

#include "mytype.h"
#include "usmart.h"

/*��ʱ�����õ��Ƕ�ʱ��7*/
#define TIMX  htim7

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define RXBUFFERSIZE   1 //�����С
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;      //����״̬���	
extern u8 aRxBuffer[RXBUFFERSIZE];  //HAL��USART����Buffer


/*USMART���������ʼ��*/
void APPInteractionInit(void);
/*��ʱ������������1�����ж�*/
void Reset_Usart1_Receive_IT(void);
/*USMARTɨ�裬100msִ��һ�Σ���TIM7_IRQHandler�б�����*/
void UsmartScan(void);


#endif


#ifndef __ANO_DT_
#define __ANO_DT_

#include "usart.h"
#include "mytype.h"

extern uint8_t Data_Send_ANO_DT;


//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/*********************************************************������*****************************************************************/
void Data_Send_User(u16 data1,u8 data2,s16 data3);
void DataSendS16(float data1,float data2);
/**********************************************************END*******************************************************************/

/*ͨ�����ڷ������ݵ���������վ*/
void ANO_DT_Send_Data(uint8_t *data, uint8_t len);
/*���·��͵���������վ������*/
void ANO_DT_DataUpdate(void);

/*************************************************�û��Զ������ݷ��ͺ���*********************************************************/
/*�����ĸ�float�����ݵ���������վ*/
void DataSendFloat(float data1, float data2, float data3, float data4);
/*�����ĸ�uint8_t�����ݵ���������վ*/
void DataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
/*�����ĸ�int16_t�����ݵ���������վ*/
void DataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4);
/*�����ĸ�uint16_t�����ݵ���������վ*/
void DataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
/*�����ĸ�int32_t�����ݵ���������վ*/
void DataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4);
/********************************************************END********************************************************************/


#endif


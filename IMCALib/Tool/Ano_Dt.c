/**********************************************************************************************************************
 *@brief   ���ļ�������MCU����������վ��ͨ�ţ�ͨ�����ߴ��ڽ����ݷ��͵���������վ�Ͻ��в�����ʾ��������ʾ��û�����
 *         J-Scope����ȽϺã����Ǳ��������ߵģ�����Ҫ�鿴���˶�ʱ�����ݲ���ʱ����ͨ�����ߴ��ڷ������ݵ���������վ��
 *
 *
**********************************************************************************************************************/

#include "Ano_Dt.h"
#include "CanBus_Task.h"
#include "usart.h"
#include "IMU_Task.h"
#include "ChassisControl.h"


uint8_t Data_Send_ANO_DT;
uint8_t data_to_send[50];	//�������ݻ���
uint8_t DMA_aTxBuffer[] = {10};       // ����DMA���ͻ�����


/*������*/
void Data_Send_User(u16 data1,u8 data2,s16 data3)
{
    u8 _cnt=0,sum = 0,i;

	data_to_send[_cnt++]=0xAA;           //֡ͷAAAA
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;           //������
	data_to_send[_cnt++]=0;              //���ݳ���

	data_to_send[_cnt++]=BYTE1(data1);
	data_to_send[_cnt++]=BYTE0(data1);
	
	data_to_send[_cnt++]=data2;
	
	data_to_send[_cnt++]=BYTE1(data3);
	data_to_send[_cnt++]=BYTE0(data3);

	data_to_send[3] = _cnt-4;            //���ݳ��ȸ�ֵ

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];            //����У�����
		
	data_to_send[_cnt++] = sum;          //����У�鸳ֵ
	
	ANO_DT_Send_Data(data_to_send, _cnt);//֡���͵���λ��
    
}



void DataSendS16(float data1,float data2)
{
    u8 _cnt=0,sum = 0,i;

    data_to_send[_cnt++]=0xAA;           //֡ͷAAAA
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xF1;           //������
    data_to_send[_cnt++]=0;              //���ݳ���

    data_to_send[_cnt++]=BYTE3(data1);
    data_to_send[_cnt++]=BYTE2(data1);
    data_to_send[_cnt++]=BYTE1(data1);
    data_to_send[_cnt++]=BYTE0(data1);

    data_to_send[_cnt++]=BYTE3(data2);
    data_to_send[_cnt++]=BYTE2(data2);
    data_to_send[_cnt++]=BYTE1(data2);
    data_to_send[_cnt++]=BYTE0(data2);


    data_to_send[3] = _cnt-4;            //���ݳ��ȸ�ֵ

    for(i=0;i<_cnt;i++)
    sum += data_to_send[i];            //����У�����

    data_to_send[_cnt++] = sum;          //����У�鸳ֵ

    ANO_DT_Send_Data(data_to_send, _cnt);//֡���͵���λ��
    
}


/*ͨ�����ڷ������ݵ���������վ*/
void ANO_DT_Send_Data(uint8_t *data, uint8_t len)
{
    
    uint8_t i;
	
	for(i=0;i<len;i++)
	{
        
        HAL_UART_Transmit(&huart1,data+i,1,10);
        
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);//��������жϣ����ڼ�鴮��UART1�Ƿ������
//        while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC) == RESET);
        
	}
    
}


/*���·��͵���������վ������*/
void ANO_DT_DataUpdate(void)
{
    
//    DataSendFloat(Chassis.fr_motor_rpm_201, Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2, 0, 0);
    /*�����ĸ������Ӧ��ת��*/
    
    DataSendFloat(Chassis.vx, Chassis.vy, 0, 0);
    
//    DataSendFloat(Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2, Chassis_Motor[FRON_LEFT_202].speed_rpm/19.2, \
//                  Chassis_Motor[REAR_LEFT_203].speed_rpm/19.2, Chassis_Motor[REAR_RIGH_204].speed_rpm/19.2);
    
    
}


/*�����ĸ�float�����ݵ���������վ*/
void DataSendFloat(float data1, float data2, float data3, float data4)
{
    
    u8 _cnt=0,sum = 0,i;
	float _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
	
	_temp = data1;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//�������ݵ�������λ��
    
}


/*�����ĸ�uint8_t�����ݵ���������վ*/
void DataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	uint8_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
    data_to_send[_cnt++]=_temp;
    _temp = data2;
    data_to_send[_cnt++]=_temp;
    _temp = data3;
    data_to_send[_cnt++]=_temp;
    _temp = data4;
    data_to_send[_cnt++]=_temp;
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//�������ݵ�������λ��
    
}


/*�����ĸ�int16_t�����ݵ���������վ*/
void DataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	int16_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//�������ݵ�������λ��
    
}


/*�����ĸ�uint16_t�����ݵ���������վ*/
void DataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	uint16_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//�������ݵ�������λ��
    
}


/*�����ĸ�int32_t�����ݵ���������վ*/
void DataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	int32_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//�������ݵ�������λ��
    
}


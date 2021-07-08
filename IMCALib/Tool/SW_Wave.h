#ifndef __SHANWAI_WAVE_H
#define __SHANWAI_WAVE_H

#include "mytype.h"



/*���ڷ�������*/
void UsartSendData(uint8_t *tx_buf, uint8_t len);
/*�������ݵ�ɽ��๦�ܵ��������е�����ʾ�������в�����ʾ*/
void SwSendWare(uint8_t *wareaddr, int16_t waresize);
/*���·��͵�ɽ��๦�ܵ������ֵ�����*/
void SwDataWaveUpdate(void);

/*********************************************************��ͬ�������ͷ��ͺ���***********************************************************/
/*������λfloat�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendFloat(float data1, float data2, float data3, float data4);
/*���Ͱ�λfloat�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendFloatPro(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8);
/*������λint8_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendInt8(int8_t data1, int8_t data2, int8_t data3, int8_t data4);
/*������λuint8_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
/*������λint16_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4);
/*������λuint16_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
/*������λint32_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4);
/*������λuint32_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendUint32(uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4);

/*****************************************************************END********************************************************************/


#endif

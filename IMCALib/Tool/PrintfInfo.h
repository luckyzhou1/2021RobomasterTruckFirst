#ifndef __PRINTFINFO
#define __PRINTFINFO

#include "mytype.h"


/*�봮����ʾ�ǿ�������Ϣ��ȡ���Ǹ�����ĺ궨���ע��*/
//#define M6020_MOTOR_MEASURE  //��̨6020
#define M3508_MOTOR_MEASURE  //����3508
//#define REMOTE_MEASURE
#define IMU_MEASURE  //��������Ϣ

extern uint8_t Printf_Info;

void PrintfInfo(void);


#endif

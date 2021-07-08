#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 #include "mytype.h"
 
extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据
//extern unsigned char Datascope_Dp_i;          //计数变量
//extern unsigned char Datascope_Dp_Send_Count; //串口需要发送的数据个数
extern uint8_t Data_Scope_DP_Send;
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区
void DataScope(void);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数
 
 
#endif 

#ifndef __USER_LIB_
#define __USER_LIB_

#include "mytype.h"


#define PI 3.14159265358979f  /*Բ����*/
//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) LoopFloatConstrain((Ang), -PI, PI)
//�Ƕȸ�ʽ��Ϊ-180~180
#define theta_format(Ang) LoopFloatConstrain((Ang), -180, 180)
//�������ֵת���ɽǶ�ֵ����λ��rad 
#define Motor_Ecd_To_Rad  0.000766990394f    //  2*PI/8192 = rad/ecd_angle
//תÿ����ת��Ϊ����ÿ��
#define RPM_To_Rads  0.1047197551f   // rpm -> rad/s
//1���ȶ�Ӧ�ĽǶ�
#define Rad_To_Degree  57.3f                                                  \
            
            

//extern int32_t catch_angle;


enum{
    FAULT   = 0,
    TRUE = 1,
    
    
};


//typedef struct{
//    
//    int32_t compare_value_up;
//    int32_t compare_value_dowm;
//    int sin_ramp_switch;
//    
//}SinRampState;

//extern SinRampState  sin_ramp_state;


    
/*int16_t�����޷�����*/
extern int16_t Int16Constrain(int16_t Value, int16_t minValue, int16_t maxValue);
/*int32_t�����޷�����*/
extern int32_t Int32Constrain(int32_t Value, int32_t minValue, int32_t maxValue);
/*float�����޷�����*/
extern float FloatConstrain(float Value, float minValue, float maxValue);
/*float����ѭ���޷�����*/
extern float LoopFloatConstrain(float Input, float minValue, float maxValue);


void Sin_up_y_calc(void);
void Sin_down_y_calc(void);
void Sin_Up_or_Down_switch(int32_t variation);
int32_t SinRampVariation_1(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);//������
int32_t SinRampVariation_2(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);//�ȸ�����
int32_t SinRampVariation_3(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);//����������ֵ

#endif


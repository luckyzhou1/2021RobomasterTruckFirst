#ifndef __RAMP_H
#define __RAMP_H

#include "mytype.h"


//����������Ƽ�� 0.001s
#define CHASSIS_CONTROL_TIME 0.001f
/*************************************************************************************************/
//��ֵԽ���˲�ϵ����Խ����ֵ�仯Խƽ��
#define CHASSIS_ACCEL_X_NUM 0.499999999f  //����ƽ��
#define CHASSIS_ACCEL_Y_NUM 0.499999999f  //ǰ���˶�

//#define CHASSIS_ACCEL_X_NUM 0.0733333333f  //����ƽ��
//#define CHASSIS_ACCEL_Y_NUM 0.4899999999f  //ǰ���˶�

/************************************************************************************************/


//���ڵ���һ���˲�����
typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;


typedef struct{
    
    int32_t compare_value_up;
    int32_t compare_value_dowm;
    int sin_ramp_switch;
    
}SinRampState;

extern SinRampState Sin_Test;


extern first_order_filter_type_t chassis_cmd_slow_set_vx;
extern first_order_filter_type_t chassis_cmd_slow_set_vy;


//һ���˲���ʼ��
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//һ���˲�����
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);

void ChassisRampInit(void);


int32_t SinRampCalc(SinRampState *sin_state, int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);
int32_t SinRampCalc_Gimbal(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);

void SinRampInit(SinRampState *sin_state);

//void SinTest(void);

#endif

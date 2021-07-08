#ifndef __RAMP_H
#define __RAMP_H

#include "mytype.h"


//底盘任务控制间隔 0.001s
#define CHASSIS_CONTROL_TIME 0.001f
/*************************************************************************************************/
//数值越大滤波系数就越大，数值变化越平缓
#define CHASSIS_ACCEL_X_NUM 0.499999999f  //左右平移
#define CHASSIS_ACCEL_Y_NUM 0.499999999f  //前后运动

//#define CHASSIS_ACCEL_X_NUM 0.0733333333f  //左右平移
//#define CHASSIS_ACCEL_Y_NUM 0.4899999999f  //前后运动

/************************************************************************************************/


//用于底盘一阶滤波启动
typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;


typedef struct{
    
    int32_t compare_value_up;
    int32_t compare_value_dowm;
    int sin_ramp_switch;
    
}SinRampState;

extern SinRampState Sin_Test;


extern first_order_filter_type_t chassis_cmd_slow_set_vx;
extern first_order_filter_type_t chassis_cmd_slow_set_vy;


//一阶滤波初始化
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//一阶滤波计算
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);

void ChassisRampInit(void);


int32_t SinRampCalc(SinRampState *sin_state, int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);
int32_t SinRampCalc_Gimbal(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);

void SinRampInit(SinRampState *sin_state);

//void SinTest(void);

#endif

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "pid.h"
//用于测试哨兵时，去掉下面的注释
//#define SKYGUARD_CHASSIS       //哨兵底盘控制

//3508在带有麦轮负载且悬空的情况下的最大转速为430rpm左右，设置最大速度尽量不要超过这个数值
#define CHASSIS_MAXSPEED_RPM            300       //工程底盘电机最大转速，单位：rpm   
#define BACK_CHASSIS_SHIFT_MAXSPEED_RPM    300    //工程底盘SHIFT向后电机最大转速，单位：rpm
#define CHASSIS_SHIFT_MAXSPEED_RPM     450    //工程底盘电机SHIFT最大转速，单位：rpm
#define CHASSIS_VX_MAXSPEED_RPM       100    //底盘左右平移时，底盘电机的最大速度，单位：rpm
#define REDUCTION_RATIO_3508          19.2f   //3508电机减速比
#define CHASSIS_CATCH_MAXSPEED_RPM    30       //工程底盘对位电机最大转速，单位：rpm

/*底盘电机序号*/
enum{
    
    FRON_RIGH_201 = 0, //前右
    FRON_LEFT_202 = 1, //前左
    REAR_LEFT_203 = 2, //后左
    REAR_RIGH_204 = 3, //后右
    
};


/*底盘结构体*/
typedef struct{
    
    int32_t  fr_motor_angle_201; //前右电机
    int32_t  fl_motor_angle_202; //前左电机
    int32_t  rl_motor_angle_203; //后左电机
    int32_t  rr_motor_angle_204; //后右电机
    
    int32_t  fr_motor_rpm_201; //前右电机
    int32_t  fl_motor_rpm_202; //前左电机
    int32_t  rl_motor_rpm_203; //后左电机
    int32_t  rr_motor_rpm_204; //后右电机
    
    //车体坐标系中的速度
    float vx; //左右平移
    float vy; //前后
    float vw; //自转
    
    //车体坐标系中的位移
    float car_vx; 
    float car_vy;
    float car_vw; 
    
    //轮子对应车体坐标系的速度
    float wheel_rad_201; //轮子的转速，单位：rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
    
}chassis_t;
extern char chassis_pid_pos;
extern char chassis_pid_pos;//是否使用底盘位置环标志位，1使用，0不使用
extern char chassis_angle_init_flag;//底盘位置环初始化标志
extern int16_t chassis_pos_delay;//底盘位置移动延时
//extern int16_t chassis_vx_angle_channel, chassis_vy_angle_channel, chassis_vw_angle_channel;
extern pid_t  Moto_Chassis_Pid_Pos[4];  //位置环PID结构体
extern pid_t  Moto_Chassis_Pid_Spd[4];  //速度环PID结构体
extern chassis_t  Chassis;


void ChassisTask(void);
void ChassisDataUpdate(void);
void ChassisPidCalc(void);
void ParamInit(void);
void ChassisDataCanSend(void);
void ChassisPosDataUpdate(int32_t chassis_vx_angle_channel, int32_t chassis_vy_angle_channel, int32_t chassis_vw_angle_channel);

/****************************************测试用****************************************/
void ChassisSpeedTest(void);



/*****************************************END******************************************/



#endif

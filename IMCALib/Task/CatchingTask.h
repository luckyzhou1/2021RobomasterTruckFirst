#include "mytype.h"
#include "pid.h"
#ifndef __LIFTING_TASK
#define __LIFTING_TASK

extern int32_t speed_cha;

extern  int sinsign;


extern int flip_box_delay;
extern int delay_100ms; 
extern int delay_3000ms; 
//extern int Rotational_Delay0;
extern int rotational_delay;
extern int Rotational_Sign;

extern char down_up_sign;//抬升机构位置标志，down为0，up为1

/*新加*/
extern char r_flag;//按键标志位 
extern char v_flag;
extern char f_flag;
extern char b_flag;
extern char z_flag;
extern char x_flag;
extern char c_flag;
extern char g_flag;
extern char ctrl_z_flag;
extern char ctrl_x_flag;
extern char ctrl_c_flag;
extern char ctrl_f_flag;
extern char ctrl_v_flag;
extern char mouse_right_flag;
extern uint16_t catching_reset_delay;//夹取复位所用延时
extern char catching_reset_flag;//夹取复位所用延时标志位
extern char catching_reset_delay_flag;//夹取复位所用延时标志位
extern char in_flag;//第一步夹子复位需要用到的标志位
extern int32_t flip_angle;//夹爪翻转角度(资源岛)
extern char catch_mode;//大小资源岛夹取标志，1为小资源，2为大资源
extern char conversion_delay_flag;
extern int32_t conversion_delay;
extern int32_t lift_island_reset_speed;
extern int32_t catch_island_reset_speed;
extern char lift_island_reset_flag;
extern char ialand_catch_reset;
extern char catching_island_doing_flag;
extern char sin_end_flag;
extern uint16_t catching_island_delay;
extern uint16_t catching_reset_delay;
extern char catching_count;
extern char ialand_catch_reset;//夹取机构(资源岛)复位标志
extern char r_flag;//按键标志位 
extern char swap_speed_flag;
extern int32_t change_angle1;
extern int32_t change_angle2;
extern uint16_t z_group_set_delay;//z组合键消抖所用延时
extern uint16_t x_group_set_delay;//x组合键消抖所用延时
extern uint16_t c_group_set_delay;//c组合键消抖所用延时
extern pid_t  Moto_Lift_Island_Pid_Pos[2];  
extern pid_t  Moto_Lift_Island_Pid_Spd[2];  
extern pid_t  Moto_Catch_Island_Pid_Pos[2];  //资源岛夹取机构PID位置环结构体
extern pid_t  Moto_Catch_Island_Pid_Spd[2];  //资源岛夹取机构PID速度环结构体

void ClipControl(void);
void LiftingIsland(void);
void CatchingIslandControl(void);
void CatchingControlOne2(void);
void CatchingReset(void);

#endif

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

extern char down_up_sign;//̧������λ�ñ�־��downΪ0��upΪ1

/*�¼�*/
extern char r_flag;//������־λ 
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
extern uint16_t catching_reset_delay;//��ȡ��λ������ʱ
extern char catching_reset_flag;//��ȡ��λ������ʱ��־λ
extern char catching_reset_delay_flag;//��ȡ��λ������ʱ��־λ
extern char in_flag;//��һ�����Ӹ�λ��Ҫ�õ��ı�־λ
extern int32_t flip_angle;//��צ��ת�Ƕ�(��Դ��)
extern char catch_mode;//��С��Դ����ȡ��־��1ΪС��Դ��2Ϊ����Դ
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
extern char ialand_catch_reset;//��ȡ����(��Դ��)��λ��־
extern char r_flag;//������־λ 
extern char swap_speed_flag;
extern int32_t change_angle1;
extern int32_t change_angle2;
extern uint16_t z_group_set_delay;//z��ϼ�����������ʱ
extern uint16_t x_group_set_delay;//x��ϼ�����������ʱ
extern uint16_t c_group_set_delay;//c��ϼ�����������ʱ
extern pid_t  Moto_Lift_Island_Pid_Pos[2];  
extern pid_t  Moto_Lift_Island_Pid_Spd[2];  
extern pid_t  Moto_Catch_Island_Pid_Pos[2];  //��Դ����ȡ����PIDλ�û��ṹ��
extern pid_t  Moto_Catch_Island_Pid_Spd[2];  //��Դ����ȡ����PID�ٶȻ��ṹ��

void ClipControl(void);
void LiftingIsland(void);
void CatchingIslandControl(void);
void CatchingControlOne2(void);
void CatchingReset(void);

#endif

#ifndef __RESETTASK_
#define __RESETTASK_
#include "user_lib.h"
extern char lift_island_reset_success;
extern int32_t reset_lift_island_offset_angle;//��λ��̧����ʼ�Ƕ�
extern int32_t reset_pitch_offset_angle;
extern int32_t reset_yaw_offset_angle;
extern char pitch_reset_flag;//PITCH���ٶȻ���λ��־λ
extern char yaw_reset_flag;//PITCH���ٶȻ���λ��־λ
void ResetControl(void);
    
#endif

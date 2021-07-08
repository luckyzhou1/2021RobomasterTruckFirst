#ifndef __RESETTASK_
#define __RESETTASK_
#include "user_lib.h"
extern char lift_island_reset_success;
extern int32_t reset_lift_island_offset_angle;//复位后抬升初始角度
extern int32_t reset_pitch_offset_angle;
extern int32_t reset_yaw_offset_angle;
extern char pitch_reset_flag;//PITCH轴速度环复位标志位
extern char yaw_reset_flag;//PITCH轴速度环复位标志位
void ResetControl(void);
    
#endif

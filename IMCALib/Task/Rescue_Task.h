#ifndef __RESCUETASK_
#define __RESCUETASK_
#include "mytype.h"
void SwipingCard(void);
void RescueingControl(void);
extern int swip_card_ctrl_delay;
extern int rescue_ctrl_delay;
extern int rescue_out_delay;
extern int rescue_in_delay;
extern int KEY_CTRL_A_count;//消抖
extern int KEY_A_count;//消抖
extern uint16_t rescue_delay;//救援按键延时
#endif


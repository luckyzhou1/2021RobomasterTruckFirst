#include "CatchingCurrent_Task.h"
#include "ramp.h"
#include "CanBus_Task.h"
#include "CatchingTask.h"
#include "ChassisControl.h"
#include "GimbalControl.h"
#include "ResetTask.h"
void CatchingCurrent(void)
{
    change_angle1 = SinRampCalc(&Sin_Test, flip_angle, 2, 2);
    
    if((ialand_catch_reset == 1) || (swap_speed_flag == 1))
    {
        pid_calc(&Moto_Catch_Island_Pid_Spd[0], Island_Catch_Motor[0].speed_rpm, catch_island_reset_speed);
        pid_calc(&Moto_Catch_Island_Pid_Spd[1], Island_Catch_Motor[1].speed_rpm, -catch_island_reset_speed);
    }
	else
    {
        pid_calc(&Moto_Catch_Island_Pid_Pos[0], Island_Catch_Motor[0].total_angle, change_angle1);
        pid_calc(&Moto_Catch_Island_Pid_Spd[0], Island_Catch_Motor[0].speed_rpm, Moto_Catch_Island_Pid_Pos[0].pos_out*REDUCTION_RATIO_3508);
        
        pid_calc(&Moto_Catch_Island_Pid_Pos[1], Island_Catch_Motor[1].total_angle, -change_angle1);
        pid_calc(&Moto_Catch_Island_Pid_Spd[1], Island_Catch_Motor[1].speed_rpm, Moto_Catch_Island_Pid_Pos[1].pos_out*REDUCTION_RATIO_3508);
    }
    
   
//		if(RC_UPPER_RIGHT_SW_MID)
//		{
	 SetMotorCurrent1(&hcan2, Moto_Catch_Island_Pid_Spd[0].pos_out, Moto_Catch_Island_Pid_Spd[1].pos_out,  Gimbal[YAW].can_send ,  Gimbal[PITCH].can_send ); 
       
//    }  
//    else 
//		{
//        SetMotorCurrent2(&hcan1, 0, 0, 0, 0); 
//      	SetMotorCurrent1(&hcan2, 0, 0, 0, 0); 
//		} 
}

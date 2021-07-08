#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

void ParamInit(void);

/*��Դ��̧������PIDλ�û�����*/
#define Maxout_Lift_Island_pos 5000
#define MaxIlimit_Lift_Island_pos 10
#define KP_Lift_Island_pos 0.015f
#define KI_Lift_Island_pos 0.0f
#define KD_Lift_Island_pos 0.0f

/*��Դ��̧������PID�ٶȻ�����*/
#define Maxout_Lift_Island_spd 12000
#define MaxIlimit_Lift_Island_spd 500
#define KP_Lift_Island_spd 24.0f
#define KI_Lift_Island_spd 0.05f
#define KD_Lift_Island_spd 2.0f

#endif



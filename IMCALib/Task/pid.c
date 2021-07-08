/*************************************************************************************************************
 * @file			pid.c
 * @brief   		����PID, ����/����ϰ���Խ�get/measure/real/fdb,
 *					����ֵ����һ���set/target/ref
************************************************************************************************************/
  
  
#include "pid.h"
#include <math.h>
#include "user_lib.h"


#define ABS(x)		((x>0)? (x): (-x))  /*ȡ����ֵ*/



/**********************************************************************************************************************
 * @brief   ��ֵ�޷�
 * @param   float *a        �������ݵ�ָ�����
 *          float ABS_MAX   ��ֵ
 * @retval  None
**********************************************************************************************************************/
void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


/**********************************************************************************************************************
 * @brief   PID������ʼ��
 * @param   pid_t* pid                  PID�ṹ��
 *          uint32_t mode               PIDģʽѡ�񣬷�λ��ʽPID������ʽPID
 *          uint32_t maxout             ����޷�
 *          uint32_t intergral_limit    �����޷�
 *          float kp, ki, kd            PID����������
 * @retval  None
**********************************************************************************************************************/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd )
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}


///*************************���Բ�����ֵ******************************/
//static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
//{
//    pid->p = kp;
//    pid->i = ki;
//    pid->d = kd;
//}


/**********************************************************************************************************************
 * @brief   calculate delta PID and position PID
 * @param   pid_t* pid��PID�ṹ��
 *          set: target
 *          get��measure
 * @retval  None
 * @others  PID������ϣ�https://blog.csdn.net/as480133937/article/details/89508034
**********************************************************************************************************************/
float pid_calc(pid_t* pid, float get, float set)
{
    
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	/*set - measure���õ�ƫ��*/
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
		return 0;   /*���ƫ��������ƫ�������PID����*/
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;  /*ƫ����������Χ�ڲ�����������PID����*/
    
    if(pid->pid_mode == POSITION_PID) /*λ��ʽPID����*/
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&(pid->iout), pid->IntegralLimit);  /*�����޷�*/
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);  /*�޶����ֵ�Ĵ�С*/
        pid->last_pos_out = pid->pos_out;	/*update last time*/ 
    }
    else if(pid->pid_mode == DELTA_PID) /*����ʽPID����*/
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit); /*�����޷�*/
        pid->delta_u = pid->pout + pid->iout + pid->dout; /*��������ֵ*/
        pid->delta_out = pid->last_delta_out + pid->delta_u; /*��������ʽ���*/
        abs_limit(&(pid->delta_out), pid->MaxOutput); /*����޷�*/
        pid->last_delta_out = pid->delta_out;	/*update last time*/
    }
    
    /*��������*/
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out; /*PID���*/
	
}



/**********************************************************************************************************************
 * @brief   PID����ĳ�ʼ��
 * @param   pid_t* pid                  PID�ṹ��
 *          uint32_t mode               PIDģʽѡ�񣬷�λ��ʽPID������ʽPID
 *          uint32_t maxout             ����޷�
 *          uint32_t intergral_limit    �����޷�
 *          float kp, ki, kd            PID����������
 * @retval  None
**********************************************************************************************************************/
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init; /*��pid_param_init�������׵�ַ��ֵ��ָ�����f_param_init*/
//    pid->f_pid_reset = pid_reset;
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}


#ifndef __pid_H
#define __pid_H


#include "stm32f4xx_hal.h"


enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};


/*PID�ṹ��*/
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];	//Ŀ��ֵ��NOW, LAST, LLAST
    float get[3];	//����ֵ
    float err[3];	//���
	
    
    float pout;		//P���				
    float iout;		//I���					
    float dout;		//D���		
    
    float pos_out;			//����λ��ʽ������� pos_out = pout + iout + dout
    float last_pos_out;		//�ϴ�λ��ʽ���
    float delta_u;			//��������ֵ
    float delta_out;		//��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;   //�ϴ�����ʽ���
    
    float max_err;              //���ƫ��
    float deadband;				//err < deadband return
    uint32_t pid_mode;          //PIDģʽ����λ��ʽ������ʽ
    uint32_t MaxOutput;			//����޷�
    uint32_t IntegralLimit;		//�����޷�
    
    void (*f_param_init)(struct __pid_t *pid,  /*����PID������ʼ���ĺ���ָ��*/
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
//    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);   //pid�����޸�

}pid_t;


/*����PID�ṹ���ʼ������*/
void PID_struct_init(pid_t* pid, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd);
/*PID���㺯��*/
float pid_calc(pid_t* pid, float fdb, float ref);

		
#endif


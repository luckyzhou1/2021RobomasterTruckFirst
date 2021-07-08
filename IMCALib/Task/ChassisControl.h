#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "pid.h"
//���ڲ����ڱ�ʱ��ȥ�������ע��
//#define SKYGUARD_CHASSIS       //�ڱ����̿���

//3508�ڴ������ָ��������յ�����µ����ת��Ϊ430rpm���ң���������ٶȾ�����Ҫ���������ֵ
#define CHASSIS_MAXSPEED_RPM            300       //���̵��̵�����ת�٣���λ��rpm   
#define BACK_CHASSIS_SHIFT_MAXSPEED_RPM    300    //���̵���SHIFT��������ת�٣���λ��rpm
#define CHASSIS_SHIFT_MAXSPEED_RPM     450    //���̵��̵��SHIFT���ת�٣���λ��rpm
#define CHASSIS_VX_MAXSPEED_RPM       100    //��������ƽ��ʱ�����̵��������ٶȣ���λ��rpm
#define REDUCTION_RATIO_3508          19.2f   //3508������ٱ�
#define CHASSIS_CATCH_MAXSPEED_RPM    30       //���̵��̶�λ������ת�٣���λ��rpm

/*���̵�����*/
enum{
    
    FRON_RIGH_201 = 0, //ǰ��
    FRON_LEFT_202 = 1, //ǰ��
    REAR_LEFT_203 = 2, //����
    REAR_RIGH_204 = 3, //����
    
};


/*���̽ṹ��*/
typedef struct{
    
    int32_t  fr_motor_angle_201; //ǰ�ҵ��
    int32_t  fl_motor_angle_202; //ǰ����
    int32_t  rl_motor_angle_203; //������
    int32_t  rr_motor_angle_204; //���ҵ��
    
    int32_t  fr_motor_rpm_201; //ǰ�ҵ��
    int32_t  fl_motor_rpm_202; //ǰ����
    int32_t  rl_motor_rpm_203; //������
    int32_t  rr_motor_rpm_204; //���ҵ��
    
    //��������ϵ�е��ٶ�
    float vx; //����ƽ��
    float vy; //ǰ��
    float vw; //��ת
    
    //��������ϵ�е�λ��
    float car_vx; 
    float car_vy;
    float car_vw; 
    
    //���Ӷ�Ӧ��������ϵ���ٶ�
    float wheel_rad_201; //���ӵ�ת�٣���λ��rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
    
}chassis_t;
extern char chassis_pid_pos;
extern char chassis_pid_pos;//�Ƿ�ʹ�õ���λ�û���־λ��1ʹ�ã�0��ʹ��
extern char chassis_angle_init_flag;//����λ�û���ʼ����־
extern int16_t chassis_pos_delay;//����λ���ƶ���ʱ
//extern int16_t chassis_vx_angle_channel, chassis_vy_angle_channel, chassis_vw_angle_channel;
extern pid_t  Moto_Chassis_Pid_Pos[4];  //λ�û�PID�ṹ��
extern pid_t  Moto_Chassis_Pid_Spd[4];  //�ٶȻ�PID�ṹ��
extern chassis_t  Chassis;


void ChassisTask(void);
void ChassisDataUpdate(void);
void ChassisPidCalc(void);
void ParamInit(void);
void ChassisDataCanSend(void);
void ChassisPosDataUpdate(int32_t chassis_vx_angle_channel, int32_t chassis_vy_angle_channel, int32_t chassis_vw_angle_channel);

/****************************************������****************************************/
void ChassisSpeedTest(void);



/*****************************************END******************************************/



#endif

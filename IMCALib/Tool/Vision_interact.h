#ifndef _VISION_INTERACT_H
#define _VISION_INTERACT_H
#include "stm32f4xx_hal.h"
#include "mytype.h"

/*--------��λ��Э��--------*/
//���ڹٷ�����ϵͳ���Ӿ�ͨѶ��ͨѶЭ���һЩע��
//�����Ӿ���ͨѶЭ��Ϊ�׵�ַΪ0XA5��SEQ֡���Ϊ2����Ϊ��ʱΪ�������ͣ����������ݳ��ȶ�
//Ҫע�����CRC8У��Ϊһ�ֽڣ�CRC16Ϊ���ֽڡ�CRC8��ͷУ�飬CRC16������������У��
//---header-------------------------------------//---data----------------//------tail--------//
//--�׵�ַ---֡���---ģʽѡ��-----CRC8У��λ--//---����������λ����--//---CRC16У��λ----//
//--0XA5-----2--------ռһ�ֽ�----ռһ�ֽ�----//---ռ���ֽ�------------//----ռ���ֽ�------//
/*--ע��--*/
//д�� CRC8֡ͷУ���� Append_CRC8_Check_Sum(param1,param2)
//param1 Ϊ��Ҫд������飬param2ΪCRC8д�������ݳ��ȡ���Ϊ��������0XA5��SEQ֡���λ֮������ݳ���Ϊ����crc8У����д��[2]��
//д�� CRC16����У���� Append_CRC16_Check_Sum(param1,param2)
//param1 Ϊ��Ҫд������飬param2Ϊд�����������ݳ��ȡ�ͬ��
//�Ӿ�����������16λ����ƫ�����ݸߵͰ�λ��ת��Ҫ����һ��ת����
#define    PROTOCOL_TEST  2

#define    NOW_t            1
#define    LAST_t           0

#define    SE_BUFFER_SIZE 20  //�Ӿ��������ݻ����ֽ�
#define    RC_BUFFER_SIZE 100 //�Ӿ��������ݻ����ֽ�
//��ʼ�ֽڣ� Ϊ0XA5
#define    VIOSN_SOF 0XA5
//֡���
#define    VISON_SEQ 2

//Э��֡����
#define    VISON_LEN_HAEDER 3 //֡ͷ��
#define    VISON_LEN_PACKED 14//���ݰ�����
//����ģʽѡ��
#define    FeedBack_px     1  //����ƫ���
#define    FeedBack_angle  2  //�Ƕ�ƫ���


//��λ�����սṹ��
typedef __packed struct
{
	//֡ͷ
	uint8_t SOF;//�׵�ַ
	uint8_t seq; //֡���
	uint8_t model;//ģʽ
	uint8_t crc8;//CRC8У��λ
	
	//����
//	float visionYawData;//YAW������ƫ��
//	float visionPitchData;//PITCH������ƫ��
    float visionMineflag;//�Զ���λȡ���־λ
    float visionAngleview;//�ӽ��л���־λ
	
	
	//����β
	uint16_t crc16;//CRC16У��λ
}stoVisonRecvData_t;

//��λ�����ͽṹ��,֡ͷ�����ݶ�֡βУ��ֿ�����Ϊ�ڴ������ݼ���ʱ����Ҫ�ֽڶ���
typedef  struct//ֻ��������
{
  //֡ͷ
	uint8_t SOF;//�׵�ַ
	uint8_t datatype; //������ 0x01 ����״̬��Ϣ 0x02 �ط���������Ϣ
	uint8_t crc8;//CRC8У��λ
}traVisonHeader_t;
typedef  struct
{
	uint8_t robotColor;//װ�װ���ɫʶ�� 0x01 ��װ�װ� 0x02 ��ɫװ�װ�
	uint8_t Mode;//ģʽ�л� 0x01 ���飬 0x02С�� 0x03���
	
	/***���ݵ�λ��ǰ***/
	int32_t visionFps;		//֡���ڵ�11��12λ
	int32_t visonYawData;//YAW������ƫ���ڵ�7��8λ
	int32_t visonPitchData;//PITCH������ƫ���ڵ�9��10λ
	
	
	uint16_t visionDistance;	//����ƫ���ڵ�13��14λ
		//����β
	uint16_t crc16;//CRC16У��λ�ڵ�13��14λ
	
}traVisonSendData_t;



//��ʽת��������"�������Ա���������ڴ�"
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FloatTrans;

//���������������ȫ�ֱ���
extern uint8_t vision_view_angle_flag;
extern uint8_t vision_send_flag;//�Ӿ���ʱ���ͱ�־λ

extern stoVisonRecvData_t  VisonRecvData;//�Ӿ����սṹ��




extern uint8_t Rx_Buffer[RC_BUFFER_SIZE];//���ջ�������
extern uint8_t Visoin_Buffer[RC_BUFFER_SIZE];//�Ӿ���������

extern traVisonSendData_t VisonSendData;//��������
extern stoVisonRecvData_t  VisonRecvData;

extern float  YawLast_T;
extern float  YawPreSpeed;
extern uint32_t count_time;//֡����
extern uint32_t time_fps;//TIM3����ֵ100us
extern uint8_t SEND_FALG;//���ݷ��ͱ�־
/***��ϵͳ��õ�ǰ��������ʱ��***/
uint32_t GetTickClock(void);
/*--------�Ӿ����ݶ�������--------*/
void sendVisionData(uint8_t value);//�����Ӿ�����
void vision_connect(uint8_t *RecvUsartData);//�����Ӿ�����
/*--------�Ӿ�����ƫ����º���--------*/
float Vision_Yaw_Error(float *Yaw_Error);
void Vision_Pitch_Error(float *Pitch_Error);
/*--------�������Ͳ�������--------*/
void Float_to_Byte(float *target,unsigned char *buf,unsigned char beg);
void rev_shrort_data(void);
/*--------����֡����--------*/
uint16_t fps_count(void);//
/*--------����DMA����--------*/
void HAL_UART3_Receive_IT_IDLE(void);//����3DMA����
/*-------�Ӿ����ܺ���--------*/
bool_t Vision_UpDate(void);
void Vision_UpDate_Clean(void);
#if  PROTOCOL_TEST==1

extern PC_interact_Rc_data_set PC_interact_Rc_data;//����3�鸡����
extern u8 usart1_Recv_End_flag;//������ɱ�־λ
extern uint8_t seq;//֡���

typedef struct//�������ݽṹ��
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}PC_interact_data_set;

typedef struct//�������ݽṹ��
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}PC_interact_Rc_data_set;

void PC_interact_data_send_test(float Data_a,float Data_b, float Data_c);
void PC_interact_Rc_data_test(void);
void PC_interact_Rc_data_Replay(void);//����1��������

#endif


#endif

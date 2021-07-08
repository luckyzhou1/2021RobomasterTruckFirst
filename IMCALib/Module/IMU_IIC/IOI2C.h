#ifndef __IOI2C_H__
#define __IOI2C_H__

#include "stm32f4xx_hal.h"
#include "main.h"
#include "mytype.h"
#include "REG.h"


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014 
#define GPIOJ_ODR_ADDr    (GPIOJ_BASE+20) //0x40022414
#define GPIOK_ODR_ADDr    (GPIOK_BASE+20) //0x40022814

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
#define GPIOJ_IDR_Addr    (GPIOJ_BASE+16) //0x40022410 
#define GPIOK_IDR_Addr    (GPIOK_BASE+16) //0x40022810 


#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

#define PJout(n)   BIT_ADDR(GPIOJ_ODR_Addr,n)  //��� 
#define PJin(n)    BIT_ADDR(GPIOJ_IDR_Addr,n)  //����

#define PKout(n)   BIT_ADDR(GPIOK_ODR_Addr,n)  //��� 
#define PKin(n)    BIT_ADDR(GPIOK_IDR_Addr,n)  //����

/**************************************************�û�IO��������******************************************************************/
/*IIC_SDA���ź궨������*/
#define GPIOX    GPIOB
#define PINX     7


/*IIC��SDA���ŵ�IO��������*/
#define SDA_IN()  {GPIOX->MODER&=~(3<<(PINX*2));GPIOX->MODER|=0<<PINX*2;} //����ģʽ
#define SDA_OUT() {GPIOX->MODER&=~(3<<(PINX*2));GPIOX->MODER|=1<<PINX*2;} //���ģʽ

/*IIC��SDA���ź�SCL���ŵ�IO��������*/	 
#define IIC_SCL    PBout(6) //SCL�����
#define IIC_SDA    PBout(7) //SDA�����
#define READ_SDA   PBin(7)  //SDA������ 

/***************************************************END****************************************************************************/


typedef struct 
{
	float a[3];			//���ٶ�
	float w[3];			//���ٶ�
	float h[3];			//�ų�
	float Angle[3];		//�Ƕ�
}stuff;

extern unsigned char chrTemp[30];
extern stuff ImuData;

/*IIC���в�������*/
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

/*IICд�뺯��*/
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);

void ShortToChar(short sData,unsigned char cData[]);
short CharToShort(unsigned char cData[]);
void Get_Imu_Data(stuff *s); /*��ȡIMU����*/


#endif

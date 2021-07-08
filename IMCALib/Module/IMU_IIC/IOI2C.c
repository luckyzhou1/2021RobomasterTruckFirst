/******************************************************************************
*������:��ȡ������ģ��
����	Get_Imu_Data(&ImuData);
���	ģ���������ݣ�JY901/HWT101������Ư�Ƹ��;��ȸ���
IIC�ӿ�	 IIC_SCL    PBout(8) 
		 IIC_SDA    PBout(9) 	 
˵��	ʹ������ֲIOI2C.c��IOI2C.h���ļ�������Ҫʹ�õ��ļ��������ͷ�ļ���
		ֱ�ӵ��ú���Get_Imu_Data();ʹ�ã�����һ�κ��������ݸ���һ�Σ�
		���߷��ڶ�ʱ�����ڸ�����Ƶ���ϸ������ݡ�
*******************************************************************************/ 



#include "IOI2C.h"

unsigned char chrTemp[30];
stuff ImuData;

//JY901ģ�����ݶ�ȡ����
void Get_Imu_Data(stuff *s)
{
	
	//IIC��ȡ����  IIC��AX�� 0x34 X����ٶȿ�ʼ��ȡ���ݣ�һֱ��0x3f Z��Ƕ�
	//24���ֳ�����ȡ�������ݷָ�λ�͵�λ����ȡ��12���Ĵ���������
	IICreadBytes(0x50, AX, 24,&chrTemp[0]);
	
	s->a[0] = (float)CharToShort(&chrTemp[0])/32768*16;          //������ٶ� ��x��y��z�ᣩ
	s->a[1] = (float)CharToShort(&chrTemp[2])/32768*16;
	s->a[2] = (float)CharToShort(&chrTemp[4])/32768*16;
    
	s->w[0] = (float)CharToShort(&chrTemp[6])/32768*2000;		  //������ٶ�
	s->w[1] = (float)CharToShort(&chrTemp[8])/32768*2000;
	s->w[2] = (float)CharToShort(&chrTemp[10])/32768*2000;
    
	s->h[0] = CharToShort(&chrTemp[12]);							//����ų�
	s->h[1] = CharToShort(&chrTemp[14]);
	s->h[2] = CharToShort(&chrTemp[16]);
    
	s->Angle[0] = (float)CharToShort(&chrTemp[18])/32768*180;		//����Ƕ�
	s->Angle[1] = (float)CharToShort(&chrTemp[20])/32768*180;
	s->Angle[2] = (float)CharToShort(&chrTemp[22])/32768*180;
    
}

	
void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}


short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
} 


void Delay(u32 count)//���ڲ���400KHzIIC�ź�����Ҫ����ʱ
{
	count = count*10;
	while (count--);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
void IIC_Init(void)
{			
	//SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	
	Delay(5);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	
	Delay(5);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	
		Delay(5);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	
		Delay(5);							   	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	SDA_IN();      //SDA����Ϊ����  
 	IIC_SDA=1;
		Delay(5);	  
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
		Delay(5);
	}  
	IIC_SCL=1;
	Delay(5); 
	IIC_SCL=0;//ʱ�����0  
	return 0;  
} 


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
		Delay(5);
	IIC_SCL=1;
		Delay(5);
	IIC_SCL=0;
}
	

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	
		Delay(5);
	IIC_SCL=1;
		Delay(5);
	IIC_SCL=0;
}					 				     


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
		SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
			
		Delay(2);   
		IIC_SCL=1;
		Delay(5);
		IIC_SCL=0;	
		Delay(3);
    }	 
} 	 
   

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        
		Delay(5);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		
		Delay(5); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
	
}

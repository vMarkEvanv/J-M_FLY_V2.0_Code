#include "ICM_42688_P.h"
#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "sys.h"
#include "FLY_Control_Logic.h"
#include "math.h"
#define pi 3.1415926

#define RAD2DEG 57.295779513

ATTU attu;
ATTU attu_loss;
unsigned char ICM_42688_Addr_AD0_LOW_READ = 0xD1;   //AD0�͵�ƽ��ַ�Ķ�
unsigned char ICM_42688_Addr_AD0_HIGH_READ = 0xD3;	 //AD0�ߵ�ƽ��ַ�Ķ�
unsigned char ICM_42688_Addr_AD0_LOW_WRITE = 0xD0;	 //AD0�͵�ƽ��ַ��д
unsigned char ICM_42688_Addr_AD0_HIGH_WRITE = 0xD2; //AD0�ߵ�ƽ��ַ��д

GYRO Gyro_Get;
GYRO Gyro_Last;
ACC Acc_Get;
TEMP Temp;
ACC Acc_Last;
extern FIX_VALUE FIXED_VALUE;

void IIC_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}
/**************************************************/
/*��������ICM_Port_Init;***********************/
/*���ܣ���ʼ��Ӳ��IICͨ��;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void ICM_Port_Init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C1->CR1 |= 0x8000;  // ?????BUSY
	I2C1->CR1 &= ~0x8000;
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 500000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C1, &I2C_InitStructure);
	
	I2C_Cmd(I2C1, ENABLE);				 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
}






/******************************************************************/
/*��������ICM_IIC_WRITE_BYTE;***************************************/
/*���ܣ�дһ���ֽ�;*************/
/*���룺RA���Ĵ�����ַ data_byte:����;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
unsigned char ICM_IIC_WRITE_BYTE(unsigned char RA, unsigned char data_byte){
	
	I2C_GenerateSTART(I2C1, ENABLE);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, ICM_42688_Addr_AD0_LOW_WRITE, I2C_Direction_Transmitter);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1, RA);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C1, data_byte);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0;
}

/******************************************************************/
/*��������ICM_IIC_READ_BYTE;***************************************/
/*���ܣ���һ���ֽ�;*************/
/*���룺RA���Ĵ�����ַ;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
unsigned char ICM_IIC_READ_BYTE(unsigned char RA, unsigned char *data){
		uint8_t Data;
	
	I2C_GenerateSTART(I2C1, ENABLE);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, ICM_42688_Addr_AD0_LOW_WRITE, I2C_Direction_Transmitter);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1, RA);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C1, ENABLE);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, ICM_42688_Addr_AD0_LOW_WRITE, I2C_Direction_Receiver);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C1);
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	*data = Data;
	return 0;
}

/******************************************************************/
/*��������ICM_INIT;***************************************/
/*���ܣ�ICMоƬ��ʼ��;*************/
/*���룺��;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
unsigned char ICM_INIT(){
	if(ICM_IIC_WRITE_BYTE(DEVICE_CONFIG,0x00)) return 1;//Software reset configuration and SPI mode selection
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(DRIVE_CONFIG,0x05)) return 2;//Control the communication speed(I guess)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_CONFIG,0x02)) return 3;//interrupt settings
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(PWR_MGMT0,0x0F)) return 4;//power register of sensors(it won't working if we don't turn it on)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_CONFIG1,0x00)) return 5;//this register is to set the interrupt port's Interrupt pulse duration (more details on datasheet)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_SOURCE0,0x08)) return 6;//setting interrupt port's interrupt source
	delay_ms(50);
	if(ICM_Gyroscope_INIT()) return 7;//�����ǳ�ʼ��
	delay_ms(50);
	if(ICM_ACC_INIT()) return 8;//���ٶȼƳ�ʼ��
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(SELF_TEST_CONFIG,0x1F)) return 9;//�Լ�
	delay_ms(50);
	return 0;
}

/******************************************************************/
/*��������ICM_Gyroscope_INIT;***************************************/
/*���ܣ�ICM�����ǳ�ʼ��;*************/
/*���룺��;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
unsigned char ICM_Gyroscope_INIT(){
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG0,0x06)) return 1;//���������ʺ�ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG1,0x1A)) return 1;//����������˲�����
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(GYRO_ACCEL_CONFIG0,0x11)) return 1;//���������Ǻͼ��ٶȼƵĵ�ͨ�˲�������
	delay_ms(50);
	return 0;
}

/******************************************************************/
/*��������ICM_ACC_INIT;***************************************/
/*���ܣ�ICM���ٶȼƳ�ʼ��;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
unsigned char ICM_ACC_INIT(){
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG0,0x66)) return 1;//���������ʺ�ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG1,0x0D)) return 1;//����������˲�����
	delay_ms(50);
	return 0;
	
}

double myabs(double f){
	if(f<0){
		return -f;
	}
	return f;
}
/******************************************************************/
/*��������GYRO_ACC_TEMP_GET;***************************************/
/*���ܣ���ȡ�����ǣ����ٶȼƣ��¶�����;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
unsigned char GYRO_ACC_TEMP_GET(){
	unsigned char temp = 0;
	short Counting_Temp = 0;
	
	Acc_Last.X = Acc_Get.X;
	Acc_Last.Y = Acc_Get.Y;
	Acc_Last.Z = Acc_Get.Z;
	
	//�¶ȶ�ȡ
	if(ICM_IIC_READ_BYTE(TEMP_DATA1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(TEMP_DATA0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Temp.T = (Counting_Temp / 132.48) + 25;
	
	//X�������Ƕ�ȡ ��2000dps
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.X = (Counting_Temp*1.0)/32767.0*2000;
	
	//Y�������Ƕ�ȡ ��2000dps
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Y = (Counting_Temp*1.0)/32767.0*2000;
	
	//Z�������Ƕ�ȡ ��2000dps
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Z = (Counting_Temp*1.0)/32767.0*2000;
	
	//X����ٶȼƶ�ȡ ��16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.X = (Counting_Temp*1.0)/32767.0*2.0*9.8;
	
	//Y����ٶȼƶ�ȡ ��16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Y = (Counting_Temp*1.0)/32767.0*2.0*9.8;
	
	//Z����ٶȼƶ�ȡ ��16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Z = (Counting_Temp*1.0)/32767.0*2.0*9.8;
	
	//����������
//	Gyro_Get.X -= FIXED_VALUE.X;
//	Gyro_Get.Y -= FIXED_VALUE.Y;
//	Gyro_Get.Z -= FIXED_VALUE.Z;
	Gyro_Get.X -= 101.38;
	Gyro_Get.Y -= 110.84;
	Gyro_Get.Z -= 118.94;
	//printf("%.2f,%.2f,%.2f\r\n",Gyro_Get.X,Gyro_Get.Y,Gyro_Get.Z);
	//�˲�
	if(myabs(Gyro_Get.X) <= 0.3){Gyro_Get.X = 0;}
	if(myabs(Gyro_Get.Y) <= 0.3){Gyro_Get.Y = 0;}
	if(myabs(Gyro_Get.Z) <= 0.3){Gyro_Get.Z = 0;}
	
	
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",myabs(Gyro_Get.X - Gyro_Last.X),myabs(Gyro_Get.Y - Gyro_Last.Y),myabs(Gyro_Get.Z - Gyro_Last.Z),Gyro_Get.X,Gyro_Get.Y,Gyro_Get.Z);

	return 0;
}







/******************************************************************/
/*��������Kalman_Filter_X;***************************************/
/*���ܣ�pitch�Ŀ������˲�;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
void Kalman_Filter_X()
{
	static float Accel;
	static float Gyro;
	
	static float angle_dot;
	static float Q_angle=0.001;// ����������Э����
	static float Q_gyro=0.003;//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
	static float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
	static float dt=0.05;//                 
	static char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Accel=atan2(Acc_Get.Y,Acc_Get.Z)*RAD2DEG;
	Gyro=Gyro_Get.X/16.4;
	

	attu.pitch+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - attu.pitch;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	attu.pitch	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

/******************************************************************/
/*��������Kalman_Filter_Y;***************************************/
/*���ܣ�row�Ŀ������˲�;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
void Kalman_Filter_Y()
{
	static float Accel;
	static float Gyro;
	
	static float angle_dot;
	static float Q_angle=0.001;// ����������Э����
	static float Q_gyro=0.003;//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
	static float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
	static float dt=0.05;//                 
	static char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Accel=-atan2(Acc_Get.X,Acc_Get.Z)*RAD2DEG;
	Gyro=Gyro_Get.Y/16.4;
	

	attu.row+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - attu.row;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	attu.row	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

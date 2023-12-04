#include "stm32f10x.h"                  // Device header
#include "FLY_Control_Logic.h"
#include "ICM_42688_P.h"
#include "bmp280.h"
#include "myiic.h"
#include "UART_Select.h" 
#include "bmp280.h"
#include "sys.h"
#include "math.h"
#include "Motor_Ctrl.h"

extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern ACC Acc_Last;
extern TEMP Temp;
extern BMP_280 bmp280;

PID Pitch_M0_PID;
PID Pitch_M1_PID;
M_PWM m_pwm;
M_PWM pitch_pwm;
M_PWM base_pwm;
M_PWM row_pwm;
M_PWM yaw_pwm;

double Integral_M0=0;
double Integral_M1=0;
double Integral_M2=0;
double Integral_M3=0;

extern ATTU attu;
extern ATTU attu_loss;
FIX_VALUE FIXED_VALUE;

/******************************************************************/
/*��������FLY_BIOS_INIT;***************************************/
/*���ܣ��ɿػ�����������ʼ��;*************/
/*���룺��;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
void FLY_BIOS_INIT(){
	//ָʾ�Ƴ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	PCout(13)=1;
	
	//��̬���г�ʼ��
	ICM_Port_Init();
	ICM_INIT();
	
	//��ѹ�Ƴ�ʼ��
	IIC_Init();
	//while(!bmp280Init());
	
	//ѡ��Դ���ڳ�ʼ��
	UART_Init(115200);
	
	Pitch_M0_PID.Kp = 6;
	Pitch_M0_PID.Ki = 0.09;
	Pitch_M0_PID.Kd = -2.34;
	Pitch_M1_PID.Kp = 5;
	Pitch_M1_PID.Ki = 0.09;
	Pitch_M1_PID.Kd = -2.34;
	
	attu_loss.pitch=0;
	attu_loss.row = 0;
	attu_loss.yaw = 0;
//	FIXED_VALUE.X =0;
//	FIXED_VALUE.Y =0;
//	FIXED_VALUE.Z =0;
//	
//	double temp_x = 0;
//	double temp_y = 0;
//	double temp_z = 0;
//	
//	int n = 10000;
//	while(n--){
//		GYRO_ACC_TEMP_GET();
//		temp_x += Gyro_Get.X;
//		temp_y += Gyro_Get.Y;
//		temp_z += Gyro_Get.Z;
//	}
//	FIXED_VALUE.X = temp_x/10000.0;
//	FIXED_VALUE.Y = temp_y/10000.0;
//	FIXED_VALUE.Z = temp_z/10000.0;
	
	
// 	while(SD_Init())//��ⲻ��SD��
//	{
//		PCout(13)=~PCout(13);
//	}
	//CH9141_Init();
	//CH9141_EN();
	FLY_PWM_Port_Init();
	base_pwm.M0=200;
	base_pwm.M1=200;
	base_pwm.M2=200;
	base_pwm.M3=200;
	Set_Duty(0,0,0,0);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
}

#define pi 3.1415926
double toDegrees(double radians) {
    return radians * (180.0 / pi);
}
/******************************************************************/
/*��������ICM_Gyroscope_INIT;***************************************/
/*���ܣ�ICM�����ǳ�ʼ��;*************/
/*���룺��;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
void Attitude_Calculate(){
	Kalman_Filter_X();
	Kalman_Filter_Y();
}

/******************************************************************/
/*��������Load_Attu_PID;***************************************/
/*���ܣ�����PID;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
void Load_Attu_PID(double pitch, double row, double yaw, double goal_pitch, double goal_row, double goal_yaw){
	double pitch_loss_temp = 0.0;
	pitch_loss_temp = goal_pitch - pitch;
	m_pwm.M0 = base_pwm.M0 + (int)(Pitch_M0_PID.Kp*pitch_loss_temp + Pitch_M0_PID.Ki*Integral_M0 + Pitch_M0_PID.Kd*(Gyro_Get.X));
	m_pwm.M1 = base_pwm.M1 - (int)(Pitch_M1_PID.Kp*pitch_loss_temp + Pitch_M1_PID.Ki*Integral_M1 + Pitch_M1_PID.Kd*(Gyro_Get.X));
	if(pitch_loss_temp>0){
		Integral_M0 = Integral_M0 + 1;
	}
	else if(pitch_loss_temp<0.05&&pitch_loss_temp>-0.05){
		Integral_M0=0;
	}
	else{
		Integral_M0 = Integral_M0 - 1;
	}
	if(pitch_loss_temp>0){
		Integral_M1 = Integral_M1 + 1;
	}
	else if(pitch_loss_temp<0.05&&pitch_loss_temp>-0.05){
		Integral_M1=0;
	}
	else{
		Integral_M1 = Integral_M1 - 1;
	}
}
	
void TIM4_Interrupt_Init(unsigned int arr, unsigned int psc)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_InternalClockConfig(TIM4);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM4, ENABLE);
	
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		PCout(13) = ~PCout(13);
		GYRO_ACC_TEMP_GET();
		Attitude_Calculate();
		Load_Attu_PID(attu.pitch, attu.row, 0, 0, 0, 0);
		//printf("%d,%d\n",m_pwm.M0,m_pwm.M1);
		Set_Duty(m_pwm.M0,m_pwm.M1,m_pwm.M2,m_pwm.M3);
		
	}
}
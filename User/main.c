#include "stm32f10x.h"                  // Device header
#include "ICM_42688_P.h"
#include "sys.h"
#include "delay.h"
#include "Interrupt.h"

#include "bmp280.h"
#include "usart.h"
#include "FLY_Control_Logic.h"
#include "bmp.h"
#include "Motor_Ctrl.h"


extern PID Pitch_M0_PID;
extern PID Pitch_M1_PID;
extern BMP_280 bmp280;
extern ATTU attu;
extern GYRO Gyro_Get;
int main(void)
{
	delay_init();
	uart_init(420000);
	FLY_BIOS_INIT();
	
	TIM4_Interrupt_Init(50,7200);
	
	
	while(1)
	{
		bmp280GetData(&bmp280.pressure,&bmp280.temperature,&bmp280.asl);
		//printf("%.2f,%.2f,%.2f\n",attu.X,attu.Y,attu.Z);
		printf("%.2f,%.2f,1.0\n",attu.pitch,attu.row,attu.yaw);
		//printf("%.2f,%.2f,%.2f\r\n",Pitch_M0_PID.Kd,Pitch_M0_PID.Kp,attu.pitch);

	}
}



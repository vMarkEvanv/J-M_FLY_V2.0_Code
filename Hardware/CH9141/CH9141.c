#include "CH9141.h"
#include "stm32f10x.h"                  // Device header
#include "sys.h"
#include "stdio.h"	
#include "delay.h"
#include "stdarg.h"	 	 	 
#include "string.h"	 
#include "usart.h"
#include "stdlib.h"

#define CH9141_AT PAout(5)
#define CH9141_SLEEP PAout(4)

#define CH9141_RTS PBin(14)
#define CH9141_CTS PBin(13)

/**************************************************/
/*��������CH9141_Port_Init;***********************/
/*���ܣ���ʼ��Ӳ��IOͨ��;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void CH9141_Port_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//��ʹ������IO PORTBʱ�� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
  GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);						 //PB10,PB11 �����	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTBʱ�� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
	PBout(14)=1;
	CH9141_SLEEP = 0;
	CH9141_AT = 0; 
 }


 /**************************************************/
/*��������CH9141_Init;***********************/
/*���ܣ���ʼ����������;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void CH9141_Init(){
	CH9141_SLEEP = 1;
	CH9141_AT = 1; 
 }
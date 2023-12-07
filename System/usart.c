#include "sys.h"
#include "usart.h"	  
#include "FLY_Control_Logic.h"

extern PID Pitch_M0_PID;
extern PID Pitch_M1_PID;

unsigned char lut[256];
unsigned char poly = 0xD5;//У�鳣��

u8 Res=0;
u8 RxBuf[25];
int CNT=0;
void CRC_INIT(){
	unsigned char crc;
	unsigned char crc_temp;
	int shift = 0;
	for(int idx=0;idx<256;++idx){
		crc = idx;
		while(shift < 8){
			shift += 1;
			if(crc & 0x80){
				crc_temp = poly;
			}
			else{
				crc_temp = 0;
			}
		}
    crc = (crc << 1) ^ crc_temp;
    lut[idx] = crc & 0xff;
    idx += 1;
	}
}


////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

 
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound){
  //GPIO�˿�����
	CRC_INIT();
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	//USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
			Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
			if((USART_RX_STA&0x8000)==0)//����δ���
				{
				if(USART_RX_STA&0x4000)//���յ���0x0d
					{
					if(Res!=0x0a)
					{
						USART_RX_STA=0;
						
					}//���մ���,���¿�ʼ
					else 
					{
						
						USART_RX_STA|=0x8000;	//��������� 
//						if(Res == 0xC8){
//							for(int i=0;i<24;i++){
//								RxBuf[i] = 0;
//							}
//							CNT = 0;
//							RxBuf[CNT] = Res;
//							
//						}
//						else{
//							CNT++;
//							RxBuf[CNT] = Res;
//							printf("%d\r\n",RxBuf[CNT]);
//						}
					}
					}
				else //��û�յ�0X0D
					{	
					if(Res==0x0d)USART_RX_STA|=0x4000;
					else
						{
						USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
						USART_RX_STA++;
						if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
						}		 
					}	
						
					
				}
//				//USART_SendData(USART3,Res);
//				if(Res==0xDD){
//							Pitch_M0_PID.Kd = Pitch_M0_PID.Kd + 0.01;
//							Pitch_M1_PID.Kd = Pitch_M1_PID.Kd + 0.01;
//							
//						}
//				if(Res==0xAA){
//							Pitch_M0_PID.Kd = Pitch_M0_PID.Kd - 0.01;
//							Pitch_M1_PID.Kd = Pitch_M1_PID.Kd - 0.01;
//							//printf("%.2f\n",Pitch_M0_PID.Kd);
//				}
//				if(Res==0xDA){
//							Pitch_M0_PID.Kp = Pitch_M0_PID.Kp + 0.01;
//							Pitch_M1_PID.Kp = Pitch_M1_PID.Kp + 0.01;
//							
//						}
//				if(Res==0xAD){
//							Pitch_M0_PID.Kp = Pitch_M0_PID.Kp - 0.01;
//							Pitch_M1_PID.Kp = Pitch_M1_PID.Kp - 0.01;
//							//printf("%.2f\n",Pitch_M0_PID.Kd);
//				}
     }
		
}




#endif	


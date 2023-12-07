#include "stm32f10x.h"                  // Device header
#include "UART_Select.h"
#include "sys.h"
#include "stm32f10x.h"                  // Device header

//���ڽ��ջ����� 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 			//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
 
 extern u8 RxBuf[];
 extern int CNT;
//ͨ���жϽ�������2���ַ�֮���ʱ������10ms�������ǲ���һ������������.
//���2���ַ����ռ������10ms,����Ϊ����1����������.Ҳ���ǳ���10msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
vu16 USART3_RX_STA=0; 

void SELECT_Port_INIT(){

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
  GPIO_ResetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);		
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
}

/**************************************************/
/*��������UART_Init;***********************/
/*���ܣ���ʼ��ѡ�񴮿�;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void UART_Init(u32 bound){
 
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	// GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //����3ʱ��ʹ��
 
 	USART_DeInit(USART3);  //��λ����3
		 //USART3_TX   PB10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��PB10
   
    //USART3_RX	  PB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��PB11
	
	USART_InitStructure.USART_BaudRate = bound;//������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  
	USART_Init(USART3, &USART_InitStructure); //��ʼ������	3
  
 
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
	
	//ʹ�ܽ����ж�
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�   
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART3_RX_STA=0;		//����
 }

/**************************************************/
/*��������USART3_IRQHandler;***********************/
/*���ܣ�����3�жϷ������;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/

void USART3_IRQHandler(void)
{
	u8 Res;	      
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{	 
		Res =USART_ReceiveData(USART3);	 
		if((USART3_RX_STA&0x8000)==0)//�������һ������,��û�б�����,���ٽ�����������
		{ 
		    if((USART3_RX_STA&0X7FFF)<USART3_MAX_RECV_LEN)	//�����Խ�������
				{
					if(Res!='!')
					{
						USART3_RX_BUF[USART3_RX_STA++]=Res;	//��¼���յ���ֵ
						//printf("%c\r\n",Res);
					}	
          else 
					{
						USART3_RX_STA|=0x8000;	//����Ϣ��������� 
					} 					
				}
				else 
				{
					USART3_RX_STA|=0x8000;	//����Ϣ��������� 
				} 
		}
		if(Res == 0xC8){
							for(int i=0;i<24;i++){
								RxBuf[i] = 0;
							}
							CNT = 0;
							RxBuf[CNT] = Res;
							
						}
						else{
							CNT++;
							RxBuf[CNT] = Res;
							
						}
		USART3_RX_Data();
		//PCout(13)=~PCout(13);
		//USART_SendData(USART1,Res);
	}  				 											 
}  

/**************************************************/
/*��������USART3_RX_Data;***********************/
/*���ܣ�����3����;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void USART3_RX_Data()
{
	u8 len=0;
	if(USART3_RX_STA&0x8000)
		{					   
			len=USART3_RX_STA&0X7FFF;//�õ��˴ν��յ������ݳ���
			USART3_RX_BUF[len]=0;	 	//���������
			
			if(len>USART3_MAX_RECV_LEN-2)
			{
				len=USART3_MAX_RECV_LEN-1;
				USART3_RX_BUF[len]=0;	 	//���������
			}
			
			USART3_RX_BUF[USART3_MAX_RECV_LEN-1]=0x01;
//			u3_printf("%s\r\n",USART3_RX_BUF);
			USART3_RX_STA=0;
		}
 
}
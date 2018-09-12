#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "stdbool.h"
#include "Meteorological.h"

#define RS485_USART  UART4
extern uint8_t ReadyFlag;
#if RS485_USART_RX   		//如果使能了接收   	  
//接收缓存区 	
u8 RS485_RX_BUF[2][2000];  	//接收缓冲,最大2000个字节.
//接收到的数据长度
uint16_t RS485_RX_CNT=0;   

u8 Rs485Meteor_Status=0;
u8 Rs485BufferNumber0=0;
u8 Rs485BufferNumber1=1;
u8 Rs485BufferFinishNumber=0;
u8 Rs485BufferWorkNumber=0;
void UART4_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET)//接收到数据
	{	 
		USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
	  res =USART_ReceiveData(RS485_USART);//;读取接收到的数据USART2->DR
		if(ReadyFlag)
			fifo_in(&NuclearFIFOBuffer,&res,1);

		if(Rs485BufferWorkNumber==Rs485BufferNumber1)
		{
			RS485_RX_BUF[Rs485BufferNumber1][RS485_RX_CNT] = res;
		}
		else 
		{
			RS485_RX_BUF[Rs485BufferNumber0][RS485_RX_CNT] = res;
		}
		RS485_RX_CNT++;
	}
	
	if(USART_GetITStatus(RS485_USART, USART_IT_IDLE) != RESET)
	{
		USART_ClearITPendingBit(RS485_USART, USART_IT_IDLE);
		RS485_USART->DR;
		Rs485Meteor_Status = 1;
		NuclearBufCounter = RS485_RX_CNT;
		RS485_RX_CNT = 0;
		/*记录当前缓存区*/
		Rs485BufferFinishNumber = Rs485BufferWorkNumber;
		if(Rs485BufferWorkNumber==Rs485BufferNumber1)
			 Rs485BufferWorkNumber = Rs485BufferNumber0;
		else 
			 Rs485BufferWorkNumber = Rs485BufferNumber1;
	}	
} 
#endif										 
//初始化IO 串口2
//bound:波特率	  
void RS485_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART2时钟
	
  //串口2引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0复用为UART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1复用为UART4
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA0与GPIOA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0，PA1
	
	//PG8推挽输出，485模式控制  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2
	

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(RS485_USART, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(RS485_USART, ENABLE);  //使能串口 2
	
	USART_ClearFlag(RS485_USART, USART_FLAG_TC);
	
#if RS485_USART_RX	
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);//开启接受中断
	USART_ITConfig(RS485_USART, USART_IT_IDLE,ENABLE);	//开启空闲中断
	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;//抢占优先级10
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =10;		//子优先级10
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif	
	
	RS485_TX_EN=0;				//默认为接收模式	
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
	delay_ms(2);
  for(t=0;t<len;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(RS485_USART,USART_FLAG_TC)==RESET)
		{}			//等待发送结束		
    USART_SendData(RS485_USART,buf[t]); //发送数据
	}
	while(USART_GetFlagStatus(RS485_USART,USART_FLAG_TC)==RESET); //等待发送结束		
	//RS485_RX_CNT=0;	
	RS485_TX_EN=0;				//设置为接收模式	
}
////RS485查询接收到的数据
////buf:接收缓存首地址
////len:读到的数据长度
//void RS485_Receive_Data(u8 *buf,uint16_t *len)
//{
//	u8 rxlen=0;
//	u8 i=0;
//	*len=0;				//默认为0
//	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
//	rxlen = RS485_RX_CNT;
//	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
//	{
//		for(i=0;i<rxlen;i++)
//		{
//			buf[i]=RS485_RX_BUF[i];	
//		}		
//		*len=RS485_RX_CNT;	//记录本次数据长度
//		RS485_RX_CNT=0;		//清零
//	}
//}

////RS485查询接收到的数据长度
//void RS485_Receive_BufferLen(uint16_t *len)
//{
//	u8 rxlen=0;
//	*len=0;				//默认为0
//	delay_ms(26);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
//	rxlen=RS485_RX_CNT;
//	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
//	{
//		*len=RS485_RX_CNT;	//记录本次数据长度
//		RS485_RX_CNT=0;		//清零
//	}
//}


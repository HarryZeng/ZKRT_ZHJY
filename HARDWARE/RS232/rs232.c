#include "sys.h"		    
#include "rs232.h"	 
#include "delay.h"
#include "stdbool.h"
#include "fifo.h"
#include "Meteorological.h"

#if RS232_RX   		//如果使能了接收   	  
//接收缓存区 	
u8 RS232_RX_BUF[2][2000];  	//接收缓冲,最大2000个字节.
//接收到的数据长度
uint16_t RS232_RX_CNT=0;   
u8 Meteor_Status=0;
u8 BufferNumber0=0;
u8 BufferNumber1=1;
u8 BufferFinishNumber=0;
u8 BufferWorkNumber=0;
void USART2_IRQHandler(void)
{
	u8 res;	    
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
		{	 	
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			res = USART_ReceiveData(USART2);//;读取接收到的数据USART2->DR

				if(BufferWorkNumber==BufferNumber1)
				{
					RS232_RX_BUF[BufferNumber1][RS232_RX_CNT] = res;
				}
				else 
				{
					RS232_RX_BUF[BufferNumber0][RS232_RX_CNT] = res;
				}
				
				RS232_RX_CNT++;

		} 
		if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
		{
			USART_ClearITPendingBit(USART2, USART_IT_IDLE);
			USART2->DR;
			Meteor_Status = 1;
			MeteorBufCounter = RS232_RX_CNT;
			RS232_RX_CNT = 0;
			/*记录当前缓存区*/
			BufferFinishNumber = BufferWorkNumber;
			if(BufferWorkNumber==BufferNumber1)
				 BufferWorkNumber = BufferNumber0;
			else 
				 BufferWorkNumber = BufferNumber1;
		}		
}

#endif										 
//初始化IO 串口2
//bound:波特率	  
void RS232_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
  //串口2引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD5复用为USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD6复用为USART2
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD5与GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA2，PA3
	
   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART2, ENABLE);  //使能串口 2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if RS232_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接受中断
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);	//开启空闲中断
	
	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif	
	
	RS232_TX_EN=0;				//默认为接收模式	
}

////RS232查询接收到的数据
////buf:接收缓存首地址
////len:读到的数据长度
//void RS232_Receive_Data(u8 *buf,uint16_t *len)
//{
//	u8 rxlen=RS232_RX_CNT;
//	u8 i=0;
//	*len=0;				//默认为0
//	//delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
//	if(rxlen==RS232_RX_CNT&&rxlen)//接收到了数据,且接收完成了
//	{
//		for(i=0;i<rxlen;i++)
//		{
//			buf[i]=RS232_RX_BUF[i];	
//		}		
//		*len=RS232_RX_CNT;	//记录本次数据长度
//		RS232_RX_CNT=0;		//清零
//	}
//}

////RS232查询接收到的数据长度

//void RS232_Receive_BufferLen(uint16_t *len)
//{
//	u8 rxlen=0;
//	*len=0;				//默认为0
//	//delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
//	rxlen=RS232_RX_CNT;
//	if(rxlen==RS232_RX_CNT&&rxlen)//接收到了数据,且接收完成了
//	{
//		*len=RS232_RX_CNT;	//记录本次数据长度
//		RS232_RX_CNT=0;		//清零
//	}
//}



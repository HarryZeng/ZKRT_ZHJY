#ifndef __RS232_H
#define __RS232_H			 
#include "sys.h"	 								  

extern u8 RS232_RX_BUF[2][2000]; 		//接收缓冲,最大1000个字节
extern uint16_t RS232_RX_CNT;   			//接收到的数据长度
extern u8 Meteor_Status;
extern u8 BufferFinishNumber;
//模式控制
#define RS232_TX_EN		PGout(8)	//485模式控制.0,接收;1,发送.
//如果想串口中断接收，设置EN_USART2_RX为1，否则设置为0
#define RS232_RX 	1			//0,不接收;1,接收.

														 
void RS232_Init(u32 bound);
void RS232_Send_Data(u8 *buf,u8 len);
void RS232_Receive_Data(u8 *buf,uint16_t *len);	
void RS232_Receive_BufferLen(uint16_t *len);
#endif	   

















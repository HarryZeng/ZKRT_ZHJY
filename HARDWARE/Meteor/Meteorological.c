/**
  ******************************************************************************
  * @file    Meteorological.c 
  * @author  ZKRT
  * @version V1.0
  * @date    12-March-2018
  * @brief   
	*					 + (1) init
	*                       
  ******************************************************************************
  * @attention
  *
  * ...
  *
  ******************************************************************************
  */

#include "Meteorological.h"
#include "delay.h" 								  
#include "usart.h"
#include "rs485.h"
#include "fifo.h"
#include "led.h"

u8      ReceiveBuf[2000];
fifo 		MeteorFIFOBuffer;
fifo 		NuclearFIFOBuffer;
u16 		MeteorBufCounter;
u16 		NuclearBufCounter;

//uint8_t  NuclearBufCounter[NUCLEAR_Buffer_SIZE]; 


/**
  * @brief   int to char
  * @param  None
  * @retval None
  */
char* itoa(int num,char*str,int radix)
{		/*索引表*/
		char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		unsigned unum;/*中间变量*/
		char temp;
		int i=0,j,k;
		/*确定unum的值*/
		if(radix==10&&num<0)/*十进制负数*/
		{
		unum=(unsigned)-num;
		str[i++]='-';
		}
		else unum=(unsigned)num;/*其他情况*/
		/*转换*/
		do{
		str[i++]=index[unum%(unsigned)radix];
		unum/=radix;
		}while(unum);
		str[i]='\0';
		/*逆序*/
		if(str[0]=='-')k=1;/*十进制负数*/
		else k=0;
		
		for(j=k;j<=(i-1)/2;j++)
		{
		temp=str[j];
		str[j]=str[i-1+k-j];
		str[i-1+k-j]=temp;
		}
		return str;
}
	
/**
  * @brief  Meteor Check
  * @param  None
  * @retval None
  */

bool CheckCommunication(void)
{
	uint8_t MeteorCheckData[5];
	uint16_t Len=5;
	uint8_t SendCmd[] = "Q\n";
	u8 i=0;
	char CheckReceiveFlag[]="OK Q\n";
	char ReceiveData[5];
	
	LED0 = 0;
	RS485_Send_Data(SendCmd,2);
	delay_ms(10);
	if(Rs485Meteor_Status)
	{
		Rs485Meteor_Status = 0;
		for(i=0;i<5;i++)
			MeteorCheckData[i] = RS485_RX_BUF[Rs485BufferFinishNumber][i];
		RS485_RX_CNT = 0;
	}
	if(strncmp(CheckReceiveFlag,MeteorCheckData,5))
		return false;
	return true;
}

/**
  * @brief  Meteor Start to measure
  * @param  2018-8-29  V2
  * @retval None
  */
bool SetVoltage(u16 VoltageValue)
{
		uint8_t Cmd[6];
		
		Cmd[0] = 'V';
		Cmd[1] = 0x30+(VoltageValue/100)%10;
		Cmd[2] = 0x30+(VoltageValue/10)%10;
		Cmd[3] = 0x30+VoltageValue%10;
		Cmd[4] = '\n';
	
		RS485_Send_Data(Cmd, 5);

}

/**
  * @brief  Meteor Start to measure
  * @param  None
  * @retval None
  */
bool StartMea(void)
{
		uint8_t Cmd[]="K\n";
	  
		RS485_Send_Data(Cmd, 2);

}

/**
  * @brief  Meteor Start to measure
  * @param  None
  * @retval None
  */
bool StopMea(void)
{
	uint8_t MeteorStopkData[5];
	uint16_t Len=0;
	uint8_t SendCmd[] = "S\n";
	
	char CheckReceiveFlag[]="OK S\n";
	char ReceiveData[5];
	
	RS485_Send_Data(SendCmd,2);
	delay_ms(10);
	if(RS485_RX_CNT>4)
	{
		fifo_out(&NuclearFIFOBuffer,MeteorStopkData,RS485_RX_CNT);
		RS485_RX_CNT = 0;
	}
	if(strncmp(CheckReceiveFlag,MeteorStopkData,5))
		return false;
	return true;
}

/**
  * @brief  读取气象测量值
  * @param  value测量数值
  * @retval 读取成功返回TRUE 失败返回 FALSE
  */
bool ReadMeteorVal (void)
{
	uint8_t Len=0;
	uint8_t SendCmd[] = "E\n";
		
	RS485_Send_Data(SendCmd,2);
	delay_ms(10);
	return true;
	
}

u8 NuclearData_Status=0;
u8 NuclearData_DataLength=0;
u8 tempstatus = 0;
void NuclearGetData(u8 *NuclearData,u16 *Length)
{
	u8 res=0;	 
	u8 getCounter=0;
	while(NuclearData_Status<=0x0A)
	{
		tempstatus = fifo_out(&NuclearFIFOBuffer,&res,1);
		getCounter++;
		if(NuclearData_Status==0x0A)
		{
			fifo_out(&NuclearFIFOBuffer,&NuclearData[4],NuclearData_DataLength);
			*Length =  NuclearData_DataLength + 4;
			NuclearData_Status = 0x00;
			break;
		}
		else if(NuclearData_Status == 0x02)  //第三步，获取数据长度
		{
			if(getCounter==3)
			{
				NuclearData_DataLength = NuclearData_DataLength+res;
				NuclearData[2] =res;
			}
			else if(getCounter==4)
			{
				NuclearData[3] =res;
				NuclearData_DataLength = NuclearData_DataLength+(res<<8);
				NuclearData_Status = 0x0A;   //成功获取到了数据长度，可以进行读取
			}
		}
		else if(NuclearData_Status == 0x01&&res==0xFE)  //第二个报头
			{
					NuclearData_Status = 0x02;
					NuclearData[1] =res;
			}
		else if(NuclearData_Status==0x00&&res==0xFE)			//第一个报头
		{
			NuclearData_Status = 0x01;
			NuclearData[0] =res;
		}
		else
		{
			NuclearData_Status = 0x00;
		}
	}

}

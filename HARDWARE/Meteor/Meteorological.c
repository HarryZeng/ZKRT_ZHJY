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

fifo 		MeteorFIFOBuffer;
fifo 		NuclearFIFOBuffer;
u8      ReceiveBuf[2000];
u16 		MeteorBufCounter;
u16 		NuclearBufCounter;

//uint8_t  NuclearBufCounter[NUCLEAR_Buffer_SIZE]; 


/**
  * @brief   int to char
  * @param  None
  * @retval None
  */
char* itoa(int num,char*str,int radix)
{		/*������*/
		char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		unsigned unum;/*�м����*/
		char temp;
		int i=0,j,k;
		/*ȷ��unum��ֵ*/
		if(radix==10&&num<0)/*ʮ���Ƹ���*/
		{
		unum=(unsigned)-num;
		str[i++]='-';
		}
		else unum=(unsigned)num;/*�������*/
		/*ת��*/
		do{
		str[i++]=index[unum%(unsigned)radix];
		unum/=radix;
		}while(unum);
		str[i]='\0';
		/*����*/
		if(str[0]=='-')k=1;/*ʮ���Ƹ���*/
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
	
	char CheckReceiveFlag[]="OK Q\n";
	char ReceiveData[5];
	
	
	LED0 = 0;
	RS485_Send_Data(SendCmd,2);
	delay_ms(10);
	if(RS485_RX_CNT>4)
	{
		fifo_out(&NuclearFIFOBuffer,MeteorCheckData,RS485_RX_CNT);
		RS485_RX_CNT = 0;
	}
	if(strncmp(CheckReceiveFlag,MeteorCheckData,5))
		return false;
	return true;
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

	RS485_Receive_Data(MeteorStopkData,&Len);

	if(strncmp(CheckReceiveFlag,MeteorStopkData,5))
		return false;
	return true;
}

/**
  * @brief  ��ȡ�������ֵ
  * @param  value������ֵ
  * @retval ��ȡ�ɹ�����TRUE ʧ�ܷ��� FALSE
  */
bool ReadMeteorVal (void)
{
	uint8_t Len=0;
	uint8_t SendCmd[] = "E\n";
		
	RS485_Send_Data(SendCmd,2);
	
	return true;
	
}

u8 NuclearData_Status=0;
u8 NuclearData_DataLength=0;
void NuclearGetData(u8 *NuclearData,u16 *Length)
{
	u8 res=0;	 
	u8 getCounter=0;
	while(NuclearData_Status<=0x0A)
	{
		fifo_out(&NuclearFIFOBuffer,&res,1);
		getCounter++;
		if(NuclearData_Status==0x0A)
		{
			fifo_out(&NuclearFIFOBuffer,&NuclearData[4],NuclearData_DataLength);
			*Length =  NuclearData_DataLength + 4;
			break;
		}
		else if(NuclearData_Status == 0x02)  //����������ȡ���ݳ���
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
				NuclearData_Status = 0x0A;   //�ɹ���ȡ�������ݳ��ȣ����Խ��ж�ȡ
			}
		}
		else if(NuclearData_Status == 0x01&&res==0xFE)  //�ڶ�����ͷ
			{
					NuclearData_Status = 0x02;
					NuclearData[1] =res;
			}
		else if(NuclearData_Status==0x00&&res==0xFE)			//��һ����ͷ
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

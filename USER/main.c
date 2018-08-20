#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"  
#include "sram.h"   
#include "malloc.h" 
#include "usmart.h"  
#include "sdio_sdcard.h"    
#include "malloc.h" 
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h"    
#include "rs485.h" 
#include <Meteorological.h>
#include "usart3.h"
#include "gps.h"
#include "fifo.h"
#include "rs232.h"

 

FIL   	fileTXT;
FIL* 		fp;
BYTE 			buffer[]="hello world!";//	写入数据
BYTE 			buffer2[]="";//	写入数据
BYTE			TXTdata2UartBuffer[2000];

void PutTXTdata2Uart(void);
void PutTXTdata2TFCardTest(void);

void PutData2TXT(u8 *databuffer,uint16_t length);



uint8_t ReadyFlag=0;
uint32_t TimeFlag=0;
uint32_t TimeCounter=1000;
uint16_t BatteryAllowance=0;
u8 ReturnFlag=0;

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//串口1,发送缓存区
nmea_msg gpsx; 											//GPS信息
__align(4) u8 dtbuf[50];   								//打印缓存器
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode字符串 

extern u16 USART6_RX_STA;

/*
1-风向
2-真实风向
3-风速
4-真实风速
5-温度
6-湿度
7-大气压
8-露点温度
*/
u8 mem_perused=0;
u8 Usart1rxlen;
u8 Usart6rxlen;
u8 ComaBackCommand[]={0x5A,0x04,0xFF,0xFF,0x0D,0x0A};
int main(void)
{
	u16 i=0;
	u16 lenx;
	u32 TIME_Check=0;
 	u32 total,free;
	u8 keypress=0;
	u8 res=0;	
	u32 sumCounter=0;
	u8 upload=0; 
	fp = &fileTXT;
	u8 GPS_Save_File=0;
	FRESULT res_sd;
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	API_Uart6_init(115200);
	uart_init(115200);		//初始化串口波特率为115200
	LED_Init();					//初始化LED 
	//usmart_dev.init(84);		//初始化USMART
 	//KEY_Init();					//按键初始化 
	RS232_Init(4800);				//初始化232
	RS485_Init(115200);				//初始化485
	TIM7_Int_Init(1000,84-1);
	my_mem_init(SRAMIN);		//初始化内部内存池 
	my_mem_init(SRAMCCM);		//初始化CCM内存池
	
 	while(SD_Init())//检测不到SD卡
	{
		delay_ms(50);					
		LED0=!LED0;//DS0闪烁
	}
 	exfuns_init();							//为fatfs相关变量申请内存				 
  f_mount(fs[0],"0:",1); 					//挂载SD卡 
 	res=f_mount(fs[1],"1:",1); 				//挂载FLASH.	
										    
	while(exf_getfree("0",&total,&free))	//得到SD卡的总容量和剩余容量
	{
		delay_ms(200);
		LED0=!LED0;//DS0闪烁
	}													  			    

	res_sd = f_open(fp, "0:/ReceiveData.txt", FA_READ|FA_WRITE|FA_OPEN_ALWAYS);
	if (res_sd==FR_OK) {
//		printf("Open file successfully\r\n");
//		printf("FATFS OK!\r\n");
//		printf("SD Total Size:%d 	MB\r\n",total>>10);
//		printf("SD  Free Size:%d	MB\r\n",free>>10);
	}
	else
	{
		printf("Open file failed\r\n");
	}

	res_sd = f_lseek(fp,fp->fsize);  
	
	LED0 = 0;
	//fifo_alloc(&MeteorFIFOBuffer,2*1024);
	fifo_alloc(&NuclearFIFOBuffer,2*1024);
	mem_perused = my_mem_perused(SRAMIN);
	
	delay_ms(1000);
	
	while(1)
	{
		//printf("USART1 is oK \r\n");
		/*****************核辐射设备通讯---测试*******************/
		if(ReadyFlag==0)   
		{
			//if(1)
			if(CheckCommunication()==true)
			{
				printf("Communication is OK\r\n");
				printf("Ready to read...");
				StartMea();
				delay_ms(10);
				ReadyFlag = 1;
				RS485_RX_CNT = 0;
			}
			delay_ms(500);
			LED0 = 1;
			delay_ms(500);
		}
		/*****************核辐射设备数据---读取*******************/
		else if(ReadyFlag==1)
		{
			/*************************上位机*************************/
			if(USART_RX_STA&0x8000)
			{
				Usart1rxlen=USART_RX_STA&0x3fff;	//得到此次接收到的数据长度
				if(Usart1rxlen==4)
				{
					if(USART_RX_BUF[1]==0x01)  //设置电压
					{

					}
					if(USART_RX_BUF[1]==0x02)  //设置时间
					{
						TimeCounter = (USART_RX_BUF[3]<<8) + USART_RX_BUF[2];
						if(TimeCounter>9999) TimeCounter = 9999;
						if(TimeCounter<=0) TimeCounter = 0;
					}
					if(USART_RX_BUF[1]==0x04)  //返航指令
					{
							APIUsart6_Send_Data(ComaBackCommand,6);
					}
				}
				USART_RX_STA=0;//状态寄存器清空
			}
			/**************API**************/
			if(USART6_RX_STA&0x8000)
			{
				Usart6rxlen=USART6_RX_STA&0x3fff;	//得到此次接收到的数据长度
				if(Usart6rxlen==6)
				{
					if(USART6_RX_BUF[1]==0x03)  //读取到电量
					{
							BatteryAllowance = (USART6_RX_BUF[5]<<24) + (USART6_RX_BUF[4]<<16)+(USART6_RX_BUF[3]<<8)+USART6_RX_BUF[2];
					}
				}
				if(Usart6rxlen==4)
				{
					if(USART6_RX_BUF[1]==0x14)  //回复返航成功
					{
							ReturnFlag = 1;
					}
				}
				USART6_RX_STA=0;//状态寄存器清空
			}
			/****************核辐射气象****************/
			if(TimeFlag)
			{
				TimeFlag =0;
				LED0=1;
				//printf("USART1 IS OK\r\n");
				/*等待气象模块读取到数据后，再一起转发出去*/
				PutData2TXT(RS232_RX_BUF[BufferFinishNumber],MeteorBufCounter);
				for(i=0;i<MeteorBufCounter;i++)
				{
						printf("%c",RS232_RX_BUF[BufferFinishNumber][i]);
				}
				printf("\r\n");
				/*读取核辐射模块数据*/
				ReadMeteorVal();
				/*转发核辐射数据*/
				delay_ms(20);
				NuclearGetData(ReceiveBuf,&NuclearBufCounter);
				PutData2TXT(ReceiveBuf,NuclearBufCounter);
				printf("$HFSSJ,");
				for(i=0;i<NuclearBufCounter;i++)
					printf("%c",ReceiveBuf[i]);
				printf("\r\n");
				/*发送飞机状态*/
				printf("$WRJSJ,Battery:%d,ReturnFlag:%d",BatteryAllowance,ReturnFlag);
				
			}
			else
			{
				LED0=0;
			}			
		}
	} 
}

/*******************************************
将读取到的数据保存到SD卡里
*******************************************/
uint32_t ByteCounter=0;
void PutData2TXT(u8 *databuffer,uint16_t length)
{
	FRESULT res_sd;

	res_sd = f_write(fp, databuffer, length, &bw);
	if (res_sd == FR_OK)
	{
		f_sync(fp);
		if (res_sd==FR_OK) 
		{
				ByteCounter = ByteCounter+bw;
				//printf("%8d\r\n",ByteCounter);
		}
	}	
}

/*******************************************
将SD里的TXT读取出来，通过串口打印出去，最大只能打印1000个字节
*******************************************/
void PutTXTdata2Uart(void)
{
	FRESULT res_sd;
	UINT fnum;
	f_close(fp);
	printf("\r\n******************Read TXT data*****************\r\n");
	res_sd = f_open(fp, "0:/ReceiveData.txt", FA_READ|FA_OPEN_EXISTING);
	if (res_sd == FR_OK) {
		printf("Open file successfully,ready to read...\r\n");
	}
	res_sd = f_read(fp, TXTdata2UartBuffer, sizeof(TXTdata2UartBuffer), &fnum);
	if (res_sd==FR_OK) {
		printf("Reveive Data Counter:%d\r\n",fnum);
		printf("Data:\r\n %s \r\n", TXTdata2UartBuffer);
	}
	else
	{
		printf("Read file failed\r\n");
	}
	f_close(fp);
	if (res_sd==FR_OK) {
		printf("close file successfully\r\n");
	}
	else
	{
		printf("close file failed\r\n");
	}
}

/*******************************************
测试SD卡，往SD卡里面写入helloworld 测试
*******************************************/
void PutTXTdata2TFCardTest(void)
{
	FRESULT res_sd;
	u32 total,free;
	int fputsCounter;
	
	exf_getfree("0",&total,&free);
	
	delay_ms(200);
	res_sd = f_open(fp, "0:/ReceiveData.txt", FA_READ|FA_WRITE|FA_OPEN_ALWAYS);
	if (res_sd==FR_OK) {
		printf("Open file successfully\r\n");
		printf("FATFS OK!\r\n");
		printf("SD Total Size:%d 	MB\r\n",total>>10);
		printf("SD  Free Size:%d	MB\r\n",free>>10);
	}
	else
	{
		printf("Open file failed\r\n");
	}
	delay_ms(200);
	fputsCounter = f_puts((char *)buffer2,fp); 
	
	printf("write data %d \r\n",fputsCounter);

	res_sd = f_close(fp);
	if (res_sd==FR_OK) {
		printf("close file successfully\r\n");
	}
	else
	{
		printf("close file failed\r\n");
	}
}




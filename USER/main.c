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

 
#define  WORK_MODE  1//0->�����豸��1->�˷����豸
 
FIL   	fileTXT;
FIL* 		fp;
BYTE 			buffer[]="hello world!";//	д������
BYTE 			buffer2[]="";//	д������
BYTE			TXTdata2UartBuffer[2000];

void PutTXTdata2Uart(void);
void PutTXTdata2TFCardTest(void);

void PutData2TXT(u8 *databuffer,uint16_t length);


uint32_t SyncTimeCounter=0;
#if (WORK_MODE==0)
uint8_t WorkInFlag=0;
uint8_t ReadyFlag=1;
u32 bound=4800;
#else
uint8_t WorkInFlag=1;
uint8_t ReadyFlag=0;
u32 bound=115200;
#endif

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
nmea_msg gpsx; 											//GPS��Ϣ
__align(4) u8 dtbuf[50];   								//��ӡ������
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode�ַ��� 

/*
1-����
2-��ʵ����
3-����
4-��ʵ����
5-�¶�
6-ʪ��
7-����ѹ
8-¶���¶�
*/
u8 mem_perused=0;
int main(void)
{
	u16 i=0,rxlen;
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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	usmart_dev.init(84);		//��ʼ��USMART
 	KEY_Init();					//������ʼ�� 
	RS232_Init(4800);				//��ʼ��232
	RS485_Init(115200);				//��ʼ��485

	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ�� 
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ��
	
 	while(SD_Init())//��ⲻ��SD��
	{
		delay_ms(50);					
		LED0=!LED0;//DS0��˸
	}
 	exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
  f_mount(fs[0],"0:",1); 					//����SD�� 
 	res=f_mount(fs[1],"1:",1); 				//����FLASH.	
										    
	while(exf_getfree("0",&total,&free))	//�õ�SD������������ʣ������
	{
		delay_ms(200);
		LED0=!LED0;//DS0��˸
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
	delay_ms(5);
	LED0 = 0;
	
	fifo_alloc(&MeteorFIFOBuffer,2*1024);
	fifo_alloc(&NuclearFIFOBuffer,2*1024);
	
	mem_perused = my_mem_perused(SRAMIN);
	
	while(1)
	{
		//printf("USART1 is oK \r\n");
		/*****************�˷����豸ͨѶ---����*******************/
		if(WorkInFlag==1&&ReadyFlag==0)   
		{
			//if(1)
			if(CheckCommunication()==true)
			{
				printf("Communication is OK\r\n");
				printf("Ready to read...");
				StartMea();
				delay_ms(10);
				ReadyFlag = 1;
			}
			delay_ms(500);
			LED0 = 1;
			delay_ms(500);
		}
		/*****************�˷����豸����---��ȡ*******************/
		else if(WorkInFlag==1&&ReadyFlag==1)
		{
			delay_ms(20);
			TIME_Check++;
			if(TIME_Check>=55)
			{
				TIME_Check =0;
				ReadMeteorVal();
				LED0=1;
				delay_ms(20);
				NuclearGetData(ReceiveBuf,&NuclearBufCounter);
				PutData2TXT(ReceiveBuf,NuclearBufCounter);
				for(i=0;i<NuclearBufCounter;i++)
					printf("%c",ReceiveBuf[i]);	
			}
			else
			{
				LED0=0;
			}			
		}
		/*****************�����豸����----��ȡ******************/
		else if(WorkInFlag==0)
		{
			RS232_Receive_BufferLen(&MeteorBufCounter);
			if(MeteorBufCounter)//���յ�������
			{
				fifo_out(&MeteorFIFOBuffer,ReceiveBuf,MeteorBufCounter);
				SyncTimeCounter = 0;
				LED0=1;
				PutData2TXT(ReceiveBuf,MeteorBufCounter);
				sumCounter = sumCounter + MeteorBufCounter;
				for(i=0;i<MeteorBufCounter;i++)
					printf("%c",ReceiveBuf[i]);
				MeteorBufCounter = 0;
			}
			else
			{
				LED0=0;
			}
		}
		/*******************GPS����**********************/
//		if(USART3_RX_STA&0X8000)		//���յ�һ��������
//		{
//			LED0=0;
//			rxlen=USART3_RX_STA&0X7FFF;	//�õ����ݳ���
//			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
// 			USART3_RX_STA=0;		   	//������һ�ν���
//			USART1_TX_BUF[i]=0;			//�Զ���ӽ�����
//			//GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//�����ַ���
//			//Gps_Msg_Show();				//��ʾ��Ϣ	
//			GPS_Save_File = 1;
//			PutData2TXT(USART3_RX_BUF,rxlen);
//			if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//���ͽ��յ������ݵ�����1
// 		}
//			else
//				LED0=1;
	} 
}

/*******************************************
����ȡ�������ݱ��浽SD����
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
��SD���TXT��ȡ������ͨ�����ڴ�ӡ��ȥ�����ֻ�ܴ�ӡ1000���ֽ�
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
����SD������SD������д��helloworld ����
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




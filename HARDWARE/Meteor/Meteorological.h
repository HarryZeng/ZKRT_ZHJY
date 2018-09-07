#ifndef __Meteorological_H
#define __Meteorological_H	

#include "sys.h"
#include <stdbool.h>
#include <string.h>
#include "fifo.h"

#define METEOR_Buffer_SIZE  10*1024

extern fifo 	MeteorFIFOBuffer; 
extern fifo 	NuclearFIFOBuffer;
extern u16 		MeteorBufCounter;
extern u16 		NuclearBufCounter;
extern u8     ReceiveBuf[2000];
bool CheckCommunication(void);
bool StartMea(void);
bool StopMea(void);
bool ReadMeteorVal (void);
void NuclearGetData(u8 *NuclearData,u16 *Length);
bool SetVoltage(u16 VoltageValue);
#endif	   

/*******************************************************************************
* @File 	  : fifo.h
* @Author   : cqx
* @Version  : V0.0.1
* @Date 	  : 29-november-2016
* @Brief	  : This file provides all the fifo functions.
********************************************************************************
* @Attention:
* Non
*
*******************************************************************************/
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FIFO_H
#define _FIFO_H
 
#include "sys.h"
#include "malloc.h"	   
 
#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif		
 
#define is_power_of_2(x)	((x) != 0 && (((x) & ((x) - 1)) == 0))
		
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	unsigned int	in;
	unsigned int	out;
	unsigned int	mask;
	unsigned char *data;
}fifo; 
 
/* Function prototypes -------------------------------------------------------*/
extern unsigned int fifo_used( fifo *fifo);
extern signed int fifo_alloc( fifo *fifo, unsigned int size);
extern void         fifo_free( fifo *fifo);
extern int          fifo_init( fifo *fifo, unsigned char *buffer,	unsigned int size);
extern unsigned int fifo_in( fifo *fifo, unsigned char *buf, unsigned int len);
extern unsigned int fifo_out( fifo *fifo,	unsigned char *buf, unsigned int len);
	 
#ifdef __cplusplus
}
#endif
 
#endif

#ifndef _HAPC_BSP_H_
#define _HAPC_BSP_H_

#ifndef TRUE
	#ifndef FALSE
		typedef enum
		{
			TRUE	=	0xFF,
			FALSE 	=	0x00
		}Boolean;
	#endif
#endif

#ifndef	NULL
	#define NULL ((void *) 0)
#endif

#define BUFF_SIZE 2304
typedef struct buffs
{
	short BufData[BUFF_SIZE];
	struct buffs * Next;
}BuffDef;

//==========================
#include "Sys_Delay.h"

#include "Driver_Usart.h"

//==========================






#endif

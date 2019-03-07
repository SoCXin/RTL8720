
#include "stm32f10x.h"
#include "Hapc_BSP.h"

void Sys_DelaynUs(volatile u32 nCount)
{
  for(; nCount != 0; nCount--);
}

void Sys_DelaynMs(volatile u32 nCount)
{
  unsigned int i,j;

  for(i = 0; i < nCount; i++){
    for(j = 0; j < 10000; j++);
  }
}


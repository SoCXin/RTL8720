
#include <stdlib.h>
#include <string.h> 
#include "stm32f10x.h"
#include "Hapc_BSP.h"
#include "Fatfs_Api.h"
#include "includes.h"
#include "Driver_Usart.h"
#include "cs4344.h"
#include "delay.h"
#include "config.h"

#define VERSION "V1.00.00"

#define LED1_ON() do{GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);}while(0)
#define LED2_ON() do{GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);}while(0)
#define LED3_ON() do{GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);}while(0)
#define LED4_ON() do{GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);}while(0)
#define PWR_ON() do{GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);}while(0)

#define LED1_OFF() do{GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);}while(0)
#define LED2_OFF() do{GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);}while(0)
#define LED3_OFF() do{GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);}while(0)
#define LED4_OFF() do{GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);}while(0)
#define PWR_OFF() do{GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);}while(0)

#define RUN_LED_BLINK() do{GPIOB->ODR ^= (1 << 7);}while(0)
#define PLAY_LED_BLINK() do{GPIOB->ODR ^= (1 << 6);}while(0)

#define BUTTON_PWR (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4))
#define BUTTON_INC (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5))
#define BUTTON_DEC (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))
#define BUTTON_ENTER (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))

#define CHECK_TIME 20
#define TASKSTART_STK_SIZE 100
__align(8) static OS_STK 	TASKSTART_STK[TASKSTART_STK_SIZE];


#if 0
#define DECODE_DEBUG printf	
#else
#define DECODE_DEBUG
#endif

uint8_t bINC_flag = 0;//button increase flag(next music)

//=================================
#include "mp3dec.h"
#include "mp3common.h"
extern void Convert_Mono(short *buffer);
extern void Convert_Stereo(short *buffer);

// MP3
#define READBUF_SIZE 4000       // Value must min be 2xMAINBUF_SIZE = 2x1940 = 3880bytes
MP3FrameInfo mp3FrameInfo;      // Content is the output from MP3GetLastFrameInfo, 
                                // we only read this once, and conclude it will be the same in all frames
                                // Maybe this needs to be changed, for different requirements.
HMP3Decoder hMP3Decoder; // Content is the pointers to all buffers and information for the MP3 Library
volatile int bytesLeft;         // Saves how many bytes left in readbuf
volatile u32 outOfData;         // Used to set when out of data to quit playback

//s16 outbuf[4608];          // Playback buffer - Value must be 4608 to hold 2xMp3decoded frames.
u8 readBuf[READBUF_SIZE];       // Read buffer where data from SD card is read to
u8 *readPtr;                    // Pointer to the next new data
s32 offset;                                  // Used to save the offset to the next frame    
int errs = 0;                                   // Return value from MP3decoder
//===========================
#define DECODETASK_STK_SIZE	(256)	
OS_STK	DECODETASK_STK[DECODETASK_STK_SIZE];
BuffDef Buff[2];//DAC output buffer
BuffDef* BuffP = &Buff[0];
INT8U err;
OS_EVENT* SemBufWr;
static FATFS fs;       // Work area (file system object) for logical drive
static FIL fsrc;      // file objects


void NVIC_Configuration(void);
void DMA_Configuration(u32 addr, u32 size);
void DMA_Transmit(u32 addr, u32 size);	
void decodeTaskInit(void);


void ledConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//disable jtag
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Configure for LED  ---------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

void pwrCtrlConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//disable jtag
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  //power control, hold the power
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void buttonConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
}

void system_check(void)
{
  LED1_OFF();LED2_OFF();LED3_OFF();LED4_OFF();
  _delay_ms(CHECK_TIME);
  LED1_ON();
  _delay_ms(CHECK_TIME);
  LED2_ON();LED1_OFF();
  _delay_ms(CHECK_TIME);
  LED3_ON();LED2_OFF();
  _delay_ms(CHECK_TIME);
  LED4_ON();LED3_OFF();
  _delay_ms(CHECK_TIME);
  LED4_ON();LED3_OFF();
  _delay_ms(CHECK_TIME);
  LED3_ON();LED4_OFF();
  _delay_ms(CHECK_TIME);
  LED2_ON();LED3_OFF();
  _delay_ms(CHECK_TIME);
  LED1_ON();LED2_OFF();
}

void IWDG_Configuration(void)
{
  /* write 0x5555, enable wdg write access */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  /* WDG prescaler 40K/256=156HZ(6.4ms) */
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  /* feed interval 4s/6.4ms = 525, do not exceed 0xfff */
  IWDG_SetReload(525);
  /* feed the dog */
  IWDG_ReloadCounter();
  /*enable the dog */
  IWDG_Enable();
}

void TaskStart(void *parg)
{
  uint8_t pwr_flag = 0;
  uint8_t bINC_buf1 = 0,bINC_buf2 = 0;
  RCC_ClocksTypeDef  rcc_clocks;
  
  (void)parg;
	
  SystemInit();
  OS_CPU_SysTickInit();
  NVIC_Configuration();
  pwrCtrlConfig();
  PWR_ON();//HOLD PWR
  ledConfig(); 
  LED1_ON();//RUN LED on
  buttonConfig();
 
  USART_Conf();
  printf("System is up and running!\r\n");
  printf("Chip ID:0x%X%X%X.Flash size:%dKB.\r\n",
    *((int*)(0x1FFFF7E8+8)), *((int*)(0x1FFFF7E8+4)),
    *((int*)(0x1FFFF7E8+0)), *((short*)(0x1FFFF7E0)));	
  RCC_GetClocksFreq(&rcc_clocks);	//get the main clock	
  printf("Clock:%dMHz\r\n", (INT32U)rcc_clocks.HCLK_Frequency/1000000);
  
  system_check();
  
  decodeTaskInit();
  IWDG_Configuration();  // all init, config the watchdog
  LED3_OFF();
  
  while(1){    
	PLAY_LED_BLINK();
	if(BUTTON_PWR == SET) pwr_flag = 1;
	else pwr_flag = 0;

	bINC_buf1 = bINC_buf2;
	
	IWDG_ReloadCounter();  //feed the dog
	OSTimeDlyHMSM(0, 0, 0, 200);
	if((BUTTON_PWR == SET)&&(pwr_flag == 1)) PWR_OFF();
	else pwr_flag = 0;

	bINC_buf2 = BUTTON_INC;
	if(!bINC_buf1&&bINC_buf2) bINC_flag = 1;
	else bINC_flag = 0;	
  }
}

int main()
{
  OSInit();
  OSTaskCreate(TaskStart, (void *)0, &TASKSTART_STK[TASKSTART_STK_SIZE - 1], 7);		
  OSStart();
}

void decodeTask(void *parg)
{
  static UINT br; 

  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  (void) *parg;

  /* Initilizes the MP3 Library */
  hMP3Decoder = MP3InitDecoder();

  //=============================
  disk_initialize(0);
  
  f_mount(0, &fs);	//register to file system partition 0

  while(1){		
    if (f_opendir(&dirs, "") == FR_OK){ 			  //success to open directory	
	  while (f_readdir(&dirs, &finfo) == FR_OK){  	  //if there is file in this directory
		if (finfo.fattrib & AM_ARC) {
          if(!finfo.fname[0]) break;				
          printf("\r\n Now Playing:[");
          printf(finfo.fname);
          printf("]\r\n");
          res = f_open(&fsrc, finfo.fname, FA_OPEN_EXISTING | FA_READ);

          /* Reset counters */
          bytesLeft = 0;
          outOfData = 0;
          readPtr = readBuf;					

          res = f_read(&fsrc,readBuf, READBUF_SIZE, &br);
		  
		  bytesLeft += br;
					
          if( (res) ||(br == 0)) break;

          while(1){
			if(bINC_flag == 1){bINC_flag = 0; break;}//change file	
			
            /* find start of next MP3 frame - assume EOF if no sync found */
            offset = MP3FindSyncWord(readPtr, bytesLeft);
            if (offset < 0) {
              outOfData = 1;
              break;
            }
            else{
              readPtr += offset;       //data start point
              bytesLeft -= offset;      //in buffer
              errs = MP3Decode(hMP3Decoder, &readPtr, (int *)&bytesLeft, (BuffP->BufData), 0);
              
              if (bytesLeft < READBUF_SIZE){
                memmove(readBuf,readPtr,bytesLeft);
                res = f_read(&fsrc, readBuf + bytesLeft, READBUF_SIZE - bytesLeft, &br);

				if( (res) ||(br == 0)) break;
								
                if (br < READBUF_SIZE - bytesLeft)
                  memset(readBuf + bytesLeft + br, 0, READBUF_SIZE - bytesLeft - br);
                bytesLeft = READBUF_SIZE;
                readPtr = readBuf;                
              }
              MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
							
              //sample_rate(mp3FrameInfo.samprate);
              if(1 == mp3FrameInfo.nChans)
                Convert_Mono((BuffP->BufData));
              else
                Convert_Stereo((BuffP->BufData));

              OSSemPend(SemBufWr, 0, &err);
			  
              DMA_Transmit((u32)(BuffP->BufData), 2304);

              BuffP = BuffP->Next;
            }
          }
          f_close(&fsrc);					
        }
      }
    }
  }
}

void decodeTaskInit(void)
{
  Buff[0].Next = &Buff[1];
  Buff[1].Next = &Buff[0];

  SDCard_Init();
  cs4344_init();
 	  
  OSTaskCreate((void (*)(void *))decodeTask,(void *)0,&DECODETASK_STK[DECODETASK_STK_SIZE - 1], 8);
  
  SemBufWr = OSSemCreate(0);
  DMA_Configuration((u32)(BuffP->BufData), 1);
}


void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure the NVIC Preemption Priority Bits */
  /*NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);*/
#define VECT_TAB_FLASH
#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x10000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*enable DMA1 channel 5 NVIC*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口1中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	     	//抢占优先级 0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);		  
}

DMA_InitTypeDef DMA_InitStructure;

void DMA_Configuration(u32 addr, u32 size)
{
  /*open DMA clock*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_Cmd(DMA1_Channel5, DISABLE);
  I2S_Cmd(SPI2, DISABLE);
  DMA_DeInit(DMA1_Channel5);
  /* DMA Channel configuration ----------------------------------------------*/
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(0x4000380c);
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32) addr;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = (size);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  /* Enable SPI DMA Tx request */

  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
  I2S_Cmd(SPI2, ENABLE);
  DMA_Cmd(DMA1_Channel5, ENABLE);
}

void DMA_Transmit(u32 addr, u32 size)
{
  DMA_Cmd(DMA1_Channel5, DISABLE);   
  DMA_InitStructure.DMA_MemoryBaseAddr = addr;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel5, ENABLE);
}



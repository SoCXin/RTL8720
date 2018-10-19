/*
 * CS4344 driver for STM32.
 * 2014-07-13 Created by WUZUGUI.
 */
#include "cs4344.h"

void I2S_Configuration(void)
{
  I2S_InitTypeDef I2S_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure SPI2 pins: CK, WS and SD ---------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Configure SPI2 pins: CK, WS and SD ---------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Configure SPI2 pins: MCK ---------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

   /*打开I2S时钟*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  SPI_I2S_DeInit(SPI2);
  /* I2S peripheral configuration */
  I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16bextended;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
  I2S_InitStructure.I2S_AudioFreq = I2S_AudioFreq_44k;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;//Rx 是在录音模式下使用，Tx 是在播放模式下使用
  I2S_Init(SPI2, &I2S_InitStructure);
  I2S_Cmd(SPI2, ENABLE);
}

void cs4344_init(void)
{
  I2S_Configuration();
}


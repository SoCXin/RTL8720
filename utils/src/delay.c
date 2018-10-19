//Generate some kinds of delays.
//Copyright (C) 2008-2010  JOGY.
//2008-08-22 First created by JOGY.
//2010-01-19 Modified by JOGY.
//Standardise the coding style.
//2011-05-16 Modified by WUZUGUI.
//Add the real delay time of the functions in this module.

#include "delay.h"

void _delay1us(void)
{
	uint16_t i = COUNT_OF_US;
	while(i--);
}

void _delay_us(uint16_t usec)
{
    while(--usec)
	{
	_delay1us();
	}
}

void _delay_ms(uint16_t msec)
{
	while(--msec)
	_delay_us(1000);
}



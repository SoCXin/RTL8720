
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



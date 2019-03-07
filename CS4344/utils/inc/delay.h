
//Generate some kinds of delays.
//Copyright (C) 2008-2010  JOGY.
//2008-08-22 First created by JOGY.
//2010-01-19 Modified by JOGY.
//Standardise the coding style.
//2011-05-16 Modified by WUZUGUI.
//Add the real delay time of the functions in this module.


#ifndef __DELAY_H__
#define __DELAY_H__

#include "stdint.h"

#define FOSC_M (72000000UL) 
#define COUNT_OF_US (FOSC_M/(1000000UL))

// delay usec us
void _delay_us(uint16_t usec);


// delay msec ms
void _delay_ms(uint16_t msec);	

#endif

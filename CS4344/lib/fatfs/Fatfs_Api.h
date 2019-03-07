#ifndef _FATFS_API_H_
#define _FATFS_API_H_

#include "sdcard.h"
#include "integer.h"
#include "ff.h"
#include "diskio.h"


//======================================
void SDCard_Init(void);
FRESULT Fatfs_SearchPath(char* path);

#endif


#include <string.h>
#include "stm32f10x.h"
#include "Fatfs_Api.h"


void SDCard_Init(void)
{
	SD_Error Status = SD_OK;
	SD_CardInfo SDCardInfo;
	/*-------------------------- SD Init ----------------------------- */
	Status = SD_Init();

	if (Status == SD_OK)
	{
		/*----------------- Read CSD/CID MSD registers ------------------*/
		Status = SD_GetCardInfo(&SDCardInfo);
	}

	if (Status == SD_OK)
	{
		/*----------------- Select Card --------------------------------*/
		Status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
	}

	if (Status == SD_OK)
	{
		//Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
	}

	/* Set Device Transfer Mode to DMA */
	if (Status == SD_OK)
	{  
		//    Status = SD_SetDeviceMode(SD_DMA_MODE);//oet
		//   Status = SD_SetDeviceMode(SD_POLLING_MODE);
		Status = SD_SetDeviceMode(SD_INTERRUPT_MODE);
	}

}

#include "IAP.h"

void IAP_WriteFlag(uint32_t flag)
{
    __disable_irq();
    FLASH_Unlock();

    FLASH_ErasePage(IAP_FLAG_PAGE_ADDR);

    FLASH_ProgramWord(IAP_FLAG_ADDR, flag);

    FLASH_Lock();
    __enable_irq();
}

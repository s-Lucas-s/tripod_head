#ifndef __IAP_H
#define __IAP_H

#include "stm32f10x.h" // Device header

// STM32F103C8T6 64KB Flash 最后一页地址
#define IAP_FLAG_PAGE_ADDR 0x0800FC00 // 最后1KB页起始

#define IAP_FLAG_ADDR      (IAP_FLAG_PAGE_ADDR + 0x00) // 页内偏移0，存标志

#define NVIC_VectTab_FLASH_OFFSET	0x2000

// 升级标志定义
#define IAP_UPGRADE_FLAG 0xABCD1234                // 代表“需要升级”

uint32_t IAP_ReadFlag(void);
void IAP_WriteFlag(uint32_t flag);

#endif

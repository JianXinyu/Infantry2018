/**
  ******************************************************************************
  * File Name          : drivers_flash.c
  * Description        : Flash驱动
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * FLASH用于在掉电后存储数据，如PID参数等
  ******************************************************************************
  */
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_flash.h>
#include "drivers_flash.h"


uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(uint32_t*)faddr;
}

uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;
	else return FLASH_SECTOR_11;
}

void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)
{
	FLASH_EraseInitTypeDef FlashEraseInit;
	HAL_StatusTypeDef FlashStatus=HAL_OK;
	uint32_t SectorError=0;
	uint32_t addrx=0,endaddr=0;
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4){
		return;
	}
	HAL_FLASH_Unlock(); 
	addrx=WriteAddr;
	endaddr=WriteAddr+NumToWrite*4; //写入的结束地址
	if(addrx<0X1FFF0000)
	{
		while(addrx<endaddr) //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF){//有非0XFFFFFFFF的地方,要擦除这个扇区
				FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;//类型扇区擦除
				FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx); //要擦除的扇区
				FlashEraseInit.NbSectors=1; //一次只擦除一个扇区
				FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;//VCC=2.7~3.6V
				if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK){
					break;//发生错误了
				}
			}else{
				addrx+=4;
			}
			FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
		}
	}
	FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
	if(FlashStatus==HAL_OK)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr,*pBuffer)!=HAL_OK)//写入数据
			{
				break; //写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		}
	}
	HAL_FLASH_Lock(); //上锁
}
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址pBuffer:数据指针 NumToRead:字(32位)数
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.
	}
}

//#include "utilities_debug.h"
//void testFlashTask(void const * argument){
//	uint32_t testValue = 233;
//	STMFLASH_Write(ADDR_FLASH_SECTOR_11, &testValue, 1);
//	uint8_t hasRead = 0;
//	while(1){
//		if(hasRead == 0){
//			uint32_t readValue = 0;
//			STMFLASH_Read(ADDR_FLASH_SECTOR_11, &readValue, 1);
//			fw_printfln("readValue = %d", readValue);
//			hasRead = 1;
//		}
//		
//	}
//}

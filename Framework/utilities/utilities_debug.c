/**
  ******************************************************************************
  * File Name          : utilities_debug.c
  * Description        : 调试相关
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 重定向C标准库函数printf到串口DEBUG_UART
  ******************************************************************************
  */
#include "utilities_debug.h"

//调试时开启
#ifdef _FW_DEBUG
	#include "usart.h"
	#include "peripheral_define.h"
	
	#ifdef __GNUC__
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif 
	PUTCHAR_PROTOTYPE
	{
		HAL_UART_Transmit(&DEBUG_UART , (uint8_t *)&ch, 1, 0xFFFF);
		return ch;
	}
#endif

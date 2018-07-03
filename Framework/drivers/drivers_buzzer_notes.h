/**
  ******************************************************************************
  * File Name          : drivers_buzzer_notes.h
  * Description        : 蜂鸣器驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 蜂鸣器音乐表格
  ******************************************************************************
  */
#ifndef DRIVERS_BUZZER_NOTES_H
#define DRIVERS_BUZZER_NOTES_H

#include <stdint.h>

#define L1	262
#define L1U	277
#define L2	294
#define L2U	311
#define L3	330
#define L4	349
#define L4U	370
#define L5	392
#define L5U	415
#define L6	440
#define L6U	466
#define L7	494

#define M1	523
#define M1U	554
#define M2	587
#define M2U	622
#define M3	659
#define M4	698
#define M4U	740
#define M5	784
#define M5U	831
#define M6	880
#define M6U	932
#define M7	988

#define H1	1046
#define H1U	1109
#define H2	1175
#define H2U	1245
#define H3	1318
#define H4	1397
#define H4U	1480
#define H5	1568
#define H5U	1661
#define H6	1760
#define H6U	1865
#define H7	1976

uint16_t notes_low[] = {
	L1, L1U, L2, L2U, L3, L4, L4U, L5, L5U, L6, L6U, L7
};
uint16_t notes_mid[] = {
	M1, M1U, M2, M2U, M3, M4, M4U, M5, M5U, M6, M6U, M7
};
uint16_t notes_high[] = {
	H1, H1U, H2, H2U, H3, H4, H4U, H5, H5U, H6, H6U, H7
};

#endif

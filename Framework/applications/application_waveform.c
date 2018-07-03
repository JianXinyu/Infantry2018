/**
  ******************************************************************************
  * File Name          : application_waveform.c
  * Description        : 波形观察
  **********************debug by ZY*********************************************
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 使用匿名四轴地面站做上位机观察电机波形
	* 按地面站的串口协议发送
	*
  ******************************************************************************
  */
#include <usart.h>
#include <stdint.h>
/*
typedef struct{
	uint16_t head;
	uint8_t id;
	uint8_t dlc;
	float dataPitch;
	float dataYaw;
	uint8_t checkSum;
}data_to_PC;
*/


uint8_t data_send_to_PC[17];//要发送给上位机的报文
//data_to_PC my_data_to_PC;

/*
发送给上位机的数据帧定义
@前2个字节为帧头0xAAAA
@第3个字节为帧ID，应设置为0xF1~0xFA中的一个
@第4个字节为报文数据长度(dlc)
@第5个字节开始到第5+dlc-1个字节为要传输的数据内容段，每个数据场为高字节在前，地字节在后
@第5+dlc个字节为CheckSum,为第1个字节到第5+dlc-1个字节所有字节的值相加后，保留结果的低八位作为CheckSum
@我们需要传输3个float数据，pitchRealAngle(Pitch),ZGyroModuleAngle(Yaw), gYroZs(Speed),dlc应该为3*4 bytes=12 bytes
@所以整个报文长度为5+12=17个字节
*/
void send_data_to_PC(UART_HandleTypeDef *huart,float zyPitch,float zyYaw,float zySpd)
{
//	my_data_to_PC.head=0xAAAA;
//	my_data_to_PC.id=0xF1;
//	my_data_to_PC.dlc=8;
//	my_data_to_PC.dataPitch=zyPitch;
//	my_data_to_PC.dataYaw=zyYaw;
//	
//	uint8_t * pTemp;
//	//uint8_t temp;
//	int i;
//	my_data_to_PC.checkSum=0;
//	pTemp = (uint8_t *)&(my_data_to_PC);  
//	for(i=0;i<12;i++)
//	{
//		 my_data_to_PC.checkSum+=pTemp[i];
//	}
	
	
	uint8_t * pTemp;
	int i;
	data_send_to_PC[0]=0xAA;
	data_send_to_PC[1]=0xAA;//前两个字节为帧头0xAA
	data_send_to_PC[2]=0xF1;//第3个字节为帧ID，设为0xF1
	data_send_to_PC[3]=12;//第4个字节为数据长度dlc，需要传输3个float数据(3*4 bytes=12 bytes)
	pTemp=(uint8_t *)&zyPitch;//将数据场首指针转换为uint8_t型指针，方便后续操作
	
	/*第一个float数据Pitch轴角度，传输时要将高位放在前面，低位放在后面
	STM32是小端存储(Little Endian),高位数据储存在高地址中。
	所以需要如下的转换，将高位数据移到数据帧的前面来。其他两个数据Yaw和速度Spd用同样方法处理
	*/
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[4+i]=pTemp[3-i];
	}
	
	pTemp=(uint8_t *)&zyYaw;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[8+i]=pTemp[3-i];
	}
	
	pTemp=(uint8_t *)&zySpd;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[12+i]=pTemp[3-i];
	}
	
	data_send_to_PC[16]=0;//第17个字节为本帧报文的CheckSum，按照前述CheckSum的要求求和计算即可
	for(i=0;i<16;i++)
	{
		 data_send_to_PC[16]+=data_send_to_PC[i];
	}
	
	HAL_UART_Transmit(huart,data_send_to_PC,17,1000);//向上位机发送本帧报文，共17个字节
}

/**
  ******************************************************************************
  * File Name          : drivers_imu.c
  * Description        : IMU驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * MPU6050 IST8310
  ******************************************************************************
  */
#include <cmsis_os.h>
#include <spi.h>
#include <tim.h>
#include "drivers_imu_low.h"
#include "utilities_debug.h"
#include "utilities_tim.h"
#include "drivers_imu_mpu6500_reg.h"
#include "drivers_imu_IST8310_reg.h"
#include "rtos_semaphore.h"
#include "iwdg.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)


uint8_t MPU_id = 0;
uint8_t datasc = 1;

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};

IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
	datasc = data;
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}

//Initialize the IST8310
uint8_t InitIST8310(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
	HAL_IWDG_Refresh(&hiwdg);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
	HAL_IWDG_Refresh(&hiwdg);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  HAL_IWDG_Refresh(&hiwdg);
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

//Initialize the MPU6500
uint8_t InitMPU6500(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    
		//{MPU6500_INT_PIN_CFG,   0x06},//Enable INT ==
		//{MPU6500_INT_ENABLE,    0x00},//Enable INT ==
		
		{MPU6500_USER_CTRL,     0x20},      // Enable AUX
		
		{MPU6500_INT_ENABLE,    0x01}//Enable INT ==
  };
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  //fw_printfln("MPU_id %x", MPU_id);
	
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }

  return 0;
}

//Get 10 axis data from MPU6500
void IMU_Get_Data()
{
  uint8_t mpu_buff[22];
//	uint8_t ist_buff[6];
//	uint8_t a,b;
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 22);
  
  imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
  imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
  imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
  
  imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
  imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
  imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
  imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;
  
	imu_data.mx=mpu_buff[15]<<8 |mpu_buff[14];
	imu_data.my=mpu_buff[17]<<8 |mpu_buff[16];
  imu_data.mz=mpu_buff[19]<<8 |mpu_buff[18];
	
}

float gYroXs, gYroYs, gYroZs;
void IMUTask(void const * argument){
//	int16_t tmaxx, tmaxy, tmaxz;
//	int16_t tminx, tminy, tminz;
//	IMU_Get_Data();
//	tmaxx = tminx = imu_data.mx;
//	tmaxy = tminy = imu_data.my;
//	tmaxz = tminz = imu_data.mz;
	
//			__HAL_SPI_DISABLE(&hspi5); //关闭SPI
//	hspi5.Instance->CR1 &= 0XFFC7; //位3-5清零，用来设置波特率
//	hspi5.Instance->CR1 |= SPI_BAUDRATEPRESCALER_4;//设置SPI速度
//	__HAL_SPI_ENABLE(&hspi5); //使能SPI

	
	while(1){
		//IMU_Get_Data();
		//updateQuaternion();
		
		osSemaphoreWait(refreshMPU6500SemaphoreHandle, osWaitForever);
		
		MPU6500_NSS_Low();
		uint8_t MPU_Tx = MPU6500_ACCEL_XOUT_H | 0x80;
		HAL_SPI_Transmit(&hspi5, &MPU_Tx, 1, 55);
//		HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
//		osSemaphoreWait(imuSpiTxRxCpltSemaphoreHandle, osWaitForever);
//		if(HAL_SPI_Transmit_DMA(&hspi5, &MPU_Tx, 1) != HAL_OK){
//			fw_Warning();
//		}
		uint8_t mpu_buff[22];
		HAL_SPI_Receive(&hspi5, mpu_buff, 22, 55);
//		HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, mpu_buff, 22, 55);
//		osSemaphoreWait(imuSpiTxRxCpltSemaphoreHandle, osWaitForever);
//		if(HAL_SPI_Receive_IT(&hspi5, mpu_buff, 22) != HAL_OK){
//			fw_Warning();
//		}
		MPU6500_NSS_High();

		imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
		imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
		imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];

		imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];

		imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
		imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
		imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;

		imu_data.mx=mpu_buff[15]<<8 |mpu_buff[14];
		imu_data.my=mpu_buff[17]<<8 |mpu_buff[16];
		imu_data.mz=mpu_buff[19]<<8 |mpu_buff[18];
		
		gYroXs = imu_data.gx / 32.8f;
		gYroYs = imu_data.gy / 32.8f;
		gYroZs = imu_data.gz / 32.8f;
	
		
		//osSemaphoreRelease(refreshIMUSemaphoreHandle);
		
//		fw_printfln("axyz %d %d %d", imu_data.ax, imu_data.ay, imu_data.az);
//		fw_printfln("gxyz %d %d %d", imu_data.gx, imu_data.gy, imu_data.gz);
//		fw_printfln("mxyz %d %d %d", imu_data.mx, imu_data.my, imu_data.mz);
		
//		fw_printfln("xyz %f %f %f", angles[0], angles[1], angles[2]);
//		fw_printfln("mxyz %f %f %f", 
//			(imu_data.mx - (maxx + minx) / 2.0) / (maxx - minx) * 2,
//			(imu_data.my - (maxy + miny) / 2.0) / (maxy - miny) * 2,
//			(imu_data.mz - (maxz + minz) / 2.0) / (maxz - minz) * 2
//		);
//		if(imu_data.mx > tmaxx)tmaxx = imu_data.mx;
//		if(imu_data.mx < tminx)tminx = imu_data.mx;
//		if(imu_data.my > tmaxy)tmaxy = imu_data.my;
//		if(imu_data.my < tminy)tminy = imu_data.my;
//		if(imu_data.mz > tmaxz)tmaxz = imu_data.mz;
//		if(imu_data.mz < tminz)tminz = imu_data.mz;
//		fw_printfln("max %d %d %d", tmaxx, tmaxy, tmaxz);
//		fw_printfln("min %d %d %d", tminx, tminy, tminz);
		
		//fw_printfln("time %f", fw_getTimeMicros() / 1000000.0);
//		fw_printfln("=============");
		//osDelay(500);
	}
}

//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
//	osSemaphoreRelease(imuSpiTxRxCpltSemaphoreHandle);
//}

//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
//	osSemaphoreRelease(imuSpiTxRxCpltSemaphoreHandle);
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
//	osSemaphoreRelease(imuSpiTxRxCpltSemaphoreHandle);
//}

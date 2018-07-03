
#include "drivers_servo.h"


int id = 0, pwm = 500, time = 0;//默认舵机ID是0 
char InitServoMes[]="#000P0500T0000!";//#000P0500T1000!:0号舵机转到0500位置, 位置范围500-2500. 
																								//#000PMOD3!:舵机模式, 最大角度范围180, 顺时针旋转
char ServoMes[15];
uint8_t servoBuffer[20]={0};
uint8_t p;
void InitServoUart()
{
	HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&InitServoMes, sizeof(InitServoMes), 0xFFFF);
	if(HAL_UART_Receive_DMA(&SERVO_UART, &p, 1) != HAL_OK){
			Error_Handler();
	} 
}

void servoUartRxCpltCallback()
{

}

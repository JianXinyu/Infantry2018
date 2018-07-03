/**
  ******************************************************************************
  * File Name          : application_quaternion.h
  * Description        : 四元数解算
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * IMU四元数解算到角度
  ******************************************************************************
  */
#include "stdint.h"

#include "utilities_tim.h"
#include "drivers_imu_low.h"
extern IMUDataTypedef imu_data ;

//初始化IMU数据
#define BOARD_DOWN 1   //板子正面朝下摆放

#include "math.h"
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float gx, gy, gz, ax, ay, az, mx, my, mz;
float gYroX, gYroY, gYroZ;
float angles[3];
int16_t maxx = 203, maxy = -45, maxz = 421;
int16_t minx = -90, miny = -321, minz = 134;
void updateQuaternion()
{
			float mygetqval[9];
			mygetqval[0] = imu_data.ax;
			mygetqval[1] = imu_data.ay;
			mygetqval[2] = imu_data.az;
			
			mygetqval[3] = imu_data.gx / 32.8f;
			mygetqval[4] = imu_data.gy / 32.8f;
			mygetqval[5] = imu_data.gz / 32.8f;
			
			mygetqval[6] = (imu_data.mx - (maxx + minx) / 2.0) / (maxx - minx) * 2;
			mygetqval[7] = (imu_data.my - (maxy + miny) / 2.0) / (maxy - miny) * 2;
			mygetqval[8] = (imu_data.mz - (maxz + minz) / 2.0) / (maxz - minz) * 2;
			
			gYroX = mygetqval[3];
			gYroY = mygetqval[4];
			gYroZ = mygetqval[5];
			
#define Kp 2.0f
#define Ki 0.01f 
#define M_PI  (float)3.1415926535
			static uint64_t lastUpdate, now;
			static float exInt, eyInt, ezInt;

			float norm;
			float hx, hy, hz, bx, bz;
			float vx, vy, vz, wx, wy, wz;
			float ex, ey, ez, halfT;
			float tempq0,tempq1,tempq2,tempq3;

			float q0q0 = q0*q0;
			float q0q1 = q0*q1;
			float q0q2 = q0*q2;
			float q0q3 = q0*q3;
			float q1q1 = q1*q1;
			float q1q2 = q1*q2;
			float q1q3 = q1*q3;
			float q2q2 = q2*q2;   
			float q2q3 = q2*q3;
			float q3q3 = q3*q3;   
			
			//halfT=2.5/1000;
			now = fw_getTimeMicros();  //读取时间 单位是us   
			halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//			if((now-lastUpdate)<100)
//			{
//				//halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
//				return;
//			}
//			else	
//			{
//					halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//			}
			lastUpdate = now;	//更新时间
			
			gx = mygetqval[3] * M_PI/180;
			gy = mygetqval[4] * M_PI/180;
			gz = mygetqval[5] * M_PI/180;
			ax = mygetqval[0];
			ay = mygetqval[1];
			az = mygetqval[2];
			mx = mygetqval[6];
			my = mygetqval[7];
			mz = mygetqval[8];

			//快速求平方根算法
			norm = invSqrt(ax*ax + ay*ay + az*az);       
			ax = ax * norm;
			ay = ay * norm;
			az = az * norm;
			//把加计的三维向量转成单位向量。
			norm = invSqrt(mx*mx + my*my + mz*mz);          
			mx = mx * norm;
			my = my * norm;
			mz = mz * norm; 
			// compute reference direction of flux
			hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
			hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
			hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
			bx = sqrt((hx*hx) + (hy*hy));
			bz = hz; 
			// estimated direction of gravity and flux (v and w)
			vx = 2.0f*(q1q3 - q0q2);
			vy = 2.0f*(q0q1 + q2q3);
			vz = q0q0 - q1q1 - q2q2 + q3q3;
			wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
			wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
			wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = (ay*vz - az*vy) + (my*wz - mz*wy);
			ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
			ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

			if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
			{
					exInt = exInt + ex * Ki * halfT;
					eyInt = eyInt + ey * Ki * halfT;	
					ezInt = ezInt + ez * Ki * halfT;
					// 用叉积误差来做PI修正陀螺零偏
					gx = gx + Kp*ex + exInt;
					gy = gy + Kp*ey + eyInt;
					gz = gz + Kp*ez + ezInt;
			}
			// 四元数微分方程
			tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
			tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
			tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
			tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

			// 四元数规范化
			norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
			q0 = tempq0 * norm;
			q1 = tempq1 * norm;
			q2 = tempq2 * norm;
			q3 = tempq3 * norm;
			
			float q[4];
			q[0] = q0; //返回当前值
			q[1] = q1;
			q[2] = q2;
			q[3] = q3;
			//float angles[3];
			angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
			angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
			angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  

			static int countPrint = 0;
			if(countPrint > 50)
			{
				countPrint = 0;
				
//				fw_printf("mx max = %d | min = %d\r\n", mymaxmx, myminmx);
//				fw_printf("my max = %d | min = %d\r\n", mymaxmy, myminmy);
//				fw_printf("mz max = %d | min = %d\r\n", mymaxmz, myminmz);
//				fw_printf("========================\r\n");
				
//				fw_printf("now = %d \r\n", now);
//				fw_printf("xxx = %d \r\n", 2147483647);
//				fw_printf("halfT = %f \r\n", halfT);

//				fw_printf("angles0 = %f | ", angles[0]);
//				fw_printf("angles1 = %f | ", angles[1]);
//				fw_printf("angles2 = %f\r\n", angles[2]);
//				fw_printf("========================\r\n");
//				
//				fw_printf("mx = %d | ",imu_data.mx);
//				fw_printf("my = %d | ",imu_data.my);
//				fw_printf("mz = %d\r\n",imu_data.mz);
//				fw_printf("========================\r\n");
				
//				fw_printf("mx = %f | ",mx);
//				fw_printf("my = %f | ",my);
//				fw_printf("mz = %f\r\n",mz);
//				fw_printf("========================\r\n");
			}
			else
			{
				countPrint++;
			}
}

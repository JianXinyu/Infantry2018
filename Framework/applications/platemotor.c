#include "pid_regulator.h"
#include "platemotor.h"

int plate_stuck = 0;
 
PID_Regulator_t PlatePositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t PlateSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;
#include "application_pidfunc.h"

#include "utilities_minmax.h"

void PID_Reset(PID_Regulator_t *pid){
	
}

void PID_Calc(PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	pid->errorSum += pid->target - pid->feedback;
	
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}

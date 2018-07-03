#ifndef APPLICATION_PIDFUNC_H
#define APPLICATION_PIDFUNC_H

#define PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax) { \
	0.0, 0.0, 0.0, 0.0, 0.0, \
	Kp, Ki, Kd, 0.0, 0.0, 0.0, \
	KpMax, KiMax, KdMax, 0.0, \
	OutputMax, \
	&PID_Calc, &PID_Reset \
};

typedef struct PID_Regulator_t
{
	float target;
	float feedback;
	float errorCurr;
	float errorSum;
	float errorLast;
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	
	void (*Calc)(struct PID_Regulator_t *pid);
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;

void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);

#endif

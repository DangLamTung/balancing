#include "PID.h"

int PID(float setpoint, float input){
	Kp = 6;
	Kd = 0.5;
	Ki = 1;
    dt = 0.01;
	err = input - setpoint;

	if(err > 0)
		dir = -1;
	else
		dir = 1;
	P = Kp*(err - err_1);
	I = 0.5*Ki*dt*(err + err_1);
	D = Kd/dt*(err - 2*err_1 + err_2);
	PID_value = PID_value_pre + ( (int) P +(int)D);
    err_1 = err;
    err_2 = err_1;
    PID_value_pre = PID_value;

    return PID_value + 500;
}

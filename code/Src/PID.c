#include "PID.h"

int PID(float setpoint, float input){
	Kp = 10;
	Kd = 1;
	Ki = 0.01;
    dt = 0.0116;
	err = input - setpoint;

	if(err > 0)
		dir = -1;
	else
		dir = 1;
	P = Kp*(err );
	I = 0.5*Ki*dt*(err + err_1);
	D = Kd/dt*(err - 2*err_1 + err_2);
	PID_value = + ( (int) P +(int) I+(int)D);
    err_1 = err;
    err_2 = err_1;
    PID_value_pre = PID_value;
    if(PID_value>0)
    return PID_value + 570;
    else
    return PID_value - 570;
}

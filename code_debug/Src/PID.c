#include "PID.h"

int PID(float setpoint, float input){
	Kp = 10;
	Kd = 0.35;
	Ki = 0;
	dt = 0.0116;
    int out;

	err = input - setpoint;

	P = Kp*(err );
	I = 0.5*Ki*dt*(err + err_1);
	D = Kd/dt*(err - 2*err_1 + err_2);
	PID_value =  ( (int) P +(int)D +(int)I);
    err_1 = err;
    err_2 = err_1;
    PID_value_pre = PID_value;


    if(PID_value > 0){
       out = PID_value + 570;
    }
    if(PID_value < 0){
       out = PID_value - 570;
    }
    return out;
}

int PID_1(float setpoint, float input){
	Kp1 = 40;
	Kd1 = 0.45;
	Ki1 = 0;
    dt = 0.0116;
    int out;

	err1 = input - setpoint;

	P = Kp1*(err1);
	I = 0.5*Ki1*dt*(err1 + err_1_1);
	D = Kd1/dt*(err1 - 2*err_1_1 + err_2_1);
	PID_value1 =  ( (int) P +(int)D +(int)I);
    err_1_1 = err1;
    err_2_1 = err_1_1;
    PID_value_pre1 = PID_value1;


    if(PID_value1 > 0){
       out = PID_value1 + 575;
    }
    if(PID_value1 < 0){
       out = PID_value1 - 575;
    }
    return out;
}

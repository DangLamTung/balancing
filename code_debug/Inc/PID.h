#ifndef PID_H_
#define PID_H_

float Kp, Ki, Kd;
float Kp1, Ki1, Kd1;
float dt;

float P,I,D;
float P1,I1,D1;

volatile int PID_value;
volatile int PID_value1;

volatile int PID_value_pre;
volatile int PID_value_pre1;

volatile int dir;
volatile float err;
volatile float err_1;
volatile float err_2;

volatile float err1;
volatile float err_1_1;
volatile float err_2_1;
int PID(float setpoint, float input);

#endif

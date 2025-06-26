#ifndef INC_OS_PID_H_
#define INC_OS_PID_H_

#ifdef QASSERT_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float input;
    float integral_sum;
    float error_prev;

    float max;
    float min;
} PIDController;

extern uint32_t SENSOR_TICKS;

void PID_setup(PIDController* controller, float kp, float ki, float kd, float setpoint, float max, float min);
float PID_action(PIDController* controller, float error);

#endif

#endif /* INC_OS_PID_H_ */


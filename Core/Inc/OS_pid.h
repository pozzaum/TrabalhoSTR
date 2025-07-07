#ifndef INC_OS_PID_H_
#define INC_OS_PID_H_

#define PID_SCALE 1000000

#include <stdint.h>

typedef struct {
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;

    int32_t setpoint;
    int32_t input;
    int32_t integral_sum;
    int32_t error_prev;

    int32_t max;
    int32_t min;
} PIDController;

extern uint32_t SENSOR_TICKS;

void PID_setup(PIDController* controller, int32_t kp, int32_t ki, int32_t kd, int32_t setpoint, int32_t max, int32_t min);
uint32_t PID_action(PIDController* controller, int32_t error);


#endif /* INC_OS_PID_H_ */

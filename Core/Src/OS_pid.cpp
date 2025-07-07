#include "OS_pid.h"


void PID_setup(PIDController* controller, int32_t kp, int32_t ki, int32_t kd, int32_t setpoint, int32_t max, int32_t min) {

    controller->Kp = kp;
    controller->Ki = ki;
    controller->Kd = kd;
    controller->setpoint = setpoint;
    controller->input = 0;
    controller->integral_sum = 0;
    controller->error_prev = 0;
    controller->max = max;
    controller->min = min;
}

uint32_t PID_action(PIDController* controller, int32_t error) {

    controller->integral_sum += error * SENSOR_TICKS;
    int32_t derivative_term = (error - controller->error_prev) / SENSOR_TICKS;

    controller->error_prev = error;

    int32_t comp_p = controller->Kp * error;
    int32_t comp_i = controller->Ki * controller->integral_sum;
    int32_t comp_d = controller->Kd * derivative_term;

    int32_t output = (comp_p + comp_i + comp_d) / PID_SCALE;

    if (output > (int32_t)controller->max) {
      output = controller->max;
    }
    if (output < (int32_t)controller->min) {
      output = controller->min;
    }

    return (int32_t)output;
}

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "rover_types.h"

typedef enum {
    EULER_FORWARD,
    EULER_BACKWARD,
    MIDPOINT_RK2,
    RK4,
} IntegrationMethod;


void integrator_set_method(IntegrationMethod new_method);

void integrator_step(
    RoverState *state,
    double v_C,
    double omega,
    double dt
);

#endif
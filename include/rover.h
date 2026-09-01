#ifndef ROVER_H
#define ROVER_H

#ifdef _WIN32
    #define ROVER_API __declspec(dllexport)
#else
    #define ROVER_API
#endif

#include "rover_types.h"
#include "integrator.h"


ROVER_API void rover_init(double new_L);

ROVER_API void rover_step(
    double v_R,
    double v_L,
    double dt
);

ROVER_API RoverState rover_get_state(void);

ROVER_API void rover_set_integration_method(IntegrationMethod method);

#endif
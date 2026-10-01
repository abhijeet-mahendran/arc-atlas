// Ctrl + Shift + B to compile and run
#include "rover.h"
#include <math.h>

static RoverState rover;
static double L; // track width

static double calc_v_C(double v_R, double v_L) // rover center linear velocity
{
    return (v_R + v_L)/2;
}
    
static double calc_omega_C(double v_R, double v_L, double L) // rover center angular velocity
{
    return (v_R - v_L)/L;
}

void rover_init(double new_L)
{
    rover = (RoverState){
        .x = 0,
        .y = 0,
        .theta = 0
    };

    L = new_L;
}

void rover_step(double v_R, double v_L, double dt)
{
    double v_C = calc_v_C(v_R, v_L);
    double omega = calc_omega_C(v_R, v_L, L);
    integrator_step(&rover, v_C, omega, dt);
}

RoverState rover_get_state(void)
{
    return rover;
}

void rover_set_integration_method(IntegrationMethod new_method)
{
    integrator_set_method(new_method);
}
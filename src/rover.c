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

static void update_state(RoverState *rover, double v_C, double omega, double dt)
{
    rover->x += v_C * cos(rover->theta) * dt;
    rover->y += v_C * sin(rover->theta) * dt;
    rover->theta += omega * dt;
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
    update_state(&rover, v_C, omega, dt);
}

RoverState rover_get_state()
{
    return rover;
}

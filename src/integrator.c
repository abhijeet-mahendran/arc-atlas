#include "rover_types.h"
#include "integrator.h"
#include <math.h>

static IntegrationMethod method = RK4;

void integrator_set_method(IntegrationMethod new_method)
{
    method = new_method;
}

// ------------------------------------------------------------ EULER
static void integrate_euler_forward(
    RoverState *state,
    double v_C,
    double omega,
    double dt
)
{
    state->x += v_C * cos(state->theta) * dt;
    state->y += v_C * sin(state->theta) * dt;
    state->theta += omega*dt;
}

static void integrate_euler_backward(
    RoverState *state,
    double v_C,
    double omega,
    double dt
)
{
    state->theta += omega*dt;
    state->x += v_C * cos(state->theta) * dt;
    state->y += v_C * sin(state->theta) * dt;
}

// ------------------------------------------------------------ RK2
static void integrate_midpoint_rk2(
    RoverState *state,
    double v_C,
    double omega,
    double dt
)
{
    double theta_mid = state->theta + 0.5*omega*dt;
    state->x += v_C * cos(theta_mid) * dt;
    state->y += v_C * sin(theta_mid) * dt;
    state->theta += omega*dt;
}

// ------------------------------------------------------------ RK4
static void integrate_rk4(
    RoverState *state,
    double v_C,
    double omega,
    double dt
)
{
    double k_avg_x = (v_C*cos(state->theta) + 4*v_C*cos(state->theta + 0.5*omega*dt) + v_C*cos(state->theta + omega*dt))/6;
    double k_avg_y = (v_C*sin(state->theta) + 4*v_C*sin(state->theta + 0.5*omega*dt) + v_C*sin(state->theta + omega*dt))/6;
    double k_avg_theta = omega; // bro
    state->x += k_avg_x * dt;
    state->y += k_avg_y * dt;
    state->theta += k_avg_theta * dt;
}

// ------------------------------------------------------------ INTEGRATION SWITCH
void integrator_step(RoverState *state, double v_C, double omega, double dt)
{
    switch(method)
    {
        case EULER_FORWARD:
            integrate_euler_forward(state, v_C, omega, dt);
            break;
        
        case EULER_BACKWARD:
            integrate_euler_backward(state, v_C, omega, dt);
            break;
        
        case MIDPOINT_RK2:
            integrate_midpoint_rk2(state, v_C, omega, dt);
            break;
        
        case RK4:
            integrate_rk4(state, v_C, omega, dt);
            break;
        
        default:
            integrate_euler_forward(state, v_C, omega, dt);
            break;
    }
}


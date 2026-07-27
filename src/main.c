// Ctrl + Shift + B to compile and run
#include <stdio.h>
#include <math.h>

typedef struct {
    double x; // x-coordinate of rover m
    double y; // y-coordinate of rover in m
    double theta; // orientation of rover in rad
} RoverState;

double calc_v_C(double v_R, double v_L) // rover center linear velocity
{
    return (v_R + v_L)/2;
}
    
double calc_omega_C(double v_R, double v_L, double L) // rover center angular velocity
{
    return (v_R - v_L)/L;
}

void print_state(const RoverState *rover)
{
    // printf("Rover state:\n");
    printf("x = %.2f m\n", rover->x);
    printf("y = %.2f m\n", rover->y);
    printf("theta = %.2f rad\n", rover->theta);
}

void update_state(RoverState *rover, double v_C, double omega, double dt)
{
    rover->x += v_C * cos(rover->theta) * dt;
    rover->y += v_C * sin(rover->theta) * dt;
    rover->theta += omega * dt;
}

int main() {

    double dt = 0.01; // simulation timestep in s
    double L = 2; // rover track width in m
    double v_R = 8; // right wheel speed in m/s
    double v_L = 4; // left wheel speed in m/s
    double T = 1; // simulation time in s
    RoverState rover = {
        .x = 0,
        .y = 0,
        .theta = 0
    };

    double v_C = calc_v_C(v_R,v_L);
    double omega_C = calc_omega_C(v_R,v_L,L);
    
    printf("linear velocity v_c = %.2f m/s\n", v_C);
    printf("angular velocity omega_c = %.2f rad/s\n", omega_C);
    print_state(&rover);
    int steps = (int)round(T / dt);
    for (int i = 0; i < steps; i++) // i = step
    {
        printf("Step %d\n", i + 1);
        update_state(&rover, v_C, omega_C, dt);
        print_state(&rover);
    }
    
    return 0; 
}
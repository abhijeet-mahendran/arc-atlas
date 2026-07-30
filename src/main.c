// Ctrl + Shift + B to compile and run
#include <stdio.h>
#include <math.h>
#include "rover.h"

void print_state(const RoverState *rover)
{
    // printf("Rover state:\n");
    printf("x = %.2f m\n", rover->x);
    printf("y = %.2f m\n", rover->y);
    printf("theta = %.2f rad\n", rover->theta);
}

int main() {

    double dt = 0.1; // simulation timestep in s
    double L = 2; // rover track width in m
    double v_R = 8; // right wheel speed in m/s
    double v_L = 4; // left wheel speed in m/s
    double T = 1; // simulation time in s

    rover_init(L);

    RoverState state = rover_get_state();
    print_state(&state);

    int steps = (int)round(T / dt); // simulation steps

    for (int i = 0; i < steps; i++) // i = step
    {
        rover_step(v_R, v_L, dt);

        state = rover_get_state();

        printf("Step %d - ", i + 1);
        printf("Elapsed time: %.3fs\n", (i + 1) * dt);
        print_state(&state);
    }
    
    return 0; 
}
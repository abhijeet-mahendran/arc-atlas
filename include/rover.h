#ifndef ROVER_H
#define ROVER_H

#ifdef _WIN32
    #define ROVER_API __declspec(dllexport)
#else
    #define ROVER_API
#endif

typedef struct {
    double x;
    double y;
    double theta;
} RoverState;

ROVER_API void rover_init(double new_L); // init with track width new_L

ROVER_API void rover_step(double v_R, double v_L, double dt);

ROVER_API RoverState rover_get_state();

#endif
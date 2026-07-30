from rover_api import rover # type: ignore

rover.rover_init(2.0)

print("Step 0 - Elapsed time: 0s")
state = rover.rover_get_state()
print(state.x)
print(state.y)
print(state.theta)

for i in range(10):
    print("Step " + str(i + 1) + " - Elapsed time: " + str((i + 1) * 0.1) + "s")
    rover.rover_step(8.0, 4.0, 0.1)
    state = rover.rover_get_state()
    print(f"x = {state.x:.2f}")
    print(f"y = {state.y:.2f}")
    print(f"theta = {state.theta:.2f}")
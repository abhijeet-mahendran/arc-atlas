using UnityEngine;

public class RoverDriver : MonoBehaviour
{
    [SerializeField]
    private double trackWidth = 1.0;

    [SerializeField]
    private double rightWheelVelocity = 4.0;

    [SerializeField]
    private double leftWheelVelocity = 2.0;

    [SerializeField]
    private int integrator = 3;


    private void Start()
    {
        RoverNative.rover_init(trackWidth);

        RoverNative.rover_set_integration_method(
            integrator
        );
    }


    private void FixedUpdate()
    {
        double dt = Time.fixedDeltaTime;

        RoverNative.rover_step(
            rightWheelVelocity,
            leftWheelVelocity,
            dt
        );

        RoverNative.RoverState state;

        RoverNative.rover_get_state_into(
            out state
        );

        UpdateVisual(state);
    }


    private void UpdateVisual(
        RoverNative.RoverState state
    )
    {
        transform.position = new Vector3(
            (float)state.x,
            0.0f,
            (float)state.y
        );

        Vector3 forward = new Vector3(
            Mathf.Cos((float)state.theta),
            0.0f,
            Mathf.Sin((float)state.theta)
        );

        transform.rotation =
            Quaternion.LookRotation(
                forward,
                Vector3.up
            );
    }
}
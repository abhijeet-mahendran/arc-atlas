using System.Runtime.InteropServices;

public static class RoverNative
{
    [StructLayout(LayoutKind.Sequential)]
    public struct RoverState
    {
        public double x;
        public double y;
        public double theta;
    }

    [DllImport("rover", CallingConvention = CallingConvention.Cdecl)]
    public static extern void rover_init(double new_L);

    [DllImport("rover", CallingConvention = CallingConvention.Cdecl)]
    public static extern void rover_step(
        double v_R,
        double v_L,
        double dt
    );

    [DllImport("rover", CallingConvention = CallingConvention.Cdecl)]
    public static extern void rover_get_state_into(
        out RoverState state
    );

    [DllImport("rover", CallingConvention = CallingConvention.Cdecl)]
    public static extern void rover_set_integration_method(
        int method
    );
}
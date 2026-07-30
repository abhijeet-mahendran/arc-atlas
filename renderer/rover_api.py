from pathlib import Path
import ctypes


class RoverState(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("theta", ctypes.c_double),
    ]


DLL_PATH = Path(__file__).resolve().parent.parent / "rover.dll"

rover = ctypes.CDLL(str(DLL_PATH))


rover.rover_init.argtypes = [  # init rover, inputs: L
    ctypes.c_double
]
rover.rover_init.restype = None


rover.rover_step.argtypes = [ # calculate step, inputs: v_R, v_L, dt
    ctypes.c_double,
    ctypes.c_double,
    ctypes.c_double,
]
rover.rover_step.restype = None


rover.rover_get_state.argtypes = [] # get rover state       
rover.rover_get_state.restype = RoverState 
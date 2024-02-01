import sys
import numpy as np

traffic_file_paths = {
    "CarlaTown10": (
        "limsim/networkFiles/CarlaTown10/Town10.net.xml",
        "limsim/networkFiles/CarlaTown10/carlavtypes.rou.xml,limsim/networkFiles/CarlaTown10/Town10.rou.xml",
    ),
}

class MapOffset:
    x_offset = 109.5
    y_offset = 136
    vehicle_z = 2.50
    T_car2frontcam = np.array(
        [
            [0, -0.173648178, 0.984807753, 2.0],
            [-1.0, 0, 0, 0],
            [0, -0.984807753, -0.173648178, -0.5],
            [0, 0, 0, 1.0],
        ]
    )



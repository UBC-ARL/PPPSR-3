import numpy as np
from scipy.spatial.transform import Rotation

from pppsr_3.lib.inverse_kinematics import PPPSRDimension

DIMENSION = PPPSRDimension(
    u_i=[
        Rotation.from_euler("z", 210, degrees=True).apply(111.5 * np.array([1, 0, 0])),
        Rotation.from_euler("z", 330, degrees=True).apply(111.5 * np.array([1, 0, 0])),
        Rotation.from_euler("z", 90, degrees=True).apply(111.5 * np.array([1, 0, 0])),
    ],
    O_i=[
        Rotation.from_euler("z", -60, degrees=True),
        Rotation.from_euler("z", 60, degrees=True),
        Rotation.from_euler("z", 180, degrees=True),
    ],
    b_i=[
        b1 := np.array([-12.47, -68.15, 0]),
        Rotation.from_euler("z", 120, degrees=True).apply(b1),
        Rotation.from_euler("z", 240, degrees=True).apply(b1),
    ],
    l_i=85 * np.ones(3),
)

RDOF0 = np.array(
    [
        np.rad2deg(np.atan2(-12.396, 84.09)),
        np.rad2deg(np.atan2(79.02, -31.31)),
        np.rad2deg(np.atan2(-66.63, -52.78)),
    ]
)

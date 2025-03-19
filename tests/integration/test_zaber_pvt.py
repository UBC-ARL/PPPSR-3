import tomllib
from pathlib import Path
from typing import Callable

import numpy as np
from arl_utils_py.trajectory_planning.task_space_trajectory_planning import (
    QuinticTrajectoryPlanner,
    Range,
    TaskSpaceTrajectory,
    TaskSpaceTrajectorySegment,
    TrapezoidalTrajectoryPlanner,
)
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from zaber_motion import Units
from zaber_motion.ascii import Axis, Connection, Lockstep
from zaber_motion.ascii.pvt_axis_definition import PvtAxisDefinition
from zaber_motion.ascii.pvt_axis_type import PvtAxisType

from pppsr_3 import DIMENSION, RDOF0
from tests.lib.pvt_utils import pvt_abs_move

Q0 = np.eye(3)
R0 = Rotation.from_matrix(Q0)
p0 = np.array([0, 0, 0])


def trajectory_p_R_func_raise_tilt_torsion_down_1_10_40_10_1(
    tilt_angle_deg: float,
) -> tuple[Callable[[float], tuple[NDArray, Rotation]], Range]:
    R0 = Rotation.from_matrix(Q0)
    trajectory_f = TaskSpaceTrajectory(
        [
            TaskSpaceTrajectorySegment(
                Range(0, 1),
                lambda u: p0 + u * np.array([0, 0, 0.1]),
                lambda u: R0,
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(1, 11),
                lambda u: p0 + np.array([0, 0, 0.1]),
                lambda u: Rotation.from_euler("y", u * tilt_angle_deg, degrees=True)
                * R0,
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(11, 51),
                lambda u: p0 + np.array([0, 0, 0.1]),
                lambda u: Rotation.from_rotvec(
                    Rotation.from_euler("z", u * 2 * np.pi).apply(np.array([0, 1, 0]))
                    * tilt_angle_deg,
                    degrees=True,
                )
                * R0,
                # TrapezoidalTrajectoryPlanner(v_max_unsigned=1 / (10 - 2) * 1.2),
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(51, 61),
                lambda u: p0 + np.array([0, 0, 0.1]),
                lambda u: Rotation.from_euler(
                    "y", (1 - u) * tilt_angle_deg, degrees=True
                )
                * R0,
                QuinticTrajectoryPlanner(),
            ),
            TaskSpaceTrajectorySegment(
                Range(61, 62, include_end=True),
                lambda u: p0 + (1 - u) * np.array([0, 0, 0.1]),
                lambda u: Rotation.from_euler("y", 0) * R0,
                QuinticTrajectoryPlanner(),
            ),
        ]
    )
    return trajectory_f, trajectory_f.range


COMMAND_FREQ_HZ = 30

trajectory_f, range_ = trajectory_p_R_func_raise_tilt_torsion_down_1_10_40_10_1(90)
t_list = np.linspace(
    range_.start, range_.end, (range_.end - range_.start) * COMMAND_FREQ_HZ
)
p_list, R_list = zip(*[trajectory_f(t) for t in t_list])

local_data = np.array(
    [DIMENSION.p_i_local(p, R, RDOF0) for p, R in zip(p_list, R_list)]
)


def test_zaber_pvt():
    with open(Path(__file__).parent / "config.toml", "rb") as f:
        config = tomllib.load(f)
        comport_address: str = config["comport"]
        devices_config = config["devices"]

    with Connection.open_serial_port(comport_address) as connection:
        connection.enable_alerts()
        connection.alert.subscribe(lambda alert: print(f"Alert from device: {alert}"))
        connection.detect_devices()

        # device initialization as in the config.toml
        devices: list[list[Axis | Lockstep]] = []
        for device_config in devices_config:
            axes: list[Axis | Lockstep] = []
            device = connection.get_device(device_config["address"])

            try:
                available_locksteps = iter(
                    range(1, 1 + int(device.settings.get("lockstep.numgroups")))
                )
            except Exception:
                available_locksteps = iter(range(0))

            for ax in device_config["axes"]:
                ax_num = ax["axis"]
                if isinstance(ax_num, int):
                    axes.append(device.get_axis(ax_num))
                    device.get_axis(ax_num).settings.set(
                        "maxspeed", 45, Units.VELOCITY_MILLIMETRES_PER_SECOND
                    )
                    device.get_axis(ax_num).settings.set(
                        "motion.accel.ramptime", 0, Units.TIME_MILLISECONDS
                    )

                elif isinstance(ax_num, list) or isinstance(ax_num, tuple):
                    for i in ax_num:
                        device.get_axis(i).settings.set(
                            "maxspeed", 45, Units.VELOCITY_MILLIMETRES_PER_SECOND
                        )
                        device.get_axis(i).settings.set(
                            "motion.accel.ramptime", 0, Units.TIME_MILLISECONDS
                        )
                    lockstep = device.get_lockstep(next(available_locksteps))
                    lockstep.disable()
                    lockstep.enable(*ax_num)
                    lockstep.set_tolerance(0.5, Units.LENGTH_MILLIMETRES)
                    axes.append(lockstep)
            devices.append(axes)

        device1 = connection.get_device(1)
        device2 = connection.get_device(2)
        device3 = connection.get_device(3)

        # start position definition and conversion
        leg1_start_position_driver = (
            np.diag([1, 1, -1]) @ DIMENSION.p_i_local(p0, R0, RDOF0)[0]
        ) + np.array([250, 250, 250])
        leg2_start_position_driver = (
            np.diag([1, 1, -1]) @ DIMENSION.p_i_local(p0, R0, RDOF0)[1]
        ) + np.array([250, 250, 250])
        leg3_start_position_driver = (
            np.diag([1, 1, -1]) @ DIMENSION.p_i_local(p0, R0, RDOF0)[2]
        ) + np.array([250, 250, 250])

        # move to start position
        devices[0][0].move_absolute(
            leg1_start_position_driver[0],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )
        devices[0][1].move_absolute(
            leg1_start_position_driver[1],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )
        devices[0][2].move_absolute(
            leg1_start_position_driver[2],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )

        devices[1][0].move_absolute(
            leg2_start_position_driver[0],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )
        devices[1][1].move_absolute(
            leg2_start_position_driver[1],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )
        devices[1][2].move_absolute(
            leg2_start_position_driver[2],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )

        devices[2][0].move_absolute(
            leg3_start_position_driver[0],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )
        devices[2][1].move_absolute(
            leg3_start_position_driver[1],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )
        devices[2][2].move_absolute(
            leg3_start_position_driver[2],
            Units.LENGTH_MILLIMETRES,
            wait_until_idle=False,
        )

        device1.all_axes.wait_until_idle()
        device2.all_axes.wait_until_idle()
        device3.all_axes.wait_until_idle()

        # convert local data to driver data
        driver_xyz_leg1 = np.array(
            [np.diag([1, 1, -1]) @ p + 250 for p in local_data[:, 0, :]]
        )
        driver_xyz_leg2 = np.array(
            [np.diag([1, 1, -1]) @ p + 250 for p in local_data[:, 1, :]]
        )
        driver_xyz_leg3 = np.array(
            [np.diag([1, 1, -1]) @ p + 250 for p in local_data[:, 2, :]]
        )

        # Execute PVT Movement
        pvt_abs_move(
            connection,
            device1_address=1,
            device2_address=2,
            device3_address=3,
            pvt_axes_definition1=(
                PvtAxisDefinition(4, PvtAxisType.PHYSICAL),
                PvtAxisDefinition(3, PvtAxisType.PHYSICAL),
                PvtAxisDefinition(1, PvtAxisType.LOCKSTEP),
            ),
            pvt_axes_definition2=(
                PvtAxisDefinition(4, PvtAxisType.PHYSICAL),
                PvtAxisDefinition(3, PvtAxisType.PHYSICAL),
                PvtAxisDefinition(1, PvtAxisType.LOCKSTEP),
            ),
            pvt_axes_definition3=(
                PvtAxisDefinition(4, PvtAxisType.PHYSICAL),
                PvtAxisDefinition(3, PvtAxisType.PHYSICAL),
                PvtAxisDefinition(1, PvtAxisType.LOCKSTEP),
            ),
            xyz1=driver_xyz_leg1,
            xyz2=driver_xyz_leg2,
            xyz3=driver_xyz_leg3,
            dt=np.ones_like(t_list) * 1 / COMMAND_FREQ_HZ,
            zero_velocity_indices=[],
        )

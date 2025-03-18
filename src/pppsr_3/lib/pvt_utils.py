import click
from numpy.typing import NDArray
from zaber_motion import Measurement, Units
from zaber_motion.ascii import Connection
from zaber_motion.ascii.pvt_axis_definition import PvtAxisDefinition


def pvt_abs_move(
    connection: Connection,
    device1_address: int,
    device2_address: int,
    device3_address: int,
    pvt_axes_definition1: tuple[
        PvtAxisDefinition, PvtAxisDefinition, PvtAxisDefinition
    ],
    pvt_axes_definition2: tuple[
        PvtAxisDefinition, PvtAxisDefinition, PvtAxisDefinition
    ],
    pvt_axes_definition3: tuple[
        PvtAxisDefinition, PvtAxisDefinition, PvtAxisDefinition
    ],
    xyz1: NDArray,
    xyz2: NDArray,
    xyz3: NDArray,
    dt: NDArray,
    zero_velocity_indices: list[int],
    ask_confirm: bool = True,
    wait_until_idle: bool = True,
):
    """PVT Movement using absolute coordinates and store mode, with start and end point a velocity of 0.

    Args:
        connection (Connection): _description_
        device1_address (int): _description_
        device2_address (int): _description_
        device3_address (int): _description_
        xyz1 (NDArray): _description_
        xyz2 (NDArray): _description_
        xyz3 (NDArray): _description_
        t (NDArray): _description_
    """

    # PVT initialization
    device1 = connection.get_device(device1_address)
    device2 = connection.get_device(device2_address)
    device3 = connection.get_device(device3_address)

    pvt1 = device1.get_pvt(1)
    pvt1.disable()
    pvt1_buffer = device1.get_pvt_buffer(1)
    pvt1_buffer.erase()

    pvt2 = device2.get_pvt(1)
    pvt2.disable()
    pvt2_buffer = device2.get_pvt_buffer(1)
    pvt2_buffer.erase()

    pvt3 = device3.get_pvt(1)
    pvt3.disable()
    pvt3_buffer = device3.get_pvt_buffer(1)
    pvt3_buffer.erase()

    pvt1.setup_store_composite(pvt1_buffer, *pvt_axes_definition1)
    pvt2.setup_store_composite(pvt2_buffer, *pvt_axes_definition2)
    pvt3.setup_store_composite(pvt3_buffer, *pvt_axes_definition3)

    # PVT normal points
    for i, (p1, p2, p3, t_step) in enumerate(zip(xyz1, xyz2, xyz3, dt)):
        pvt1.point(
            [Measurement(p, Units.LENGTH_MILLIMETRES) for p in p1],
            (
                [None for _ in p1]
                if (i != 0 and i != len(xyz1) - 1) and i not in zero_velocity_indices
                else [Measurement(0, Units.VELOCITY_MILLIMETRES_PER_SECOND) for _ in p1]
            ),
            Measurement(t_step, Units.TIME_SECONDS),
        )
        pvt2.point(
            [Measurement(p, Units.LENGTH_MILLIMETRES) for p in p2],
            (
                [None for _ in p1]
                if (i != 0 and i != len(xyz1) - 1) and i not in zero_velocity_indices
                else [Measurement(0, Units.VELOCITY_MILLIMETRES_PER_SECOND) for _ in p1]
            ),
            Measurement(t_step, Units.TIME_SECONDS),
        )
        pvt3.point(
            [Measurement(p, Units.LENGTH_MILLIMETRES) for p in p3],
            (
                [None for _ in p1]
                if (i != 0 and i != len(xyz1) - 1) and i not in zero_velocity_indices
                else [Measurement(0, Units.VELOCITY_MILLIMETRES_PER_SECOND) for _ in p1]
            ),
            Measurement(t_step, Units.TIME_SECONDS),
        )

    # Finish writing to the sequence
    pvt1.disable()
    pvt2.disable()
    pvt3.disable()

    pvt1.setup_live_composite(*pvt_axes_definition1)
    pvt2.setup_live_composite(*pvt_axes_definition2)
    pvt3.setup_live_composite(*pvt_axes_definition3)

    print("PVT points writing finished")

    if not ask_confirm or click.confirm("Start moving?"):
        pvt1.call(pvt1_buffer)
        pvt2.call(pvt2_buffer)
        pvt3.call(pvt3_buffer)
        print("PVT move started")

    if wait_until_idle:
        pvt1.wait_until_idle()
        pvt2.wait_until_idle()
        pvt3.wait_until_idle()

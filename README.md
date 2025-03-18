# LockStep Tuner

A Python script to move multi-axis device daisy chains with some axes in lockstep mode.

## Installation

### Poetry

```bash
poetry install
```

### pip

```bash
pip install -r requirements.txt
```

## Run

```bash
python main.py
```

## Standalone Scripts

These scripts are in the [examples](./example/) folder.

- [naive_multi_axis.py](./example/naive_multi_axis.py): Multi-Axis movement using non-blocking normal move commands.
- [pvt_helix_movement.py](./example/pvt_helix_movement.py): Helix movement of an X-Y-Z gantry using the Zaber PVT functionality.
- [pvt_helix_multi_device.py](./example/pvt_helix_multi_device.py): Three X-Y-Z gantries moving in the same (relative to each of the gantry itself) helix trajectory.
- [pvt_moving_platform_multi_device.py](./example/pvt_moving_platform_multi_device.py): Three X-Y-Z gantries moving the same (relative to the ground coordinate) trajectory.
- [pvt_moving_xyz_demo.py](./example/pvt_moving_xyz_demo.py): The demo script to show z, y and x translation of the moving platform. The moving platform will move along each direction with a sinusoidal trajectory sequentially.
- [lib/inverse_kinematics.py](./example/lib/inverse_kinematics.py): The inverse kinematics calculation for the moving platform, with dimension measurement data.
- [lib/pvt_utils.py](./example/lib/pvt_utils.py): The utility functions for the PVT movement. Wraps up PVT initialization and start/stop points with a velocity of zero.
- [pvt_z_rotation.py](./example/pvt_z_rotation.py): The demo script to show the rotation of the moving platform around the z-axis, using the lib scripts.

## SolidWorks Files

- [End effector location.SLDPRT](./End%20effector%20location.SLDPRT): The SolidWorks sketch showing the moving platform dimensions.

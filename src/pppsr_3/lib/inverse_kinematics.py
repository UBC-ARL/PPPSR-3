import tomllib
from dataclasses import dataclass
from typing import Annotated, Literal, TypeVar

import numpy as np
import tomli_w
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

DType = TypeVar("DType", bound=np.generic)

Array3 = Annotated[NDArray[DType], Literal[3]]
Array3x1 = Annotated[NDArray[DType], Literal[3, 1]]
Array3x3 = Annotated[NDArray[DType], Literal[3, 3]]


@dataclass(frozen=True, slots=True)
class PPPSRDimension:
    u_i: list[Array3]  # Position of each leg
    O_i: list[Rotation]  # Orientation of each leg
    b_i: list[Array3]  # From end effector to revolute joint, in the end effector frame
    l_i: Array3  # Redundant link length

    def __post_init__(self):
        assert (
            len(self.u_i) == len(self.O_i) == len(self.b_i) == len(self.l_i)
        ), "u_i, O_i, b_i and l_i must have the same length"

    def p_i(self, p: Array3, R: Rotation, redundant_angle_deg: Array3) -> list[Array3]:
        return [
            p
            + R.apply(
                b
                - Rotation.from_euler("z", angle_deg, degrees=True).apply(
                    np.array([link_length, 0, 0])
                )
            )
            - u
            for u, b, link_length, angle_deg in zip(
                self.u_i, self.b_i, self.l_i, redundant_angle_deg
            )
        ]

    def p_i_local(
        self, p: Array3, R: Rotation, redundant_angle_deg: Array3
    ) -> list[Array3]:
        return [
            orientation.inv().apply(p_i)
            for p_i, orientation in zip(self.p_i(p, R, redundant_angle_deg), self.O_i)
        ]

    def to_dict(self) -> dict:
        """Convert the dataclass to a serializable dictionary."""
        return {
            "u_i": [u.tolist() for u in self.u_i],
            "O_i": [
                orientation.as_quat().tolist() for orientation in self.O_i
            ],  # Convert rotations to quaternions
            "b_i": [b.tolist() for b in self.b_i],
            "l_i": self.l_i.tolist(),
        }

    def save_to_toml(self, path: str):
        """Serialize and save to TOML."""
        with open(path, "wb") as f:
            f.write(tomli_w.dumps(self.to_dict()).encode())

    @classmethod
    def load_from_toml(cls, path: str):
        """Load from TOML file and deserialize."""
        with open(path, "rb") as f:
            return cls.from_dict(tomllib.load(f))

    @classmethod
    def from_dict(cls, data: dict):
        """Create an instance from a dictionary."""
        return cls(
            u_i=[np.array(u) for u in data["u_i"]],
            O_i=[Rotation.from_quat(orientation) for orientation in data["O_i"]],
            b_i=[np.array(b) for b in data["b_i"]],
            l_i=np.array(data["l_i"]),
        )

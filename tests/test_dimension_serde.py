from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pppsr_3 import DIMENSION, PPPSRDimension


def test_dimension_serde(tmp_path):
    toml_path = tmp_path / Path("dimension_output.toml")
    DIMENSION.save_to_toml(toml_path)
    assert toml_path.exists()

    dimension_loaded = PPPSRDimension.load_from_toml(toml_path)

    for f in DIMENSION.__dataclass_fields__:
        for a, b in zip(
            np.array(DIMENSION.__getattribute__(f)).flatten().tolist(),
            np.array(dimension_loaded.__getattribute__(f)).flatten().tolist(),
        ):
            if type(a) is Rotation:
                assert a.approx_equal(b)  # type: ignore
            else:
                assert a == b

    toml_path.unlink()
    assert not toml_path.exists()


if __name__ == "__main__":
    test_dimension_serde(Path.cwd())

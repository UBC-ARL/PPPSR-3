# PPPSR-3

The inverse kinematics of the [3-PPPSR parallel robot](https://youtu.be/dSiWhBYOIuw?si=0RrZgnmPRrChVzVT).

## Installation

This project is built with [Poetry](https://python-poetry.org/).

```bash
poetry install
```

## Usage

Please see the [tests](./tests/) for example usage.

## Run Tests

```bash
poetry run pytest path/to/test.py
```

Please modify [config.toml](tests/integration/config.toml) according to your connection, before running [test_zaber_pvt.py](tests/integration/test_zaber_pvt.py).

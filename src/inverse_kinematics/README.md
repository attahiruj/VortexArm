# inverse_kinematics

## Overview

The `inverse_kinematics` module provides tools and algorithms for solving inverse kinematics to calculate joint parameters that achieve a desired position of the robot's end effector.

## Simulator

The `inverse_kinematics` module includes a simulator to visualize and test the results of the inverse kinematics calculations. The simulator provides a graphical interface to observe the robot's movements and control the manipulator.

## Getting started

- Install it with uv:

```bash
uv venv -p 3.11 --seed
uv pip install -e .
```

## Contribution Guide

- Format with [ruff](https://docs.astral.sh/ruff/):

```bash
uv pip install ruff
uv run ruff check . --fix
```

- Lint with ruff:

```bash
uv run ruff check .
```

- Test with [pytest](https://github.com/pytest-dev/pytest)

```bash
uv pip install pytest
uv run pytest . # Test
```

## YAML Specification

## Examples

## License

inverse_kinematics's code are released under the MIT License

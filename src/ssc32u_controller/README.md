# ssc32u_controller

`ssc32u_controller` is a module designed to interface with the SSC-32U servo controller, providing an easy-to-use API for controlling servos in robotics projects. This library is written in Python and is intended to simplify the process of sending commands to the SSC-32U.

## Features

- Send servo movement commands to the SSC-32U.
- Query the status of the controller.
- Support for custom command sequences.
- Easy integration into robotics projects.


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

ssc32u_controller's code are released under the MIT License

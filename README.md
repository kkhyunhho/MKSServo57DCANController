# MKS Motor Control

Python driver for the **MKS SERVO57D** CAN stepper motor,
controlled via a USB2CAN-FIFO (FTDI FT245) adapter.

This module is part of a larger PipetteStation project.
Eventually it will be wrapped in a GUI executable for full
station control, but for now it focuses on motor control
only.

## Hardware

| Item | Detail |
|------|--------|
| Motor | MKS SERVO57D (CAN stepper) |
| Adapter | USB2CAN-FIFO (FTDI FT245) |
| Mechanics | Ball screw, 3.75 mm/turn |
| Travel | 0 - 450 mm |

## Installation

```bash
pip install ftd2xx
```

The FTDI **D2XX** driver must also be installed on the
system. On Windows, this usually comes with the FTDI
adapter drivers.

## Usage

### Quick run

See the `if __name__ == "__main__":` block at the
bottom of [mks_motor.py](mks_motor.py).

### Direct control

```python
from mks_motor import MKSMotor

motor = MKSMotor.open(port=0, can_id=0x01)
motor.setup()
motor.home()

motor.move_to(100, speed_pct=50)
motor.move_to(200)
motor.move_to(0)

motor.close()
```

### Multiple motors

See [running_test.py](running_test.py) for an example
of controlling two motors at the same time using
threads.

## Public API

All methods belong to the `MKSMotor` class.
`open()` and `main()` are class methods, called as
`MKSMotor.open(...)`. The rest are instance methods,
called on a motor object (e.g. `motor.setup()`).

| Method | Description |
|--------|-------------|
| `open(port, can_id)` | Open FTDI device and return a motor instance |
| `main(mm, ...)` | One-shot: open, setup, home, move, close |
| `setup()` | Apply default motor settings |
| `home(speed_rpm)` | Run homing sequence and enable limit switches |
| `move_to(mm, speed_pct, accel_pct)` | Move to absolute position in mm |
| `enable()` | Energize motor coils |
| `disable()` | De-energize motor coils |
| `set_zero()` | Set current position as the zero point |
| `read_status()` | Read motor status |
| `manual_send(cmd, *data)` | Send a raw CAN command |
| `close()` | Close the FTDI device |

## File structure

```
mks_motor/
├── mks_motor.py       # MKSMotor class (library)
├── running_test.py    # Test script for dual motors
├── CommonClaude.md    # Project conventions
└── README.md          # This file
```

## Roadmap

- [x] Single motor control
- [x] Dual motor control
- [ ] GUI executable for the full PipetteStation

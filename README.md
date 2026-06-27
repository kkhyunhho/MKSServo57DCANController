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
| Mechanics | Ball screw, 10 mm/turn |
| Travel | 0 - 450 mm |

## Installation

```bash
pip install ftd2xx
```

The FTDI **D2XX** driver must also be installed on the
system. On Windows, this usually comes with the FTDI
adapter drivers.

## Hardware setup

Before running the driver, the motor and adapter must be
configured. Otherwise the CAN link will not come up.

### CAN baud rate

The USB2CAN-FIFO adapter runs at **1 Mbps**. The motor's
CAN baud rate must be set to the same value through the
on-motor menu. If the rates do not match, no frames are
received.

### DIP switches

Three DIP switches sit under the display on the right-hand
side of the motor. Push all three to the **left** position.
This:

- enables the end-stop inputs to work reliably, and
- removes the need to wire external power to the
  USB2CAN adapter (the motor side powers the bus).

### Cable connection order

Connect the cables in this order, every time:

1. **Power cable** first.
2. **CAN (communication) cable** second.

If the CAN cable is plugged in before power, the link will
not come up.

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
| `move_sync(motors, moves, barrier)` | Move multiple motors in sync |
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
├── SDLClaude.md    # Project conventions
└── README.md          # This file
```

## Roadmap

- [x] Single motor control
- [x] Dual motor control
- [ ] GUI executable for the full PipetteStation

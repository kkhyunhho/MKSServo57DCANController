"""MKS SERVO57D CAN motor driver — standalone test-harness variant.

This is the simpler, single-axis / no-touchscreen variant kept for bench
tests. The full driver (with paired-axis interlocks, etc.) lives in
ESP32S3BOX3MotorController. See CLAUDE.md.

    from mks_motor import MKSMotor
"""

from .mks_motor import MKSMotor

__all__ = ["MKSMotor"]

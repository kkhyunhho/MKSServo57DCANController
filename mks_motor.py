# mks_motor.py
# MKS SERVO57D CAN Motor Controller via USB2CAN-FIFO (FTDI FT245)
#
# Architecture (bottom to top):
#   USB2CAN binary packet (18 bytes) wraps a CAN message
#   (8 bytes max) which contains:
#   [cmd_code] [data...] [checksum]
#
# Reading Guide:
#   1. __main__          - Entry point
#   2. send / wait       - Core CAN communication
#   3. setup / home      - Motor initialization
#   4. move_to           - High-level motion commands
#   5. Motor control     - enable, disable, set_zero, read_status

import time

import ftd2xx as ftdi


class MKSMotor:
    """Controls MKS SERVO57D via USB2CAN-FIFO adapter."""

    # --- Constants ---

    # Mechanical / motor limits
    _mm_per_turn = 3.75
    _encoder_per_turn = 16384
    _max_speed_rpm = 3000
    _max_accel = 255
    _max_travel_mm = 450
    _max_wait_sec = 250

    # FTDI device setup (USB2CAN-FIFO adapter)
    _ftdi_bitmode_mask = 0xFF
    _ftdi_bitmode_async = 0x40
    _ftdi_read_timeout_ms = 100
    _ftdi_write_timeout_ms = 100
    _ftdi_purge_rx_tx = 1 | 2

    # CAN response read retry policy
    _response_retry_count = 5
    _response_retry_delay_s = 0.05

    # Limit-stop recovery: the MKS firmware ignores the
    # first motion command after a limit stop, so a tiny
    # dummy move is issued to consume that skip.
    _limit_recover_delay_s = 0.5
    _dummy_move_offset_mm = 0.01
    _dummy_move_speed_rpm = 300

    _motion_cmds = {0x91, 0xF4, 0xF5, 0xF6, 0xFD, 0xFE}

    _motion_status = {
        0x00: "Failed",
        0x01: "Running",
        0x02: "Complete",
        0x03: "Stopped by Limit",
        0x05: "Sync Data Received",
    }

    _setting_status = {
        0x00: "Failed",
        0x01: "Success",
    }

    # --- Construction ---

    def __init__(self, dev, can_id=0x01):
        self.dev = dev
        self.can_id = can_id

    @classmethod
    def open(cls, port=0, can_id=0x01):
        """Open FTDI device and return a ready MKSMotor.

        Args:
            port: FTDI device index (default 0).
            can_id: CAN bus ID for this motor.

        Returns:
            Configured MKSMotor instance.
        """
        dev = ftdi.open(port)
        dev.setBitMode(cls._ftdi_bitmode_mask, cls._ftdi_bitmode_async)
        dev.setTimeouts(cls._ftdi_read_timeout_ms, cls._ftdi_write_timeout_ms)
        dev.purge(cls._ftdi_purge_rx_tx)
        time.sleep(0.1)
        return cls(dev, can_id)

    def close(self):
        """Close the underlying FTDI device."""
        if self.dev:
            self.dev.close()
            print("\nDevice closed.")

    # --- Internal helpers ---

    @staticmethod
    def _clamp(value, low, high):
        """Enforce that value is within [low, high].

        Args:
            value: Number to check.
            low: Lower bound (inclusive).
            high: Upper bound (inclusive).

        Returns:
            The original value if within range.

        Raises:
            ValueError: If value is outside [low, high].
        """
        if value < low or value > high:
            raise ValueError(
                f"Value {value} out of range"
                f" [{low}, {high}]"
            )
        return value

    @staticmethod
    def _int16_bytes(value):
        """Split uint16 into [high, low] bytes.

        Args:
            value: Unsigned 16-bit integer.

        Returns:
            List of two bytes [high, low].
        """
        return [(value >> 8) & 0xFF, value & 0xFF]

    @staticmethod
    def _int24_bytes(value):
        """Split int24 into [high, mid, low] bytes.

        Args:
            value: 24-bit integer.

        Returns:
            List of three bytes [high, mid, low].
        """
        value_24bit = value & 0xFFFFFF
        return [
            (value_24bit >> 16) & 0xFF,
            (value_24bit >> 8) & 0xFF,
            value_24bit & 0xFF,
        ]

    # --- Unit conversions ---

    def _pct_to_speed(self, pct):
        """Convert percentage to motor RPM.

        Maps 0-100% linearly onto [0, _max_speed_rpm].

        Args:
            pct: Speed percentage (0-100).

        Returns:
            Integer RPM value.

        Raises:
            ValueError: If pct is outside [0, 100].
        """
        return int(
            self._max_speed_rpm
            * self._clamp(pct, 0, 100) / 100
        )

    def _pct_to_accel(self, pct):
        """Convert percentage to motor acceleration.

        Maps 0-100% linearly onto [0, _max_accel].

        Args:
            pct: Acceleration percentage (0-100).

        Returns:
            Integer acceleration value.

        Raises:
            ValueError: If pct is outside [0, 100].
        """
        return int(
            self._max_accel
            * self._clamp(pct, 0, 100) / 100
        )

    def _mm_to_coord(self, mm):
        """Convert mm distance to encoder coordinate.

        Args:
            mm: Distance in millimeters.

        Returns:
            Integer encoder count.

        Raises:
            ValueError: If mm is outside
                [0, _max_travel_mm].
        """
        coord = int(
            self._clamp(mm, 0, self._max_travel_mm)
            / self._mm_per_turn * self._encoder_per_turn
        )
        return coord

    # --- Low-level: CAN packet over USB2CAN ---

    def _send(self, cmd, *data, silent=False):
        """Send a CAN command and return the response.

        Builds [cmd][data...][checksum] padded to 8 bytes,
        then wraps it in an 18-byte USB2CAN binary packet.

        Args:
            cmd: MKS command code (e.g. 0xF5).
            *data: Variable-length data bytes.
            silent: Suppress TX/RX logging if True.

        Returns:
            Status byte from the motor response,
            or None if broadcast or no response.

        Raises:
            ConnectionError: If no valid response
                after retries.
        """
        data_bytes = list(data)
        dlc = 1 + len(data_bytes) + 1
        if dlc > 8:
            print(
                f"[ERROR] Too much data"
                f" ({dlc} bytes, max 8)"
            )
            return None

        checksum = (
            self.can_id + cmd + sum(data_bytes)
        ) & 0xFF
        motor_bytes = [cmd] + data_bytes + [checksum]
        motor_bytes += [0x00] * (8 - len(motor_bytes))

        # USB2CAN binary packet (18 bytes total)
        # See USB2CAN manual section 3.2.2
        packet = bytearray(18)
        packet[0] = 0x02                        # STX
        packet[1] = 0x00                        # Type
        packet[2] = dlc                         # DLC
        packet[3] = 0x00                        # Flags
        packet[4:8] = self.can_id.to_bytes(     # CAN ID
            4, 'little'
        )
        packet[8:16] = bytes(motor_bytes)       # Data
        packet[16] = sum(packet[1:16]) & 0xFF   # Checksum
        packet[17] = 0x03                       # ETX

        self.dev.purge(1)
        self.dev.write(bytes(packet))
        if not silent:
            data_hex = (
                bytes(data_bytes).hex().upper() or '(no data)'
            )
            print(f"[TX] 0x{cmd:02X} {data_hex}")

        if self.can_id == 0x00:
            if not silent:
                print(
                    "[TX] Broadcast"
                    " -- no response expected"
                )
            return None

        resp = b''
        for _ in range(self._response_retry_count):
            time.sleep(self._response_retry_delay_s)
            resp = self.dev.read(18)
            if len(resp) == 18:
                break
            self.dev.purge(1)

        if len(resp) != 18:
            raise ConnectionError(
                f"No response for 0x{cmd:02X}"
                " -- check CAN wiring, power,"
                " and bitrate"
            )

        status = resp[9]
        if not silent:
            if cmd in self._motion_cmds:
                table = self._motion_status
            else:
                table = self._setting_status
            status_label = table.get(
                status, f'Unknown 0x{status:02X}'
            )
            print(f"[RX] {status_label}")
        return status

    def _wait(self):
        """Wait for async motor response.

        Blocks until the motor reports completion,
        failure, or limit hit. Timeout resets each
        time a "Running" response arrives.

        On limit stop (0x03), the MKS firmware
        ignores the first motion command after a
        limit stop, so this dummy consumes that
        skip and restores normal operation.

        Returns:
            Status byte (0x02=complete, 0x03=limit,
            etc.), or None on timeout.
        """
        deadline = time.time() + self._max_wait_sec

        while time.time() < deadline:
            resp = self.dev.read(18)
            if len(resp) == 18:
                status = resp[9]
                label = self._motion_status.get(
                    status, f'0x{status:02X}'
                )
                print(f"[RX] {label}")

                if status == 0x01:
                    deadline = (
                        time.time() + self._max_wait_sec
                    )
                    continue
                if status == 0x03:
                    print(
                        "[LIMIT] Motor stopped"
                        " by limit switch"
                    )
                    time.sleep(self._limit_recover_delay_s)
                    self.dev.purge(1)
                    coord = int(
                        self._dummy_move_offset_mm
                        / self._mm_per_turn
                        * self._encoder_per_turn
                    )
                    dummy = (
                        self._int16_bytes(
                            self._dummy_move_speed_rpm
                        )
                        + [0]
                        + self._int24_bytes(coord)
                    )
                    self._send(
                        0xF4, *dummy, silent=True
                    )
                    time.sleep(self._limit_recover_delay_s)
                    self.dev.purge(1)
                return status

            time.sleep(0.1)

        print(
            "[ERROR] Motor not responding"
            " -- check power, wiring, and CAN"
        )
        return None

    # --- Setup & Homing ---

    def setup(self):
        """Apply default motor settings.

        Configures SR_vFOC mode and full slave response.

        Returns:
            True if all settings applied,
            False otherwise.
        """
        commands = [
            (0x82, [0x05]),
            (0x8C, [0x01, 0x01]),
        ]
        ok = True
        for cmd, data in commands:
            if self._send(cmd, *data, silent=True) != 0x01:
                ok = False

        if ok:
            print("[SETUP] OK")
        else:
            print("[SETUP] FAILED")
        return ok

    def home(self, speed_rpm=90):
        """Run homing sequence and enable limit switches.

        Finds the origin switch, sets the zero point,
        then enables limit switches for safe operation.

        HARDWARE NOTE: Motor direction is physically
        inverted due to wiring/mounting. Manual says
        0x00=CW, 0x01=CCW, but actual movement is
        opposite. Direction values are swapped here.

        Args:
            speed_rpm: Homing speed in RPM (default 90).
        """
        print(
            f"{'='*40}\n"
            f"HOMING (speed={speed_rpm} RPM)\n"
            f"{'='*40}"
        )
        speed_bytes = self._int16_bytes(speed_rpm)

        self._send(
            0x90, 0x00, 0x01, *speed_bytes, 0x00, 0x00
        )

        self._send(0x91)
        print("Moving toward origin switch...")
        status = self._wait()

        if status == 0x02:
            print("Homing complete. Zero point set.")
            self._send(
                0x90, 0x00, 0x01, *speed_bytes, 0x01, 0x00
            )
            print("Limit switches enabled.")

            coord = int(
                self._dummy_move_offset_mm
                / self._mm_per_turn
                * self._encoder_per_turn
            )
            dummy = (
                self._int16_bytes(self._dummy_move_speed_rpm)
                + [0]
                + self._int24_bytes(coord)
            )
            self._send(0xF5, *dummy, silent=True)
            time.sleep(self._limit_recover_delay_s)
            self.dev.purge(1)
        elif status == 0x00:
            print("Homing FAILED. Check switch wiring.")
        else:
            print(f"Homing ended: {status}")

    # --- High-level motion ---

    def move_to(self, mm, speed_pct=20, accel_pct=10):
        """Move to absolute position in mm.

        Uses F5H coordinate-based absolute motion
        (manual section 11.4). Ball screw converts
        mm to encoder counts.

        Args:
            mm: Target position in millimeters.
            speed_pct: Speed as 0-100% of max RPM.
            accel_pct: Acceleration as 0-100% of max.
        """
        speed = self._pct_to_speed(speed_pct)
        accel = self._pct_to_accel(accel_pct)
        coord = self._mm_to_coord(mm)

        motion_data = (
            self._int16_bytes(speed)
            + [accel]
            + self._int24_bytes(coord)
        )
        print(
            f"  Moving to {mm}mm"
            f" (speed={speed}RPM, accel={accel},"
            f" coord=0x{coord:06X})"
        )

        initial = self._send(0xF5, *motion_data)

        if initial == 0x01:
            return self._wait()

        if initial:
            print(
                f"[ERROR] Motor failed to start"
                f" (status=0x{initial:02X})"
            )
        else:
            print("[ERROR] No response")
        return initial

    # --- Manual command ---

    def manual_send(self, cmd, *data):
        """Send a raw CAN command to the motor.

        For motion commands (e.g. 0xF5), waits for
        completion automatically.

        Args:
            cmd: MKS command code in hex (e.g. 0xF5).
            *data: Variable-length data bytes.

        Returns:
            Status byte from motor response.
        """
        if cmd in self._motion_cmds:
            initial = self._send(cmd, *data)
            if initial == 0x01:
                return self._wait()
            return initial
        return self._send(cmd, *data)

    # --- Motor control ---

    def set_zero(self):
        """Set current position as zero point.

        Returns:
            Status byte from motor response.
        """
        return self._send(0x92)

    def enable(self):
        """Enable motor (energize coils).

        Returns:
            Status byte from motor response.
        """
        return self._send(0xF3, 0x01)

    def disable(self):
        """Disable motor (de-energize coils).

        Returns:
            Status byte from motor response.
        """
        return self._send(0xF3, 0x00)

    def read_status(self):
        """Read motor status.

        Returns:
            Status byte from motor response.
        """
        return self._send(0xF1)

    # --- Entry Point ---

    @classmethod
    def main(
        cls, mm, speed_pct=20, accel_pct=10,
        port=0, can_id=0x01,
    ):
        """Open, setup, home, move, and close.
        It can be modified as user's purpose.

        Single entry point for complete motor operation.

        Args:
            mm: Target position in millimeters.
            speed_pct: Speed as 0-100% of max RPM.
            accel_pct: Acceleration as 0-100% of max.
            port: FTDI device index (default 0).
            can_id: CAN bus ID for this motor.
        """
        motor = None
        try:
            motor = cls.open(port=port, can_id=can_id)
            motor.setup()
            motor.home()
            motor.move_to(mm, speed_pct, accel_pct)

        except Exception as e:
            print(f"[ERROR] {e}")
        finally:
            if motor:
                motor.close()


if __name__ == "__main__":
    MKSMotor.main(mm=100, speed_pct=20, accel_pct=10)

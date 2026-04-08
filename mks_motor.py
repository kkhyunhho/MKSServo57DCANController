# BasicDefinitionsforRunning_v2.py
# MKS SERVO57D CAN Motor Controller via USB2CAN-FIFO (FTDI FT245)
#
# Architecture (bottom to top):
#   USB2CAN binary packet (18 bytes) wraps a CAN message (8 bytes max)
#   which contains: [cmd_code] [data...] [checksum]
#
# Reading Guide:
#   1. __main__          - Entry point: open FTDI, setup, home, menu
#   2. send / wait       - Core CAN communication
#   3. setup / home      - Motor initialization sequences
#   4. move_to           - High-level motion commands
#   5. interactive_menu  - User interface

import time

import ftd2xx as ftdi


# --- Constants ---

mm_per_turn = 3.75          # Ball screw pitch
encoder_per_turn = 16384    # 16384 encoder counts = 360 degrees
max_speed_rpm = 3000
max_accel = 255
max_coord = 8388607         # int24 max
max_travel_mm = 450         # Ball screw physical limit
max_wait_sec = 250          # Max wait for motor response (1% speed full travel ≈ 240s)

# Commands that trigger async motor responses (motion in progress -> complete)
motion_cmds = {0x91, 0xF4, 0xF5, 0xF6, 0xFD, 0xFE}

motion_status = {
    0x00: "Failed",
    0x01: "Running",
    0x02: "Complete",
    0x03: "Stopped by Limit",
    0x05: "Sync Data Received",
}

setting_status = {
    0x00: "Failed",
    0x01: "Success",
}

def clamp(value, low, high):
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
            f"Value {value} out of range [{low}, {high}]"
        )
    return value


def pct_to_speed(pct):
    """Convert 0-100% to 0-3000 RPM.

    Args:
        pct: Speed percentage (0-100).

    Returns:
        Integer RPM value.

    Raises:
        ValueError: If pct is outside [0, 100].
    """
    return int(max_speed_rpm * clamp(pct, 0, 100) / 100)


def pct_to_accel(pct):
    """Convert 0-100% to 0-255.

    Args:
        pct: Acceleration percentage (0-100).

    Returns:
        Integer acceleration value (0-255).

    Raises:
        ValueError: If pct is outside [0, 100].
    """
    return int(max_accel * clamp(pct, 0, 100) / 100)


def mm_to_coord(mm):
    """Convert mm distance to encoder coordinate value.

    Args:
        mm: Distance in millimeters (0 to max_travel_mm).

    Returns:
        Integer encoder count for the given distance.

    Raises:
        ValueError: If mm is outside [0, max_travel_mm].
    """
    coord = int(
        clamp(mm, 0, max_travel_mm)
        / mm_per_turn * encoder_per_turn
    )
    return coord


def int16_bytes(value):
    """Split uint16 into [high, low] bytes.

    Args:
        value: Unsigned 16-bit integer.

    Returns:
        List of two bytes [high, low].
    """
    return [(value >> 8) & 0xFF, value & 0xFF]


def int24_bytes(value):
    """Split int24 into [high, mid, low] bytes.

    Args:
        value: 24-bit integer.

    Returns:
        List of three bytes [high, mid, low].
    """
    v = value & 0xFFFFFF
    return [(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]


def open_device(port=0):
    """Open and configure FTDI USB2CAN device.

    Args:
        port: FTDI device index (default 0).

    Returns:
        Configured ftd2xx device handle.
    """
    dev = ftdi.open(port)
    dev.setBitMode(0xFF, 0x40)  # FT245 synchronous FIFO mode
    dev.setTimeouts(100, 100)
    dev.purge(1 | 2)
    time.sleep(0.1)
    return dev


# --- Motor Controller ---

class MKSMotor:
    """Controls MKS SERVO57D via USB2CAN-FIFO adapter."""

    def __init__(self, dev, can_id=0x01):
        self.dev = dev
        self.can_id = can_id

    # ---- Low-level: CAN packet over USB2CAN ----

    def send(self, cmd, *data, silent=False):
        """Send a CAN command and return the response status.

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
        data = list(data)
        dlc = 1 + len(data) + 1  # cmd + data + checksum
        if dlc > 8:
            print(f"[ERROR] Too much data ({dlc} bytes, max 8)")
            return None

        # MKS checksum: (can_id + cmd + data_bytes) & 0xFF
        checksum = (self.can_id + cmd + sum(data)) & 0xFF
        motor_bytes = [cmd] + data + [checksum]
        motor_bytes += [0x00] * (8 - len(motor_bytes))  # pad to 8

        # USB2CAN binary packet (18 bytes total)
        # See USB2CAN manual section 3.2.2
        pkt = bytearray(18)
        pkt[0] = 0x02                                 # STX
        pkt[1] = 0x00                                 # Type: Message
        pkt[2] = dlc                                  # DLC
        pkt[3] = 0x00                                 # Flags: Standard frame
        pkt[4:8] = self.can_id.to_bytes(4, 'little')  # CAN ID
        pkt[8:16] = bytes(motor_bytes)                # Motor data
        pkt[16] = sum(pkt[1:16]) & 0xFF               # USB2CAN checksum
        pkt[17] = 0x03                                # ETX

        self.dev.purge(1)
        self.dev.write(bytes(pkt))
        if not silent:
            print(f"[TX] 0x{cmd:02X} {bytes(data).hex().upper() or '(no data)'}")

        # Broadcast/group IDs get no response per MKS manual
        if self.can_id == 0x00:
            if not silent:
                print("[TX] Broadcast -- no response expected")
            return None

        # Retry a few times with increasing delay in case of USB/CAN latency
        resp = b''
        for attempt in range(5):
            time.sleep(0.05)
            resp = self.dev.read(18)
            if len(resp) == 18:
                break
            self.dev.purge(1)

        if len(resp) != 18:
            raise ConnectionError(
            f"No response for 0x{cmd:02X}"
            " -- check CAN wiring, power, and bitrate"
        )

        status = resp[9]
        if not silent:
            if cmd in motion_cmds:
                table = motion_status
            else:
                table = setting_status
            print(f"[RX] {table.get(status, f'Unknown 0x{status:02X}')}")
        return status


    def wait(self):
        """Wait for async motor response.

        Blocks until the motor reports completion,
        failure, or limit hit. Timeout resets each
        time a "Running" response arrives.

        Returns:
            Status byte (0x02=complete, 0x03=limit,
            etc.), or None on timeout.
        """
        deadline = time.time() + max_wait_sec

        while time.time() < deadline:
            resp = self.dev.read(18)
            if len(resp) == 18:
                status = resp[9]
                print(f"[RX] {motion_status.get(status, f'0x{status:02X}')}")

                if status == 0x01:      # Still running -- keep waiting
                    deadline = time.time() + max_wait_sec
                    continue
                if status == 0x03:
                    print("[LIMIT] Motor stopped by limit switch")
                    # Dummy move: firmware ignores first motion after limit stop
                    # Use F4H (relative) with tiny distance so it works regardless
                    # of current position
                    time.sleep(0.5)
                    self.dev.purge(1)
                    coord = int(0.01 / mm_per_turn * encoder_per_turn)
                    dummy = int16_bytes(300) + [0] + int24_bytes(coord)
                    self.send(0xF4, *dummy, silent=True)
                    time.sleep(0.5)
                    self.dev.purge(1)
                return status

            time.sleep(0.1)

        print("[ERROR] Motor not responding -- check power, wiring, and CAN connection")
        return None

    # ---- Setup & Homing ----

    def setup(self):
        """Apply default motor settings.

        Configures SR_vFOC mode and full slave response.

        Returns:
            True if all settings applied, False otherwise.
        """
        commands = [
            (0x82, [0x05]),         # SR_vFOC mode
            (0x8C, [0x01, 0x01]),   # Full slave response (XX=1, YY=1)
        ]
        ok = True
        for cmd, d in commands:
            if self.send(cmd, *d, silent=True) != 0x01:
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
        print(f"{'='*40}\nHOMING (speed={speed_rpm} RPM)\n{'='*40}")
        spd = int16_bytes(speed_rpm)

        # Set homing params: trigger=Low, dir=0x01(=actual CW), speed, no limit, origin switch
        self.send(0x90, 0x00, 0x01, *spd, 0x00, 0x00)

        # Execute homing
        self.send(0x91)
        print("Moving toward origin switch...")
        status = self.wait()

        if status == 0x02:
            print("Homing complete. Zero point set.")
            # Enable limit switches, dir=0x01 to match homing direction
            self.send(0x90, 0x00, 0x01, *spd, 0x01, 0x00)
            print("Limit switches enabled.")

            # Dummy move: firmware ignores first motion after limit enable/re-enable
            coord = int(0.01 / mm_per_turn * encoder_per_turn)
            dummy = int16_bytes(300) + [0] + int24_bytes(coord)
            self.send(0xF5, *dummy, silent=True)
            time.sleep(0.5)
            self.dev.purge(1)
        elif status == 0x00:
            print("Homing FAILED. Check switch wiring.")
        else:
            print(f"Homing ended: {status}")

    # ---- High-level motion ----

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
        spd = pct_to_speed(speed_pct)
        acc = pct_to_accel(accel_pct)
        coord = mm_to_coord(mm)

        data = int16_bytes(spd) + [acc] + int24_bytes(coord)
        print(
            f"  Moving to {mm}mm"
            f" (speed={spd}RPM, accel={acc},"
            f" coord=0x{coord:06X})"
        )

        self._send_and_wait(0xF5, data)

    def _send_and_wait(self, cmd, data):
        """Send motion command and wait for completion."""
        initial = self.send(cmd, *data)

        if initial == 0x01:
            return self.wait()

        if initial:
            print(f"[ERROR] Motor failed to start (status=0x{initial:02X})")
        else:
            print("[ERROR] No response")
        return initial

    # ---- Interactive Menu ----

    def interactive_menu(self):
        """Run the text-based motor control interface.

        Loops until the user types 'exit'.
        """
        while True:
            try:
                header = (
                    f'{"="*40}\n'
                    f'  Motor Control  |'
                    f'  CAN ID: 0x{self.can_id:02X}\n'
                    f'{"="*40}'
                )
                print(header)
                print("  1. Move to position (mm)")
                print("  2. Settings")
                print("  3. Manual command (hex)")
                print("  ────────────────────────")
                print("  0. Re-run setup")
                print("  exit. Quit")

                choice = input(">> ").strip()

                if choice == "1":
                    self._menu_move_to()
                elif choice == "2":
                    self._menu_settings()
                elif choice == "3":
                    self._menu_manual()
                elif choice == "0":
                    self.setup()
                elif choice.lower() == "exit":
                    break
                else:
                    print("[INPUT] Invalid choice")

            except Exception as e:
                print(f"\n[ERROR] {e}")

    def _menu_move_to(self):
        print("  Format: [Speed%] [Accel%] [Distance_mm]")
        print("  Example: 20 10 3.75")
        raw = input(">> ")
        try:
            parts = raw.split()
            self.move_to(
                mm=float(parts[2]),
                speed_pct=float(parts[0]),
                accel_pct=float(parts[1]),
            )
        except (IndexError, ValueError) as e:
            print(f"[INPUT] {e}")

    def _menu_settings(self):
        settings = [
            # Motion
            ("Run homing",      lambda: self.home(
                speed_rpm=int(input("  Homing speed RPM [90]: ") or "90"))),
            ("Set zero point",  lambda: self.send(0x92)),
            ("Enable motor",    lambda: self.send(0xF3, 0x01)),
            ("Disable motor",   lambda: self.send(0xF3, 0x00)),
            # Diagnostics
            ("Read status",     lambda: self.send(0xF1)),
        ]
        while True:
            print(f'\n{"-"*40}\n  Settings\n{"-"*40}')
            for i, (name, _) in enumerate(settings, 1):
                print(f"  {i}. {name}")
            print("  0. Back")

            choice = input(">> ").strip()
            if choice == "0":
                return
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(settings):
                    settings[idx][1]()
                else:
                    print("[INPUT] Invalid choice")
            except (ValueError, Exception) as e:
                print(f"[ERROR] {e}")

    def _menu_manual(self):
        try:
            cmd = int(input("  Command code (hex): "), 16)
            d_raw = input("  Data bytes (hex, or Enter): ")
            if d_raw:
                data = [int(x, 16) for x in d_raw.split()]
            else:
                data = []
            if cmd in motion_cmds:
                self._send_and_wait(cmd, data)
            else:
                self.send(cmd, *data)
        except ValueError:
            print("[INPUT] Invalid hex")


# --- Entry Point ---

if __name__ == "__main__":
    dev = None
    try:
        dev = open_device()
        motor = MKSMotor(dev, can_id=0x01)

        if motor.setup():
            motor.home()
        else:
            print("[WARN] Setup failed -- entering menu for manual setup")
        motor.interactive_menu()

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if dev:
            dev.close()
            print("\nDevice closed.")

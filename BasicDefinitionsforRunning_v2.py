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
#   4. move_to / spin    - High-level motion commands
#   5. interactive_menu  - User interface

import time

import ftd2xx as ftdi


# --- Constants ---

MM_PER_TURN = 3.75          # Ball screw pitch
ENCODER_PER_TURN = 16384    # 16384 encoder counts = 360 degrees
MAX_SPEED_RPM = 3000
MAX_ACCEL = 255
MAX_COORD = 8388607         # int24 max
MAX_TRAVEL_MM = 450         # Ball screw physical limit
MAX_WAIT_SEC = 250         # Max wait for motor response (1% speed full travel ≈ 240s)

# Commands that trigger async motor responses (motion in progress -> complete)
MOTION_CMDS = {0x91, 0xF4, 0xF5, 0xF6, 0xFD, 0xFE}

MOTION_STATUS = {
    0x00: "Failed",
    0x01: "Running",
    0x02: "Complete",
    0x03: "Stopped by Limit",
    0x05: "Sync Data Received",
}

SETTING_STATUS = {
    0x00: "Failed",
    0x01: "Success",
}

def clamp(value, low, high):
    if value < low or value > high:
        raise ValueError(f"Value {value} out of range [{low}, {high}]")
    return value


def pct_to_speed(pct):
    """Convert 0-100% to 0-3000 RPM."""
    return int(MAX_SPEED_RPM * clamp(pct, 0, 100) / 100)


def pct_to_accel(pct):
    """Convert 0-100% to 0-255."""
    return int(MAX_ACCEL * clamp(pct, 0, 100) / 100)


def mm_to_coord(mm):
    """Convert mm distance to encoder coordinate value."""
    coord = int(clamp(mm, 0, MAX_TRAVEL_MM) / MM_PER_TURN * ENCODER_PER_TURN)
    return coord


def int16_bytes(value):
    """Split uint16 into [high, low] bytes."""
    return [(value >> 8) & 0xFF, value & 0xFF]


def int24_bytes(value):
    """Split int24 into [high, mid, low] bytes."""
    v = value & 0xFFFFFF
    return [(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]


# --- Motor Controller ---

class MKSMotor:
    """Controls MKS SERVO57D via USB2CAN-FIFO adapter."""

    def __init__(self, dev, can_id=0x01):
        self.dev = dev
        self.can_id = can_id
        self._retry_after_limit = False

    # ---- Low-level: CAN packet over USB2CAN ----

    def send(self, cmd, *data, silent=False):
        """Send a CAN command and return the immediate response status byte.

        Builds: [cmd] [data...] [checksum] padded to 8 bytes (MKS motor protocol),
        then wraps it in an 18-byte USB2CAN binary packet.
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

        # Read immediate response (should always arrive -- we use full response mode)
        time.sleep(0.05)
        resp = self.dev.read(18)
        if len(resp) != 18:
            raise ConnectionError(f"No response for 0x{cmd:02X} -- check CAN wiring, power, and bitrate")

        status = resp[9]
        if not silent:
            if cmd in MOTION_CMDS:
                table = MOTION_STATUS
            else:
                table = SETTING_STATUS
            print(f"[RX] {table.get(status, f'Unknown 0x{status:02X}')}")
        return status

    def wait(self, auto_reset=True):
        """Wait for async motor response (motion complete, limit hit, etc.).

        Uses MAX_WAIT_SEC. Resets each time a Running response arrives.
        """
        deadline = time.time() + MAX_WAIT_SEC

        while time.time() < deadline:
            resp = self.dev.read(18)
            if len(resp) == 18:
                status = resp[9]
                print(f"[RX] {MOTION_STATUS.get(status, f'0x{status:02X}')}")

                if status == 0x01:      # Still running -- keep waiting
                    deadline = time.time() + MAX_WAIT_SEC
                    continue
                if status == 0x03 and auto_reset:
                    self._handle_limit_stop()
                return status

            time.sleep(0.1)

        print("[ERROR] Motor not responding -- check power, wiring, and CAN connection")
        return None

    def _handle_limit_stop(self):
        """Re-enable motor after limit switch stop."""
        print("[LIMIT] Re-enabling motor...")
        self.send(0xF3, 0x00, silent=True)  # Disable
        self.send(0xF3, 0x01, silent=True)  # Enable
        time.sleep(0.3)
        self.dev.purge(1)
        self._retry_after_limit = True
        print("[LIMIT] Ready")

    # ---- Setup & Homing ----

    def setup(self):
        """Apply default motor settings: SR_vFOC mode, full response."""
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
        """Run homing sequence: find origin switch, then enable limit switches.

        HARDWARE NOTE: Motor direction is physically inverted due to wiring/mounting.
        Manual says 0x00=CW, 0x01=CCW, but actual movement is opposite.
        All direction values in this method are swapped accordingly.
        """
        print(f"\n{'='*40}\nHOMING (speed={speed_rpm} RPM)\n{'='*40}")
        spd = int16_bytes(speed_rpm)

        # Set homing params: trigger=Low, dir=0x01(=actual CW), speed, no limit, origin switch
        self.send(0x90, 0x00, 0x01, *spd, 0x00, 0x00)

        # Execute homing
        self.send(0x91)
        print("Moving toward origin switch...")
        status = self.wait(auto_reset=False)

        if status == 0x02:
            print("Homing complete. Zero point set.")
            # Enable limit switches, dir=0x00(=actual CCW) to match inverted mapping
            self.send(0x90, 0x00, 0x00, *spd, 0x01, 0x00)
            self._retry_after_limit = True
            print("Limit switches enabled.")
        elif status == 0x00:
            print("Homing FAILED. Check switch wiring.")
        else:
            print(f"Homing ended: {status}")

    # ---- High-level motion ----

    def move_to(self, mm, speed_pct=20, accel_pct=10):
        """Move to absolute position in mm (F5H command).

        Uses coordinate-based absolute motion (manual section 11.4).
        Ball screw converts mm -> encoder counts.
        """
        spd = pct_to_speed(speed_pct)
        acc = pct_to_accel(accel_pct)
        coord = mm_to_coord(mm)

        data = int16_bytes(spd) + [acc] + int24_bytes(coord)
        print(f"  Moving to {mm}mm (speed={spd}RPM, accel={acc}, coord=0x{coord:06X})")

        self._send_and_wait(0xF5, data)

    def _send_and_wait(self, cmd, data):
        """Send motion command, wait for completion. Auto-retry if on limit switch.

        Handles two limit switch scenarios:
        1. send() returns non-0x01: motor refused to start (on limit)
        2. send() returns 0x01 but wait() gets 0x03: motor started then hit limit
        """
        initial = self.send(cmd, *data)

        if initial == 0x01:
            result = self.wait()
            # Scenario 2: started but immediately hit limit
            if result == 0x03 and self._retry_after_limit:
                self._retry_after_limit = False
                print("[RETRY] Hit limit after start, retrying...")
                initial = self.send(cmd, *data)
                if initial == 0x01:
                    return self.wait()
            return result

        # Scenario 1: motor refused to start
        if self._retry_after_limit:
            self._retry_after_limit = False
            print("[RETRY] Motor didn't start, re-enabling and retrying...")
            self._handle_limit_stop()
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
        """Text-based control interface."""
        while True:
            try:
                print(f'\n{"="*40}\n  Motor Control  |  CAN ID: 0x{self.can_id:02X}\n{"="*40}')
                print("  1. Move to position (mm)")
                print("  2. Settings")
                print("  3. Manual command (hex)")
                print("  ────────────────────────")
                print("  0. Re-run setup")
                print("  id. Change CAN ID")
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
                elif choice.lower() == "id":
                    self._menu_change_id()
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
        SETTINGS = [
            # Motion
            ("Run homing",      lambda: self.home(
                speed_rpm=int(input("  Homing speed RPM [90]: ") or "90"))),
            ("Set zero point",  lambda: self.send(0x92)),
            ("Enable motor",    lambda: self.send(0xF3, 0x01)),
            ("Disable motor",   lambda: self.send(0xF3, 0x00)),
            # Diagnostics
            ("Read status",     lambda: self.send(0xF1)),
            ("Emergency stop",  lambda: self.send(0xF7)),
        ]
        while True:
            print(f'\n{"-"*40}\n  Settings\n{"-"*40}')
            for i, (name, _) in enumerate(SETTINGS, 1):
                print(f"  {i}. {name}")
            print("  0. Back")

            choice = input(">> ").strip()
            if choice == "0":
                return
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(SETTINGS):
                    SETTINGS[idx][1]()
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
            self.send(cmd, *data)
            if cmd in MOTION_CMDS:
                self.wait()
        except ValueError:
            print("[INPUT] Invalid hex")

    def _menu_change_id(self):
        raw = input(f"  New CAN ID (hex, current=0x{self.can_id:02X}): ")
        try:
            new_id = int(raw, 16)
            if not (0x01 <= new_id <= 0x7FF):
                print("[INPUT] CAN ID range: 0x01 ~ 0x7FF")
                return
            self.can_id = new_id
            print(f"  CAN ID changed to 0x{self.can_id:02X}")
        except ValueError:
            print("[INPUT] Invalid hex")


# --- Entry Point ---

if __name__ == "__main__":
    dev = None
    try:
        dev = ftdi.open(0)
        dev.setBitMode(0xFF, 0x40)  # FT245 synchronous FIFO mode
        dev.setTimeouts(100, 100)
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

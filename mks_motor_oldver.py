# BasicDefinitionsForRunning.py
# MKS SERVO57D CAN Motor Controller via USB2CAN (FTDI)
#
# Reading Guide:
#   1. __main__ (bottom)     - Program entry point
#   2. __init__              - Constants, maps, and response patterns
#   3. setup_motor           - Motor initialization sequence
#   4. home_motor            - Homing sequence with limit switch setup
#   5. send_motor            - Send command and read immediate response
#   6. wait_response         - Wait for async motor response (motion complete, limit, etc.)
#   7. interactive_menu      - User interface loop
#   8. reset_after_limit     - Workaround: motor reset after limit switch stop

import ftd2xx as ftdi
import time

class MKSServo:

    def __init__(self, dev):
        self.dev = dev
        
        self.COMMAND_MAP = {
            "1": ("Absolute Position (mm)", 0xF5,
                  "Format: [Speed(0-100%)] [Accel(0-100%)] [Distance(mm)]\n"
                  "Distance: mm from zero point (0 to 450mm)\n"
                  "Ball screw: 3.75mm per turn\n"
                  "Example: 20 10 3.75\n"
                  "Example: 50 10 15"),
            "2": ("Speed Control Mode", 0xF6,
                  "Format: [CW/CCW] [Speed(0-100%)] [Accel(0-100%)]\nExample: CW 50 10"),
            "8": ("Settings", None, None),
        }

        self.SETTINGS_MAP = {
            "1": ("Motor Enable/Disable", 0xF3, {
                "0": ("Disable (Loose Shaft)", 0x00),
                "1": ("Enable (Lock Shaft)", 0x01),
            }),
            "2": ("Set Zero Point (Set Current Position = 0)", 0x92, None),
            "3": ("Set Work Mode", 0x82, {
                "0": ("CR_OPEN  (Pulse Open-Loop)", 0x00),
                "1": ("CR_CLOSE (Pulse Closed-Loop)", 0x01),
                "2": ("CR_vFOC  (Pulse FOC)", 0x02),
                "3": ("SR_OPEN  (Bus Open-Loop)", 0x03),
                "4": ("SR_CLOSE (Bus Closed-Loop)", 0x04),
                "5": ("SR_vFOC  (Bus FOC (Recommended))", 0x05),
            }),
            "4": ("Set Working Current [Default: 42D=1600mA, 57D=3200mA]", 0x83, None),
            "5": ("Set Subdivisions [Default: 16]", 0x84, None),
            "6": ("Set CAN Bit Rate [Default: 500K]", 0x8A, {
                "0": ("125K", 0x00),
                "1": ("250K", 0x01),
                "2": ("500K", 0x02),
                "3": ("1M (Recommended)", 0x03),
            }),
            "7": ("Set Slave Response [Default: Full Response]", 0x8C, {
                "0": ("Disabled (XX=0, YY=0)", [0x00, 0x00]),
                "1": ("No Active (XX=1, YY=0)", [0x01, 0x00]),
                "2": ("Full Response (XX=1, YY=1)", [0x01, 0x01]),
            }),
            "8": ("Set Motor Direction [Default: CW]", 0x86, {
                "0": ("CW", 0x00),
                "1": ("CCW", 0x01),
            }),
            "9": ("Set En Pin Level [Default: Active Low]", 0x85, {
                "0": ("Active Low", 0x00),
                "1": ("Active High", 0x01),
                "2": ("Always Enabled (Hold)", 0x02),
            }),
            "10": ("Set Homing Speed [Default: 90 RPM]", None, None),
            "11": ("Run Homing", None, None),
            "12": ("Read Motor Operating Status", 0xF1, None),
        }

        self.RESPONSE_PATTERNS = {
            "motion": {
                0x00: "Failed",
                0x01: "Running",
                0x02: "Run Complete",
                0x03: "Stopped by Limit Switch",
                0x05: "Sync Data Received",
            },
            "setting": {
                0x00: "Failed",
                0x01: "Success",
            },
        }

        self.MOTION_COMMANDS = {0x91, 0xF4, 0xF5, 0xF6, 0xFD, 0xFE}

        self._retry_after_limit = False

    def create_motor_packet(self, can_id, cmd_code, data_list=None):
    
        # 1. DLC Calculation: Command + Data + CRC
        dlc = 1 + len(data_list) + 1
        if dlc > 8:
            print(f"[INPUT] DLC Exceed Maximum (8). Current: {dlc}")
            return None, None

        # 2. Internal CRC Calculation 
        total_sum = can_id + cmd_code + sum(data_list)
        internal_crc = total_sum & 0xFF

        # 3. packet Construction
        motor_packet = [cmd_code] + data_list + [internal_crc]
        while len(motor_packet) < 8:
            motor_packet.append(0x00)

        # print(bytearray(packet_content).hex().upper())
        # print(dlc)
        return bytearray(motor_packet), dlc
    
    def send_can_message(self, can_id, motor_packet, dlc, silent=False):
       
        # STX(1) + Type(1) +DLC(1) + Flags(1) + ID(4) + Data(8) + Checksum(1) + ETX(1) = Total 18byte

        packet = bytearray(18)
        packet[0] = 0x02    # STX
        packet[1] = 0x00    # Type: Message Packet
        packet[2] = dlc     # DLC (Range: 0~8)
        packet[3] = 0x00    # Flags: CAN Standard Data Frame
    
        # ID Setting
        packet[4:8] = can_id.to_bytes(4, 'little') # Endian Format

        # Data for Motor Driver
        for i in range(min(len(motor_packet), 8)):
            packet[8+i] = motor_packet[i]

        # CRC and ETX
        packet[16] = sum(packet[1:16]) & 0xFF
        packet[17] = 0x03

        self.dev.write(bytes(packet))
        if not silent:
            print(f"[SENT] CAN ID {can_id}: {motor_packet.hex().upper()}")

    def send_motor(self, can_id, cmd_code, *data_values, silent=False):

        data_list = list(data_values)
        motor_data, dlc = self.create_motor_packet(can_id, cmd_code, data_list)

        if motor_data is None:
            return None

        self.dev.purge(1) # Clear stale responses before sending new command
        self.send_can_message(can_id, motor_data, dlc, silent)

        time.sleep(0.05) # Motor Response Stand by
        response = self.dev.read(18)

        if len(response) == 18:
            status_byte = response[9]
            if not silent:
                if cmd_code in self.MOTION_COMMANDS:
                    resp_type = "motion"
                else:
                    resp_type = "setting"
                pattern = self.RESPONSE_PATTERNS[resp_type]
                status_text = pattern.get(status_byte, f"Unknown (0x{status_byte:02X})")
                print(f"[RESP] 0x{cmd_code:02X}: {status_text}")
            return status_byte
        else:
            print(f"[TIMEOUT] No Response for 0x{cmd_code:02X}")
            return None

    def reset_after_limit(self, can_id):
        print("[LIMIT] Resetting motor after limit switch stop...")

        # Reset by Disable and Re-enable
        self.send_motor(can_id, 0xF3, 0x00, silent=True)
        self.send_motor(can_id, 0xF3, 0x01, silent=True)

        time.sleep(0.3) # Wait for motor to stabilize after re-enable
        self.dev.purge(1) # Clear F3 responses before next move command
        self._retry_after_limit = True # Refer interactive_menu(): # 5.

        print("[LIMIT] Motor re-enabled. Ready to move.")

    def wait_response(self, can_id, timeout=1, auto_reset=True):
        print("[WAIT] Waiting for Motor Response...")
        MAX_TIMEOUT = 60
        abs_start = time.time()
        start = time.time()
        while time.time() - start < timeout:
            if time.time() - abs_start > MAX_TIMEOUT:
                print("[TIMEOUT] Max timeout reached.")
                return None
            response = self.dev.read(18)
            if len(response) == 18:
                cmd = response[8]
                status = response[9]
                motion_pattern = self.RESPONSE_PATTERNS["motion"]
                status_text = motion_pattern.get(status, f"Unknown (0x{status:02X})")
                print(f"[RECV] CMD: 0x{cmd:02X}, Status: {status_text}")
                if status == 0x01:
                    start = time.time() # Reset timer while motor is running
                    continue
                if status == 0x03 and auto_reset:
                    self.reset_after_limit(can_id)
                return status # result can be None if it has been [TIMEOUT]
            time.sleep(0.1)  # Delay before next read attempt
        print("[TIMEOUT] No Response Received.")
        return None


    def setup_motor(self, can_id):
        setup_commands = [
            (0x82, [0x05]),        # Set SR_vFOC Mode
            (0x8C, [0x01, 0x01]),  # Set Slave Response (XX=1, YY=1)
            (0x8A, [0x03]),        # Set CAN Rate 1M
        ]

        all_ok = True
        for cmd, data in setup_commands:
            status = self.send_motor(can_id, cmd, *data)
            if status != 0x01:
                all_ok = False

        if all_ok:
            print("[SETUP] All Settings Applied.")
        else:
            print("[SETUP] Some Settings Failed.")

        return all_ok

    def home_motor(self, can_id, home_speed=90):
        print(f"\n{'='*45}\n[HOME] Starting Homing Sequence...\n{'='*45}")

        # 1. Set homing params (90H)
        #    homeTrig=0x00 (Low level), homeDir=0x01 (CCW),
        #    homeSpeed (uint16_t), EndLimit=0x00 (disable during homing),
        #    hm_mode=0x00 (origin switch homing)
        #
        #    NOTE: Dir is INVERTED on this motor setup.
        #    Manual says 0x00=CW, 0x01=CCW, but actual behavior is opposite.
        #    homeDir=0x01 actually moves CW (toward the limit switch).
        speed_hi = (home_speed >> 8) & 0xFF
        speed_lo = home_speed & 0xFF
        self.send_motor(can_id, 0x90, 0x00, 0x01, speed_hi, speed_lo, 0x00, 0x00)

        # 2. Execute homing (91H)
        self.send_motor(can_id, 0x91)

        # 3. Wait for homing to complete
        # NOTE: Dir inverted — homeDir=0x01 actually moves CW on this motor
        print("[HOME] Moving to find origin switch...")
        status = self.wait_response(can_id, timeout=30, auto_reset=False)

        if status == 0x02:
            print("[HOME] Homing Complete. Zero point set.")

            # 4. Enable limit switches with inverted homeDir (no second 91H)
            #    homeDir=0x00 here to fix limit direction mapping
            #    (homing used 0x01, but limit direction is inverted)
            self.send_motor(can_id, 0x90, 0x00, 0x00, speed_hi, speed_lo, 0x01, 0x00)
            self._retry_after_limit = True
            print("[HOME] Limit switches enabled.")
        elif status == 0x00:
            print("[HOME] Homing Failed. Check switch wiring.")
        else:
            print(f"[HOME] Homing ended with status: 0x{status:02X}" if status else "[HOME] Homing Timeout.")

    def parse_speed_control_data(self, user_input):

        try:
            parts = user_input.split()
            if len(parts) < 3:
                print("[INPUT] Need Direction, Speed and Accel.")
                return None
            
            # 1. Direction
            #    NOTE: Dir is INVERTED on this motor setup.
            #    Manual says dir=0 is CCW, dir=1 is CW,
            #    but actual behavior is opposite.
            #    So user input CW/CCW is swapped here to match reality.
            direction_str = parts[0].upper()
            if direction_str not in ("CW", "CCW"):
                print("[INPUT] Direction Must be CW or CCW.")
                return None
            direction = 8 if direction_str == "CCW" else 0

            # 2. Speed
            speed = float(parts[1])
            if not (0 <= speed <= 100):
                print("[INPUT] Speed Range: 0% ~ 100% (Max. 3000 RPM)")
                return None
            real_speed = int(3000 * (speed / 100))

            # 3. Accel
            accel = float(parts[2])
            if not (0 <= accel <= 100):
                print("[INPUT] Accel Range: 0% ~ 100%")
                return None
            real_accel = int(255 * (accel / 100))

            # 4. Byte Construction
            byte0 = (direction << 4) | ((real_speed >> 8) & 0x0F)
            byte1 = real_speed & 0xFF
            byte2 = real_accel

            return [byte0, byte1, byte2]
        
        except ValueError:
            print("[INPUT] Incorrect Number Format")
            return None

    # Ball screw: 3.75mm per revolution
    MM_PER_TURN = 3.75
    ENCODER_PER_TURN = 16384

    def parse_abs_coordinate_data(self, user_input):

        try:
            parts = user_input.split()
            if len(parts) < 3:
                print("[INPUT] Need Speed, Accel and Distance(mm).")
                return None

            # 1. Speed (0~100% -> 0~3000 RPM)
            speed_pct = float(parts[0])
            if not (0 <= speed_pct <= 100):
                print("[INPUT] Speed Range: 0% ~ 100% (Max. 3000 RPM)")
                return None
            real_speed = int(3000 * (speed_pct / 100))

            # 2. Accel (0~100% -> 0~255)
            accel_pct = float(parts[1])
            if not (0 <= accel_pct <= 100):
                print("[INPUT] Accel Range: 0% ~ 100%")
                return None
            real_accel = int(255 * (accel_pct / 100))

            # 3. mm -> Turns -> Coordinate
            distance_mm = float(parts[2])
            if distance_mm < 0:
                print("[INPUT] Distance must be 0 or above.")
                return None
            turns = distance_mm / self.MM_PER_TURN
            coord = int(turns * self.ENCODER_PER_TURN)
            if coord > 8388607:
                max_mm = 8388607 / self.ENCODER_PER_TURN * self.MM_PER_TURN
                print(f"[INPUT] Distance out of range (max ~{max_mm:.1f}mm)")
                return None

            # 4. Byte Construction per manual 11.4.1
            # Byte2-3: speed (uint16_t big-endian)
            # Byte4: acc (uint8_t)
            # Byte5-7: absAxis (int24_t big-endian)
            speed_hi = (real_speed >> 8) & 0xFF
            speed_lo = real_speed & 0xFF

            coord_bytes = coord & 0xFFFFFF

            coord_hi = (coord_bytes >> 16) & 0xFF
            coord_mid = (coord_bytes >> 8) & 0xFF
            coord_lo = coord_bytes & 0xFF

            print(f"  -> Speed: {real_speed} RPM, Accel: {real_accel}, Dist: {distance_mm}mm ({turns:.2f} turns), Coord: {coord} (0x{coord_bytes:06X})")
            return [speed_hi, speed_lo, real_accel, coord_hi, coord_mid, coord_lo]

        except ValueError:
            print("[INPUT] Incorrect Number Format")
            return None

    def settings_menu(self, can_id, safe_input):
        while True:
            print(f'\n{"-"*45}\n[ Settings Menu ]\n{"-"*45}')
            for key, (name, _, _) in self.SETTINGS_MAP.items():
                print(f"  {key}. {name}")
            print(f"  0. Back to Main Menu")

            choice = safe_input(">> Choose Setting: ")
            if choice == "0":
                return

            if choice not in self.SETTINGS_MAP:
                print("[INPUT] Invalid choice.")
                continue

            name, cmd_code, options = self.SETTINGS_MAP[choice]
            print(f"\n  [{name}]")

            # Special: Set Zero Point (no data needed)
            if choice == "2":
                self.send_motor(can_id, 0x92)
                continue

            # Special: Homing Speed
            if choice == "10":
                rpm_raw = safe_input(">> Enter Homing Speed (RPM, default 90): ")
                if not rpm_raw:
                    rpm_raw = "90"
                try:
                    self.homing_speed = int(rpm_raw)
                    print(f"[OK] Homing speed set to {self.homing_speed} RPM")
                except ValueError:
                    print("[INPUT] Invalid number.")
                continue

            # Special: Run Homing
            if choice == "11":
                speed = getattr(self, 'homing_speed', 90)
                self.home_motor(can_id, home_speed=speed)
                continue

            # Options submenu
            if isinstance(options, dict):
                for k, (desc, _) in options.items():
                    print(f"    {k}. {desc}")
                sub = safe_input(">> Choose Option: ")
                if sub not in options:
                    print("[INPUT] Invalid option. Returning to Settings Menu.")
                    continue
                _, value = options[sub]
                if isinstance(value, list):
                    self.send_motor(can_id, cmd_code, *value)
                else:
                    self.send_motor(can_id, cmd_code, value)

            # Direct value input
            else:
                if cmd_code == 0x83:
                    val_raw = safe_input(">> Enter Current (mA, e.g. 1600): ")
                    try:
                        ma = int(val_raw)
                        if not (0 <= ma <= 5200):
                            print("[INPUT] Current Range: 0 ~ 5200 mA")
                            continue
                        self.send_motor(can_id, cmd_code, (ma >> 8) & 0xFF, ma & 0xFF)
                    except ValueError:
                        print("[INPUT] Invalid number. Returning to Settings Menu.")
                elif cmd_code == 0x84:
                    val_raw = safe_input(">> Enter Subdivisions (1-256, e.g. 16): ")
                    try:
                        subdiv = int(val_raw)
                        if not (1 <= subdiv <= 256):
                            print("[INPUT] Subdivisions Range: 1 ~ 256")
                            continue
                        self.send_motor(can_id, cmd_code, subdiv)
                    except ValueError:
                        print("[INPUT] Invalid number. Returning to Settings Menu.")

    def interactive_menu(self):
        
        def safe_input(prompt):
            user_val = input(prompt).strip()
            if user_val.lower() == 'exit':
                raise StopIteration     # Force error to occur, skip all if statements and escape
            return user_val
        
        try:
            while True:
                print(f'\n{"="*45}\n[ Motor Control Terminal ]\nType "exit" to exit.\n{"="*45}')

                # 1. ID
                try:
                    id_raw = safe_input(">> Enter CAN ID: ")
                    if not id_raw: continue
                    id_input = int(id_raw, 16)
                except ValueError:
                    print("[INPUT] Invalid ID.")
                    continue

                # 2. Command
                print("\n[ Command List ]")
                for key, (name, _, _) in self.COMMAND_MAP.items():
                    print(f"{key}. {name}")
                print("99. [MANUAL] Enter Command Code Manually")
                print("0. [SETUP] Initialize Motor Settings")

                choice = safe_input(">> Choose a Number: ")

                if choice == "99":
                    try:
                        cmd_raw = safe_input(">> Enter Command Code: ")
                        cmd_code = int(cmd_raw, 16)

                        d_raw = safe_input(">> Enter Data Bytes (hex) or Enter for none: ")
                        data_values = []
                        if d_raw:
                            for x in d_raw.split():
                                data_values.append(int(x, 16))

                        self.send_motor(id_input, cmd_code, *data_values)
                        # 5. Wait for Completion (If YY==1)
                        if cmd_code in (0x91, 0xF6, 0xF4, 0xF5,  0xFD, 0xFE):
                            # 91H: Hm Restoration, F6H: Speed Control
                            # F4H: Coord relativ running, F5H: Coord abs running
                            # FDH: Pulse relativ running, FEH: Pulse abs running
                            self.wait_response(can_id=id_input)
                        continue
                    except ValueError:
                        print("[INPUT] Invalid Hex Format.")
                        continue
                
                if choice == "0":
                    self.setup_motor(id_input)
                    continue

                if choice == "8":
                    self.settings_menu(id_input, safe_input)
                    continue

                if choice not in self.COMMAND_MAP:
                    print("[INPUT] Wrong Number. Re-Choose.")
                    continue

                cmd_name, cmd_code, cmd_format = self.COMMAND_MAP[choice]
                print(f"\nRunning: {cmd_name}")

                # 3. Data
                data_values = []
                if cmd_format:
                    print(f"--- Format ---\n{cmd_format}")
                    d_raw = safe_input(">> Enter Data: ")
                    if d_raw:
                        if choice == "1":
                            converted = self.parse_abs_coordinate_data(d_raw)
                            if converted is None:
                                continue
                            data_values = converted
                        elif choice == "2":
                            converted = self.parse_speed_control_data(d_raw)
                            if converted is None:
                                continue
                            data_values = converted
                        else:
                            try:
                                data_values = []
                                for x in d_raw.split():
                                    data_values.append(int(x, 16))

                            except ValueError:
                                print("[INPUT] Invalid Hex Format.")
                                continue

                # 4. Sending Packet
                self.send_motor(id_input, cmd_code, *data_values)

                # 5. Wait for Completion (If YY==1)
                if cmd_code in (0x91, 0xF6, 0xF4, 0xF5, 0xFD, 0xFE):
                    # 91H: Hm Restoration, F6H: Speed Control
                    # F4H: Coord relativ running, F5H: Coord abs running
                    # FDH: Pulse relativ running, FEH: Pulse abs running
                    result = self.wait_response(can_id=id_input)

                    # Auto-retry once if timed out after limit reset
                    if result is None and self._retry_after_limit:
                        self._retry_after_limit = False
                        print("[RETRY] Re-sending command after limit reset...")
                        self.dev.purge(1)  # Clear timed-out response before retry
                        self.send_motor(id_input, cmd_code, *data_values)
                        self.wait_response(can_id=id_input)

        except StopIteration:
            pass
        except Exception as e:
            print(f"\n[ERROR] {e}")

# --- Running Phase ---

if __name__ == "__main__":
    dev = None
    try:
        dev = ftdi.open(0)
        dev.setBitMode(0xFF, 0x40)
        dev.setTimeouts(100, 100)
        
        motor = MKSServo(dev)

        CAN_ID = 0x01
        if not motor.setup_motor(CAN_ID):
            print("[ERROR] Setup failed. Exiting.")
        else:
            motor.home_motor(CAN_ID)
            motor.interactive_menu()
    
    except Exception as e:
        print(f"[ERROR] Connection Failed: {e}")

    finally:
        if dev is not None:
            dev.close()
            print(f'\n{"="*45}\nDevice Closed. Program Terminated.\n{"="*45}')

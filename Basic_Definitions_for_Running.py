# ---Definition Phase---

import ftd2xx as ftdi
import time

class MKSServo:

    def __init__(self, dev):
        self.dev = dev

        self.COMMAND_MAP = {
            "1": ("Motor Enable/Disable", 0xF3,
                  "Format: 01 (Enable) or 00 (Disable)"),
            "2": ("Speed Control Mode", 0xF6,
                  "Format: [CW/CCW] [Speed(0-100%)] [Accel(0-100%)]\nExample: CW 50 10"),
            "3": ("Read Motor Operating Status", 0xF1,
                  None)
        }

    def send_can_message(self, can_id, data, dlc_value):
        # STX(1) + Type(1) +DLC(1) + Flags(1) + ID(4) + Data(8) + Checksum(1) + ETX(1) = Total 18byte

        packet = bytearray(18)
        packet[0] = 0x02            # STX
        packet[1] = 0x00            # Type: Message Packet
        packet[2] = dlc_value       # DLC (Range: 0~8)
        packet[3] = 0x00            # Flags: CAN Standard Data Frame
    
        # ID Setting
        packet[4:8] = can_id.to_bytes(4, 'little')

        # Data for Motor Driver
        for i in range(min(len(data), 8)):
            packet[8+i] = data[i]
    
        # Checksum Calculation
        packet[16] = sum(packet[1:16]) & 0xFF
        packet[17] = 0x03           # ETX

        self.dev.write(bytes(packet))
        print(f"[SENT] CAN ID {can_id}: {data.hex().upper()}")

    def create_motor_packet(self, can_id, cmd_code, data_values=None):
    
        # 0. If data is none, return empty list. Else, use input list
        if data_values is None:
            data_values = []
    
        # 1. DLC Calculation: Command + Data + CRC
        dlc = 1 + len(data_values) + 1
        if dlc > 8:
            print(f"[ERROR] DLC Exceed Maximum (8). Current: {dlc}")
            return None, None

        # 2. Internal CRC Calculation 
        total_sum = can_id + cmd_code + sum(data_values)
        internal_crc = total_sum & 0xFF

        # 3. packet Construction
        packet_content = [cmd_code] + data_values + [internal_crc]
        while len(packet_content) < 8:
            packet_content.append(0x00)

        # print(bytearray(packet_content).hex().upper())
        # print(dlc)
        return bytearray(packet_content), dlc

    def send_motor(self, can_id, cmd_code, *data_values):
    
        values_list = list(data_values)
        motor_data, dlc = self.create_motor_packet(can_id, cmd_code, values_list)
        if motor_data is None:
            return
        self.dev.purge(1)
        self.send_can_message(can_id, motor_data, dlc)

        cmd_name = next(
            (name for _, (name, code, _) in self.COMMAND_MAP.items() if code == cmd_code),
            f"Unknown(0x{cmd_code:02X})"
        )

        time.sleep(0.05)    # Motor Response Stand by
        response = self.dev.read(18)

        if len(response) == 18:
            status_byte = response[9]

            if status_byte == 0x00:
                print(f"[FAIL] Motor Rejected the Command: {cmd_name}")
            else:
                print(f"[STATUS] '{cmd_name}' Response Code: 0x{status_byte:02X}")
            
            # motor_reply = response[8:16]
            # print(f"   Raw Reply: {motor_reply.hex().upper()}")

        else:
            print(f"[TIMEOUT] No Response for {cmd_name}")

    def wait_response(self, timeout=30):
        print("[WAIT] Waiting for Motor Response...")
        start = time.time()
        while time.time() - start < timeout:
            response = self.dev.read(18)
            if len(response) == 18:
                cmd = response[8]
                status = response[9]
                print(f"[RECV] CMD: 0x{cmd:02X}, Status: 0x{status:02X}")
                return status
            time.sleep(0.1)
        print("[TIMEOUT] No Response Received.")
        return None


    def setup_motor(self, can_id):
        setup_commands = [
            ("Set SR_vFOC Mode", 0x82, [0x05]),
            ("Set Slave Response (XX=1, YY=0)", 0x8C, [0x01, 0x00]),
            ("Set CAN Rate 1M", 0x8A, [0x03]),
        ]

        all_ok = True
        for name, cmd, data in setup_commands:
            self.dev.purge(1)
            motor_data, dlc = self.create_motor_packet(can_id, cmd, data)
            self.send_can_message(can_id, motor_data, dlc)
            time.sleep(0.05)
            response = self.dev.read(18)

            if len(response) == 18 and response[9] == 0x01:
                print(f"[OK] {name}")
            else:
                print(f"[FAIL] {name}")
                all_ok = False
        
        if all_ok:
            print("[SETUP] All Settings Applied.")
        else:
            print("[SETUP] Some Settings Failed.")
        
        return all_ok

    def parse_speed_control_data(self, user_input):

        try:
            parts = user_input.split()
            if len(parts) < 3:
                print("[INPUT] Need Direction, Speed and Accel.")
                return None
            
            # 1. Direction
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
                            self.wait_response()
                        continue
                    except ValueError:
                        print("[INPUT] Invalid Hex Format.")
                        continue
                
                if choice == "0":
                    self.setup_motor(id_input)
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
                        if choice == "2":
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
                if cmd_code in (0x91, 0xF6, 0xF4, 0xF5,  0xFD, 0xFE):
                    # 91H: Hm Restoration, F6H: Speed Control
                    # F4H: Coord relativ running, F5H: Coord abs running
                    # FDH: Pulse relativ running, FEH: Pulse abs running
                    self.wait_response()

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
        motor.interactive_menu()
    
    except Exception as e:
        print(f"[ERROR] Connection Failed: {e}")

    finally:
        if dev is not None:
            dev.close()
            print(f'\n{"="*45}\nDevice Closed. Program Terminated.\n{"="*45}')

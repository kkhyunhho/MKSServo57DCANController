import ftd2xx as ftdi

# 1. 연결된 모든 FTDI 장치 찾기
devices = ftdi.listDevices()

if not devices:
    print("Can't find device. Check usbipd connection.")
else:
    print(f"Connected devices list: {devices}")

    # 2. 첫 번째 장치 열기
    dev = ftdi.open(0)
    print("Device Opening Success")

    # 3. 장치 정보 확인
    print(f"Serial number: {dev.getDeviceInfo()['serial']}")
    dev.close()

from synaps_lpms3 import Lpms3
from time import sleep

# Config
PORT = "/dev/ttyUSB0"
BAUDRATE = 921600

def main():
    imu = Lpms3()
    print("Available ports:")
    imu.listPort()
    print()
    print(f"Connecting to {PORT} at {BAUDRATE} baud...")
    imu.connect(port=PORT, baudrate=BAUDRATE)
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return

    print("Sensor model:", imu.getModel())
    print("Firmware:", imu.firmwareInfo())
    print("Serial number:", imu.getSerialNumber())
    print()
    imu.setFreq(100)
    print(f"Stream frequency: {imu.getFreq()} Hz")
    print()
    if not imu.streamMode():
        print("Failed to switch to stream mode")
        return 

    print(f"{'Time':<10} {'Acc (g)':<30} {'Gyr (dps)':<30} {'Mag (Gauss)':<30}")
    try:
        for i, frame in enumerate(imu.readStream()):
            acc = frame.get("acc_raw", (0, 0, 0))
            gyr = frame.get("gyr_raw", (0, 0, 0))
            mag = frame.get("mag_raw", (0, 0, 0))
            acc_str = f"({acc[0]:7.3f}, {acc[1]:7.3f}, {acc[2]:7.3f})"
            gyr_str = f"({gyr[0]:7.2f}, {gyr[1]:7.2f}, {gyr[2]:7.2f})"
            mag_str = f"({mag[0]:7.3f}, {mag[1]:7.3f}, {mag[2]:7.3f})"
            print(f"{frame.get('timestamp', 0):<10.3f} {acc_str:<30} {gyr_str:<30} {mag_str:<30}")
            if i >= 99:
                break
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    finally:
        # Close connection
        imu.close()
        print("Connection closed")


if __name__ == "__main__":
    main()


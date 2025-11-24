from synaps_lpms3 import Lpms3, head4MagAcc, head4Mag
from time import sleep

# Config
PORT = "/dev/ttyUSB0"
BAUDRATE = 921600
MAGNETIC_DECLINATION = -0.5
HEADING_OFFSET = 0.0  # Heading offset

def main():
    imu = Lpms3()
    print(f"Connecting to {PORT} at {BAUDRATE} baud...")
    imu.connect(port=PORT, baudrate=BAUDRATE)
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return

    imu.setFreq(100)
    print(f"Stream frequency: {imu.getFreq()} Hz")
    print()
    if not imu.streamMode():
        print("Failed to switch to stream mode")
        return

    print(f"{'Time':<10} {'Acc (g)':<30} {'Mag (Gauss)':<30} {'Heading':<20}")
    try:
        for i, frame in enumerate(imu.readStream()):
            acc = frame.get("acc_raw", (0, 0, 0))
            mag = frame.get("mag_raw", (0, 0, 0))
            heading_tilt = head4MagAcc(frame, declination_deg=MAGNETIC_DECLINATION)
            heading_simple = head4Mag(frame, declination_deg=MAGNETIC_DECLINATION)
            if heading_tilt is not None:
                heading_tilt = (heading_tilt + HEADING_OFFSET) % 360.0
                
            if heading_simple is not None:
                heading_simple = (heading_simple + HEADING_OFFSET) % 360.0

            acc_str = f"({acc[0]:7.3f}, {acc[1]:7.3f}, {acc[2]:7.3f})"
            mag_str = f"({mag[0]:7.3f}, {mag[1]:7.3f}, {mag[2]:7.3f})"
            if heading_tilt is not None:
                heading_str = f"Tilt-comp: {heading_tilt:6.2f}°"
                if heading_simple is not None:
                    heading_str += f" | Simple: {heading_simple:6.2f}°"
            else:
                heading_str = "N/A"
            
            print(f"{frame.get('timestamp', 0):<10.3f} {acc_str:<30} {mag_str:<30} {heading_str:<20}")
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


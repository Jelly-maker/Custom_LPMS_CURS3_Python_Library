from synaps_lpms3 import Lpms3, head4MagAcc, head4Mag
from time import sleep, time
from collections import deque
import math

PORT = "/dev/ttyUSB0"
BAUDRATE = 921600
MAGNETIC_DECLINATION = -0.5 
HEADING_OFFSET = -108.19

# Magnetometer Parameters
MAG_RANGE_LOW = 2.0  # Gauss (±2)
MAG_RANGE_HIGH = 8.0  # Gauss (±8)
MAG_SENSITIVITY_LOW = 3000  # LSB/G (untuk range ±2 Gauss)
MAG_SENSITIVITY_HIGH = 12000  # LSB/G (untuk range ±8 Gauss)
MAG_ZERO_OFFSET = 10.0  # mG (±10)
MAG_LINEARITY = 0.1  # %FS (FS=±2 gauss)

# Heading Filter Parameters
HEADING_FILTER_SIZE = 10  # Jumlah sample untuk moving average
HEADING_NOISE_THRESHOLD = 2.0  # Variasi maksimal yang dianggap normal (derajat)
USE_NOISE_FILTER = True  # Set False untuk menonaktifkan filtering

def main():
    imu = Lpms3()
    imu.listPort()
    imu.connect(port=PORT, baudrate=BAUDRATE)
    # Command mode
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return

    print("Sensor model:", imu.getModel())
    print("Firmware:", imu.firmwareInfo())
    imu.setFreq(100)
    print("Stream freq:", imu.getFreq())
    # Stream mode
    if not imu.streamMode():
        print("Failed to switch to stream mode")
        return

    print("Reading a few streaming packets...")
    for i, pkt in zip(range(5), imu.streamData()):
        print(i, "CMD:", hex(pkt["cmd"]), "len:", pkt["length"])

    imu.close()


def readFrame():
    imu = Lpms3()
    imu.connect(port=PORT, baudrate=BAUDRATE)
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return
    imu.setFreq(100)
    if not imu.streamMode():
        print("Failed to switch to stream mode")
        return
    
    for i, frame in zip(range(10), imu.readStream()):
        print(
            f"{i}  t={frame['timestamp']:.3f}s  "
            f"acc={frame['acc_raw']}  "
            f"gyr={frame['gyr_raw']}  "
            f"mag={frame['mag_calib']}"
            )
    imu.close()


def calibrateImu(duration_sec=60, calibrate_gyro=True, calibrate_mag=True, save_to_flash=True):
    imu = Lpms3()
    imu.connect(port=PORT, baudrate=BAUDRATE)
    
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return False
    
    print("=" * 50)
    print("IMU Calibration")
    print("=" * 50)
    print(f"Duration: {duration_sec} seconds")
    print(f"Gyroscope calibration: {'Yes' if calibrate_gyro else 'No'}")
    print(f"Magnetometer calibration: {'Yes' if calibrate_mag else 'No'}")
    print("=" * 50)
    if calibrate_mag:
        timeout = float(duration_sec)
        if not imu.setMagCalibrationTimeout(timeout):
            print("Warning: Failed to set magnetometer calibration timeout")

        else:
            print(f"Magnetometer timeout set to {timeout} seconds")

    if calibrate_gyro:
        print("\nStarting gyroscope calibration...")
        if imu.startGyrCalibration():
            print("Gyroscope calibration started successfully")

        else:
            print("Warning: Failed to start gyroscope calibration")
    
    if calibrate_mag:
        print("\nStarting magnetometer calibration...")
        print("IMPORTANT: Rotate the sensor in all directions (figure-8 pattern recommended)")
        if imu.startMagCalibration():
            print("Magnetometer calibration started successfully")

        else:
            print("Warning: Failed to start magnetometer calibration")
    
    print(f"\nCalibration in progress... ({duration_sec} seconds)")
    start_time = time()
    elapsed = 0
    
    while elapsed < duration_sec:
        remaining = duration_sec - elapsed
        progress = (elapsed / duration_sec) * 100
        if int(elapsed) % 5 == 0:
            print(f"Progress: {progress:.1f}% | Elapsed: {elapsed:.0f}s | Remaining: {remaining:.0f}s")
        
        sleep(1)
        elapsed = time() - start_time
    
    print(f"\nCalibration completed! (Total: {elapsed:.1f} seconds)")
    
    # Stop magnetometer kalibrasi
    if calibrate_mag:
        print("\nStopping magnetometer calibration...")
        if imu.stopMagCalibration():
            print("Magnetometer calibration stopped successfully")

        else:
            print("Warning: Failed to stop magnetometer calibration")
    
    # Simpan hasil kalibrasi ke flash
    if save_to_flash:
        print("\nSaving calibration to flash...")
        if imu.writeRegisters():
            print("Calibration saved to flash successfully")

        else:
            print("Warning: Failed to save calibration to flash")
    
    print("\n" + "=" * 50)
    print("Calibration process completed!")
    print("=" * 50)
    imu.close()
    return True


def headingOffset(duration_sec=10):
    imu = Lpms3()
    imu.connect(port=PORT, baudrate=BAUDRATE)
    
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return None
    
    imu.setFreq(100)
    if not imu.streamMode():
        print("Failed to switch to stream mode")
        return None
    
    print("=" * 50)
    print("Heading Offset Calibration")
    print("=" * 50)
    print("IMPORTANT: Point the sensor towards TRUE NORTH (use compass/GPS)")
    print(f"Measuring for {duration_sec} seconds...")
    print("=" * 50)
    
    headings = []
    start_time = time()
    
    try:
        for frame in imu.readStream():
            heading = head4MagAcc(frame, declination_deg=MAGNETIC_DECLINATION)
            if heading is not None:
                headings.append(heading)
                elapsed = time() - start_time
                print(f"Elapsed: {elapsed:.1f}s | Current heading: {heading:.2f}° | Samples: {len(headings)}")
            
            if time() - start_time >= duration_sec:
                break
    
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
        imu.close()
        return None
    
    if len(headings) == 0:
        print("Error: No heading data collected")
        imu.close()
        return None
    
    # Hitung rata-rata heading
    avg_heading = sum(headings) / len(headings)
    # Offset adalah negatif dari rata-rata (agar 0° = Utara)
    offset = -avg_heading
    offset = (offset + 360.0) % 360.0
    print("\n" + "=" * 50)
    print(f"Average heading: {avg_heading:.2f}°")
    print(f"Calculated offset: {offset:.2f}°")
    print("=" * 50)
    print(f"\nAdd this offset to your heading calculation:")
    print(f"HEADING_OFFSET = {offset:.2f}")
    print("=" * 50)
    imu.close()
    return offset


def normalizeAngleDiff(angle1, angle2):
    diff = angle1 - angle2
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff


def movingAverageFilter(values, new_value, max_size=HEADING_FILTER_SIZE):
    values.append(new_value)
    if len(values) > max_size:
        values.popleft()
    
    if len(values) < 2:
        return new_value
    
    angles = list(values)
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    avg = math.degrees(math.atan2(sin_sum / len(angles), cos_sum / len(angles)))
    return (avg + 360.0) % 360.0


def analyzeHeadingStability(headings):
    if len(headings) < 2:
        return None
    
    min_heading = min(headings)
    max_heading = max(headings)
    avg_heading = sum(headings) / len(headings)
    
    sin_sum = sum(math.sin(math.radians(h)) for h in headings)
    cos_sum = sum(math.cos(math.radians(h)) for h in headings)
    circular_avg = math.degrees(math.atan2(sin_sum / len(headings), cos_sum / len(headings)))
    circular_avg = (circular_avg + 360.0) % 360.0
    
    variance = sum((normalizeAngleDiff(h, circular_avg) ** 2) for h in headings) / len(headings)
    std_dev = math.sqrt(variance)
    
    range_deg = max_heading - min_heading
    if range_deg > 180:
        range_deg = 360 - range_deg
    
    return {
        "min": min_heading,
        "max": max_heading,
        "range": range_deg,
        "average": circular_avg,
        "std_dev": std_dev,
        "is_stable": std_dev < HEADING_NOISE_THRESHOLD
    }


def read(use_filter: bool = USE_NOISE_FILTER):
    imu = Lpms3()
    imu.connect(port=PORT, baudrate=BAUDRATE)
    if not imu.commandMode():
        print("Failed to switch to command mode")
        return

    imu.setFreq(100)
    if not imu.streamMode():
        print("Failed to switch to stream mode")
        return
    
    heading_buffer = deque(maxlen=100) if use_filter else None
    heading_filtered = deque(maxlen=HEADING_FILTER_SIZE) if use_filter else None
    sample_count = 0
    print("Reading heading data...")
    if use_filter:
        print("Noise filter: ENABLED")
        print("Variasi 1-2 derajat saat sensor diam adalah NORMAL untuk magnetometer")
        print("(disebabkan noise ±10 mG dan interferensi magnetik)\n")

    else:
        print("Noise filter: DISABLED")
        print("Menampilkan heading raw tanpa filtering\n")
    
    try:
        for frame in imu.readStream():
            acc = frame["acc_calib"] if "acc_calib" in frame else frame.get("acc_raw")
            gyr = frame["gyr_calib"] if "gyr_calib" in frame else frame.get("gyr_raw")
            mag = frame["mag_calib"] if "mag_calib" in frame else frame.get("mag_raw")
            heading_tilt = head4MagAcc(frame, declination_deg=MAGNETIC_DECLINATION)
            heading_simple = head4Mag(frame, declination_deg=MAGNETIC_DECLINATION)
            if heading_tilt is not None:
                heading_tilt = (heading_tilt + HEADING_OFFSET) % 360.0
                if use_filter:
                    heading_filtered_val = movingAverageFilter(heading_filtered, heading_tilt)
                    heading_buffer.append(heading_tilt)
                    sample_count += 1
                    if sample_count % 50 == 0 and len(heading_buffer) >= 20:
                        stats = analyzeHeadingStability(list(heading_buffer)[-20:])
                        if stats:
                            print(f"\n[Stats last 20 samples] "
                                  f"Avg: {stats['average']:.2f}° | "
                                  f"Range: {stats['range']:.2f}° | "
                                  f"StdDev: {stats['std_dev']:.2f}° | "
                                  f"Stable: {'Yes' if stats['is_stable'] else 'No'}")
                    
                    print(f"Heading (raw): {heading_tilt:.2f}° | "
                          f"(filtered): {heading_filtered_val:.2f}° | "
                          f"(mag-only): {heading_simple:.2f}°" if heading_simple else 
                          f"Heading (raw): {heading_tilt:.2f}° | (filtered): {heading_filtered_val:.2f}°")

                else:
                    print(f"Heading (tilt-comp): {heading_tilt:.2f}° | "
                          f"Heading (mag-only): {heading_simple:.2f}°" if heading_simple else 
                          f"Heading: {heading_tilt:.2f}°")

    except KeyboardInterrupt:
        print("\nStop.")
        if use_filter and heading_buffer and len(heading_buffer) >= 10:
            stats = analyzeHeadingStability(list(heading_buffer))
            print(f"\n=== Final Statistics ===")
            print(f"Total samples: {len(heading_buffer)}")
            print(f"Min heading: {stats['min']:.2f}°")
            print(f"Max heading: {stats['max']:.2f}°")
            print(f"Range: {stats['range']:.2f}°")
            print(f"Average: {stats['average']:.2f}°")
            print(f"Standard deviation: {stats['std_dev']:.2f}°")
            print(f"Stability: {'GOOD' if stats['is_stable'] else 'NEEDS IMPROVEMENT'}")
            if stats['range'] <= 2.0:
                print("✓ Variasi ≤ 2° adalah NORMAL untuk magnetometer")

            else:
                print("⚠ Variasi > 2° - pertimbangkan kalibrasi ulang atau filtering")

    finally:
        imu.close()
        print("Port closed.")


if __name__ == "__main__":
    # calibrateImu(duration_sec=60, calibrate_gyro=False, calibrate_mag=True, save_to_flash=True)
    # offset = headingOffset(duration_sec=10)
    # print(f"Set HEADING_OFFSET = {offset:.2f} in your code")
    # Gunakan read(use_filter=True) untuk enable filter, atau read(use_filter=False) untuk disable
    read(use_filter=USE_NOISE_FILTER)
    # readFrame()
    # main()
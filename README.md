# LPMS3 IMU Python Testing

Python project for testing and implementing LPMS3 IMU commands based on the [LPMS3 Command List](https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List).

## Description

This project provides a Python library for communicating with LPMS3 IMU sensors via serial communication. The library implements various commands available on LPMS3 according to the official LP-Research documentation.

## Features

- ✅ Complete implementation of LPMS3 commands based on official documentation
- ✅ Command Mode and Stream Mode
- ✅ Sensor configuration (accelerometer, gyroscope, magnetometer)
- ✅ Sensor calibration (gyroscope and magnetometer)
- ✅ Real-time data reading with threading
- ✅ Heading calculation with tilt compensation
- ✅ Data logging to CSV
- ✅ Error handling and retry mechanism

## Project Structure

```
custom/
├── synaps_lpms3.py           # Main library for LPMS3 communication
├── example_basic_read.py     # Example: Reading basic raw IMU data
├── example_read_heading.py  # Example: Reading heading from raw data
├── main.py                   # Advanced examples (calibration, etc.)
├── imu_threaded.py          # Threaded implementation for real-time reading
└── README.md                 # Project documentation
```

## Requirements

- Python 3.6+
- pyserial

## Installation

1. Clone or download this project
2. Install dependencies:

```bash
pip install pyserial
```

## Configuration

Edit configuration in the file you're using:

```python
PORT = "/dev/ttyUSB0"        # Serial port (Linux) or "COM3" (Windows)
BAUDRATE = 921600            # Communication baudrate
MAGNETIC_DECLINATION = -0.5  # Magnetic declination (degrees)
HEADING_OFFSET = -123.19     # Heading offset (degrees)
```

## Usage

### 1. Basic Data Reading (example_basic_read.py)

Simple example for reading basic raw IMU data (accelerometer, gyroscope, magnetometer):

```bash
python example_basic_read.py
```

This example shows:
- How to connect to the IMU
- How to read raw sensor data
- Display accelerometer, gyroscope, and magnetometer values

### 2. Reading Heading from Raw Data (example_read_heading.py)

Example for calculating heading from raw magnetometer and accelerometer data:

```bash
python example_read_heading.py
```

This example shows:
- How to calculate heading with tilt compensation
- How to calculate simple heading (magnetometer only)
- How to apply magnetic declination and heading offset

### 3. Advanced Usage (main.py)

Advanced examples including calibration and filtering:

```python
from synaps_lpms3 import Lpms3

imu = Lpms3()
imu.connect(port="/dev/ttyUSB0", baudrate=921600)

# Switch to command mode
imu.commandMode()

# Get device info
print("Model:", imu.getModel())
print("Firmware:", imu.firmwareInfo())

# Set frequency
imu.setFreq(100)

# Switch to stream mode
imu.streamMode()

# Read data
for frame in imu.readStream():
    print(frame)
    break

imu.close()
```

### 4. Threaded Real-time Reading (imu_threaded.py)

```python
from imu_threaded import ImuThreaded

imu_handler = ImuThreaded(port="/dev/ttyUSB0", use_filter=True)

try:
    if imu_handler.start():
        # Data reading runs in background thread
        # Automatic data logging to CSV for 1 second
        
        # Send command (example)
        imu_handler.send_command("set_freq", {"freq": 100})
        imu_handler.send_command("set_acc_range", {"range": 16})
        
        # Get latest frame
        frame = imu_handler.get_latest_frame()
        if frame:
            print(frame)
        
        # Keep running...
        import time
        time.sleep(10)
        
finally:
    imu_handler.stop()
```

### 3. Sensor Calibration

```python
from main import calibrateImu

# Calibrate gyroscope and magnetometer for 60 seconds
calibrateImu(
    duration_sec=60,
    calibrate_gyro=True,
    calibrate_mag=True,
    save_to_flash=True
)
```

### 4. Heading Offset Calibration

```python
from main import headingOffset

# Measure heading offset by pointing sensor to north
offset = headingOffset(duration_sec=10)
print(f"HEADING_OFFSET = {offset:.2f}")
```

## Available Commands

This library implements the following commands from the LPMS3 Command List:

### Mode Switching
- `commandMode()` - Switch to command mode
- `streamMode()` - Switch to stream mode

### Device Information
- `getModel()` - Get sensor model
- `firmwareInfo()` - Get firmware version
- `getSerialNumber()` - Get serial number
- `getFilterVersion()` - Get filter version
- `getSensorStatus()` - Get sensor status

### Configuration
- `setFreq(freq_hz)` / `getFreq()` - Set/get stream frequency
- `setImuId(id)` / `getImuId()` - Set/get IMU ID
- `setDegRadOutput(mode)` / `getDegRadOutput()` - Set/get output format

### Accelerometer
- `setAccRange(range)` / `getAccRange()` - Set/get accelerometer range
  - Range: 2, 4, 8, 16 (g)

### Gyroscope
- `setGyrRange(range)` / `getGyrRange()` - Set/get gyroscope range
  - Range: 125, 250, 500, 1000, 2000 (dps)
- `startGyrCalibration()` - Start gyroscope calibration
- `setEnableGyrAutocalibration(enable)` / `getEnableGyrAutocalibration()` - Auto-calibration

### Magnetometer
- `setMagRange(range)` / `getMagRange()` - Set/get magnetometer range
  - Range: 2, 4, 8, 12, 16 (Gauss)
- `startMagCalibration()` - Start magnetometer calibration
- `stopMagCalibration()` - Stop magnetometer calibration
- `setMagCalibrationTimeout(timeout)` / `getMagCalibrationTimeout()` - Calibration timeout

### Filter
- `setFilterMode(mode)` / `getFilterMode()` - Set/get filter mode

### Register Operations
- `writeRegisters()` - Save settings to flash
- `restoreFactoryValue()` - Restore factory settings

### CAN Settings (for CAN variant)
- `setCanStartId(id)` / `getCanStartId()` - CAN start ID
- `setCanBaudrate(baudrate)` / `getCanBaudrate()` - CAN baudrate
- `setCanDataPrecision(precision)` / `getCanDataPrecision()` - CAN data precision
- `setCanMode(mode)` - CAN mode

### Data Reading
- `readOnce()` - Read single IMU data frame
- `readStream()` - Generator for reading continuous stream data

## Data Frame Structure

The returned data frame contains:

```python
{
    "timestamp": float,        # Timestamp (seconds)
    "acc_raw": (x, y, z),      # Accelerometer raw (g)
    "acc_calib": (x, y, z),    # Accelerometer calibrated (g)
    "gyr_raw": (x, y, z),      # Gyroscope raw (dps)
    "gyr_bias": (x, y, z),     # Gyroscope bias
    "gyr_calib": (x, y, z),    # Gyroscope calibrated (dps)
    "mag_raw": (x, y, z),      # Magnetometer raw (Gauss)
    "mag_calib": (x, y, z),    # Magnetometer calibrated (Gauss)
    "ang_vel": (x, y, z),      # Angular velocity
    "quat": (w, x, y, z),      # Quaternion
    "euler": (x, y, z),        # Euler angles (degrees)
    "lin_acc": (x, y, z),      # Linear acceleration
    "pressure": float,         # Pressure (hPa)
    "altitude": float,         # Altitude (m)
    "temperature": float       # Temperature (°C)
}
```

## Heading Calculation

The library provides functions for calculating heading:

```python
from synaps_lpms3 import head4MagAcc, head4Mag

# Heading with tilt compensation (using acc + mag)
heading_tilt = head4MagAcc(frame, declination_deg=-0.5)

# Simple heading (using mag only)
heading_simple = head4Mag(frame, declination_deg=-0.5)
```

## Threaded Implementation

`imu_threaded.py` provides a threaded implementation with the following features:

- **Thread 1**: Continuous data reading from sensor
- **Thread 2**: Command handler for sending commands without blocking
- **Thread 3**: Data logging to CSV file

### Command Queue

Use `send_command()` to send commands in a thread-safe manner:

```python
imu_handler.send_command("set_freq", {"freq": 100})
imu_handler.send_command("set_acc_range", {"range": 16})
imu_handler.send_command("start_gyr_calibration")
imu_handler.send_command("start_mag_calibration")
imu_handler.send_command("stop_mag_calibration")
imu_handler.send_command("write_registers")
```

## Troubleshooting

### Port not found
- Ensure sensor is connected and USB serial driver is installed
- Check port with: `imu.listPort()`
- Linux: usually `/dev/ttyUSB0` or `/dev/ttyACM0`
- Windows: usually `COM3`, `COM4`, etc.

### Timeout errors
- Check cable connection
- Check baudrate (default: 921600)
- Try flushing buffer: `imu.flushBuffers()`

### Command mode failed
- Library already has retry mechanism (3x)
- Ensure sensor is not in active stream mode
- Try flushing buffer before switching mode

### Inaccurate data
- Perform sensor calibration first
- Ensure sensor is stable during calibration
- For magnetometer: avoid magnetic interference

## References

- [LPMS3 Command List](https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List)
- [LP-Research Knowledge Base](https://lp-research.atlassian.net/wiki/spaces/LKB)

## License

This project is for testing and development purposes. Please modify as needed.

## Notes

- This project is created for testing LPMS3 IMU commands
- Ensure sensor firmware is updated to the latest version
- Some commands may not be available depending on sensor model
- CAN commands are only for variants that support CAN bus

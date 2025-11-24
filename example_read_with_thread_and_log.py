from synaps_lpms3 import Lpms3, head4MagAcc, head4Mag
from threading import Thread, Lock, Event
from queue import Queue, Empty
from time import sleep, time
import csv
from datetime import datetime
from typing import Optional, Dict, Any
from collections import deque
import math

PORT = "/dev/ttyUSB0"
BAUDRATE = 921600
MAGNETIC_DECLINATION = -0.5
HEADING_OFFSET = -123.19

# Heading Filter Parameters
HEADING_FILTER_SIZE = 10
USE_NOISE_FILTER = False


def normalizeAngleDiff(angle1, angle2):
    diff = angle1 - angle2
    if diff > 180: diff -= 360
    elif diff < -180: diff += 360
    return diff


def movingAverageFilter(values, new_value, max_size=HEADING_FILTER_SIZE):
    values.append(new_value)
    if len(values) > max_size: values.popleft()
    if len(values) < 2: return new_value
    angles = list(values)
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    avg = math.degrees(math.atan2(sin_sum / len(angles), cos_sum / len(angles)))
    return (avg + 360.0) % 360.0

 
class ImuThreaded:
    def __init__(self, port: str = PORT, baudrate: int = BAUDRATE, use_filter: bool = USE_NOISE_FILTER):
        self.port = port
        self.baudrate = baudrate
        self.use_filter = use_filter
        self.imu: Optional[Lpms3] = None
        self.data_lock = Lock()
        self.latest_frame: Optional[Dict[str, Any]] = None
        self.command_queue = Queue()
        self.stop_event = Event()
        self.logging_active = False
        self.logging_lock = Lock()
        self.csv_file = None
        self.csv_writer = None
        self.thread_read: Optional[Thread] = None
        self.thread_command: Optional[Thread] = None
        self.setLogging: Optional[Thread] = None
        self.serial_lock = Lock()
        self.read_paused = Event()
        self.read_paused.set()
        self.heading_filtered = deque(maxlen=HEADING_FILTER_SIZE) if use_filter else None
    
    
    def flushBuffer(self):
        if self.imu and self.imu.ser and self.imu.ser.is_open:
            try:
                self.imu.ser.reset_input_buffer()
                self.imu.ser.reset_output_buffer()
            except:
                pass
    
    
    def connect(self) -> bool:
        try:
            self.imu = Lpms3()
            self.imu.connect(port=self.port, baudrate=self.baudrate)
            sleep(0.1)
            self.flushBuffer()
            sleep(0.1)
            max_retries = 3
            for attempt in range(max_retries):
                self.flushBuffer()
                sleep(0.05)
                with self.serial_lock:
                    if self.imu.commandMode():
                        print("Switched to command mode")
                        break

                    else:
                        if attempt < max_retries - 1:
                            print(f"Command mode attempt {attempt + 1} failed, retrying...")
                            sleep(0.2)

                        else:
                            print("Failed to switch to command mode after retries")
                            return False
            
            sleep(0.1)
            with self.serial_lock:
                self.imu.setFreq(100)
            
            sleep(0.1)
            with self.serial_lock:
                if not self.imu.streamMode():
                    print("Failed to switch to stream mode")
                    return False
            
            sleep(0.1)
            print("IMU connected successfully")
            return True
        
        except Exception as e:
            print(f"Error connecting to IMU: {e}")
            return False
    
    
    def readData(self):
        print("Thread 1: Reading data started")
        consecutive_errors = 0
        max_consecutive_errors = 10
        while not self.stop_event.is_set():
            try:
                for frame in self.imu.readStream():
                    if self.stop_event.is_set(): break
                    if not self.read_paused.is_set(): self.read_paused.wait()
                    with self.data_lock: self.latest_frame = frame
                    consecutive_errors = 0
                    sleep(0.001)

                break

            except (ValueError, TimeoutError) as e:
                consecutive_errors += 1
                error_msg = str(e)
                if "Invalid termination bytes" in error_msg or "LRC mismatch" in error_msg:
                    if consecutive_errors < max_consecutive_errors:
                        print(f"Warning: Packet error ({consecutive_errors}/{max_consecutive_errors}): {error_msg}")
                        sleep(0.01)
                        self.flushBuffer()

                    else:
                        print(f"Error: Too many consecutive packet errors. Stopping read thread.")
                        break

                else:
                    print(f"Error in read thread: {e}")
                    break

            except Exception as e:
                print(f"Error in read thread: {e}")
                break
        
        print("Thread 1: Reading data stopped")
    
    
    def sendCommand(self):
        print("Thread 2: Command handler started")
        try:
            while not self.stop_event.is_set():
                try:
                    command = self.command_queue.get(timeout=0.1)
                    self.execCommand(command)

                except Empty: continue
                except Exception as e: print(f"Error executing command: {e}")

        finally:
            print("Thread 2: Command handler stopped")
    
    
    def execCommand(self, command: Dict[str, Any]):
        cmd_type = command.get("type")
        params = command.get("params", {})
        if not self.imu:
            print("IMU not connected")
            return
        
        try:
            if cmd_type == "command_mode":
                self.read_paused.clear()
                sleep(0.1)
                self.flushBuffer()
                sleep(0.05)
                max_retries = 3
                result = False
                for attempt in range(max_retries):
                    with self.serial_lock:
                        self.flushBuffer()
                        result = self.imu.commandMode()
                        if result: break
                        if attempt < max_retries - 1:
                            sleep(0.2)
                
                self.read_paused.set()
                print(f"Command mode: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "stream_mode":
                with self.serial_lock:
                    self.flushBuffer()
                    sleep(0.05)
                    result = self.imu.streamMode()

                print(f"Stream mode: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "set_freq":
                freq = params.get("freq", 100)
                with self.serial_lock:
                    result = self.imu.setFreq(freq)

                print(f"Set frequency {freq} Hz: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "set_acc_range":
                range_val = params.get("range", 16)
                with self.serial_lock:
                    result = self.imu.setAccRange(range_val)

                print(f"Set acc range {range_val}: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "set_gyr_range":
                range_val = params.get("range", 2000)
                with self.serial_lock:
                    result = self.imu.setGyrRange(range_val)

                print(f"Set gyr range {range_val}: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "set_mag_range":
                range_val = params.get("range", 2)
                with self.serial_lock:
                    result = self.imu.setMagRange(range_val)

                print(f"Set mag range {range_val}: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "start_gyr_calibration":
                with self.serial_lock:
                    result = self.imu.startGyrCalibration()

                print(f"Start gyro calibration: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "start_mag_calibration":
                with self.serial_lock:
                    result = self.imu.startMagCalibration()

                print(f"Start mag calibration: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "stop_mag_calibration":
                with self.serial_lock:
                    result = self.imu.stopMagCalibration()

                print(f"Stop mag calibration: {'OK' if result else 'Failed'}")
            
            elif cmd_type == "write_registers":
                with self.serial_lock:
                    result = self.imu.writeRegisters()

                print(f"Write registers: {'OK' if result else 'Failed'}")
            
            else: print(f"Unknown command type: {cmd_type}")

        except Exception as e:
            print(f"Error executing command {cmd_type}: {e}")
            self.read_paused.set()
    
    def setLogging(self):
        print("Thread 3: Logging started (1 second)")
        if self.use_filter: print("  Noise filter: ENABLED")
        else: print("  Noise filter: DISABLED")
        start_time = time()
        duration = 1.0
        filename = f"imu_log_{datetime.now().strftime('%Y%m%d_%H')}.csv"
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                header = [
                    "timestamp", "acc_x", "acc_y", "acc_z",
                    "gyr_x", "gyr_y", "gyr_z",
                    "mag_x", "mag_y", "mag_z",
                    "heading_tilt", "heading_tilt_filtered", "heading_simple"
                ]
                writer.writerow(header)
                while (time() - start_time) < duration:
                    if self.stop_event.is_set(): break
                    with self.data_lock: frame = self.latest_frame
                    if frame:
                        acc = frame.get("acc_calib") or frame.get("acc_raw", (0, 0, 0))
                        gyr = frame.get("gyr_calib") or frame.get("gyr_raw", (0, 0, 0))
                        mag = frame.get("mag_calib") or frame.get("mag_raw", (0, 0, 0))
                        heading_tilt = head4MagAcc(frame, declination_deg=MAGNETIC_DECLINATION)
                        heading_simple = head4Mag(frame, declination_deg=MAGNETIC_DECLINATION)
                        if heading_tilt is not None:
                            heading_tilt = (heading_tilt + HEADING_OFFSET) % 360.0
                            if self.use_filter and self.heading_filtered is not None:
                                heading_tilt_filtered = movingAverageFilter(self.heading_filtered, heading_tilt)
                            
                            else: heading_tilt_filtered = heading_tilt

                        else: heading_tilt_filtered = ""
                        if heading_simple is not None:
                            heading_simple = (heading_simple + HEADING_OFFSET) % 360.0
                        
                        row = [
                            frame.get("timestamp", 0),
                            acc[0], acc[1], acc[2],
                            gyr[0], gyr[1], gyr[2],
                            mag[0], mag[1], mag[2],
                            heading_tilt if heading_tilt is not None else "",
                            heading_tilt_filtered if heading_tilt_filtered != "" else "",
                            heading_simple if heading_simple is not None else ""
                        ]
                        writer.writerow(row)
                        f.flush()
                    
                    sleep(0.01)
            
            elapsed = time() - start_time
            print(f"Thread 3: Logging completed ({elapsed:.2f}s) - Saved to {filename}")

        except Exception as e:
            print(f"Error in logging thread: {e}")

    
    def sendExecCommand(self, cmd_type: str, params: Optional[Dict[str, Any]] = None):
        if params is None:
            params = {}

        self.command_queue.put({"type": cmd_type, "params": params})
    
    
    def start(self):
        if not self.connect():
            return False
        
        self.stop_event.clear()
        self.thread_read = Thread(target=self.readData, daemon=True)
        self.thread_command = Thread(target=self.sendCommand, daemon=True)
        self.setLogging = Thread(target=self.setLogging, daemon=True)
        self.thread_read.start()
        sleep(0.5)
        self.thread_command.start()
        sleep(0.1)
        self.setLogging.start()
        print("All threads started")
        return True
    
    
    def stop(self):
        print("Stopping threads...")
        self.stop_event.set()
        self.read_paused.set()
        if self.thread_read and self.thread_read.is_alive():
            self.thread_read.join(timeout=2)

        if self.thread_command and self.thread_command.is_alive():
            self.thread_command.join(timeout=2)

        if self.setLogging and self.setLogging.is_alive():
            self.setLogging.join(timeout=2)
        
        if self.imu:
            try:
                with self.serial_lock:
                    self.imu.close()

            except:
                pass
        
        print("All threads stopped")
    
    
    def getLastestFrame(self) -> Optional[Dict[str, Any]]:
        with self.data_lock:
            return self.latest_frame.copy() if self.latest_frame else None


def main():
    # use_filter=True for enable filter
    imu_handler = ImuThreaded(use_filter=USE_NOISE_FILTER)
    try:
        if not imu_handler.start(): return
        sleep(2)
        print(f"Noise filter: {'ENABLED' if imu_handler.use_filter else 'DISABLED'}\n")
        while True:
            sleep(1)
            frame = imu_handler.getLastestFrame()
            if frame:
                acc = frame.get("acc_calib") or frame.get("acc_raw")
                heading_tilt = head4MagAcc(frame, declination_deg=MAGNETIC_DECLINATION)
                if heading_tilt is not None:
                    heading_tilt = (heading_tilt + HEADING_OFFSET) % 360.0
                    if imu_handler.use_filter and imu_handler.heading_filtered is not None:
                        heading_filtered_val = movingAverageFilter(imu_handler.heading_filtered, heading_tilt)
                        print(f"Acc: {acc} | Heading (raw): {heading_tilt:.2f}° | (filtered): {heading_filtered_val:.2f}°")

                    else:
                        print(f"Acc: {acc} | Heading: {heading_tilt:.2f}°")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        imu_handler.stop()


if __name__ == "__main__":
    main()


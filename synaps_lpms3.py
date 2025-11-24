import serial
import struct
import math
from enum import IntEnum
from typing import Optional, Tuple, Dict, Any
from time import sleep


class Lpms3Command(IntEnum):
    # Acknowledge
    REPLY_ACK = 0x00
    REPLY_NACK = 0x01
    # Register / reset
    WRITE_REGISTERS = 0x04
    RESTORE_FACTORY_VALUE = 0x05
    # Mode switching
    GOTO_COMMAND_MODE = 0x06
    GOTO_STREAM_MODE = 0x07
    # Status & data
    GET_SENSOR_STATUS = 0x08
    GET_IMU_DATA = 0x09
    # Device info
    GET_SENSOR_MODEL = 0x14
    GET_FIRMWARE_INFO = 0x15
    GET_SERIAL_NUMBER = 0x16
    GET_FILTER_VERSION = 0x17
    # Data transmission
    SET_IMU_TRANSMIT_DATA = 0x1E
    GET_IMU_TRANSMIT_DATA = 0x1F
    # IMU ID
    SET_IMU_ID = 0x20
    GET_IMU_ID = 0x21
    # Stream frequency
    SET_STREAM_FREQ = 0x22
    GET_STREAM_FREQ = 0x23
    # Deg/rad output
    SET_DEGRAD_OUTPUT = 0x24
    GET_DEGRAD_OUTPUT = 0x25
    # Orientation offset
    SET_ORIENTATION_OFFSET = 0x26
    RESET_ORIENTATION_OFFSET = 0x27
    # Acc settings
    SET_ACC_RANGE = 0x32
    GET_ACC_RANGE = 0x33
    # Gyro settings
    SET_GYR_RANGE = 0x3C
    GET_GYR_RANGE = 0x3D
    START_GYR_CALIBRATION = 0x3E
    SET_ENABLE_GYR_AUTOCALIBRATION = 0x40
    GET_ENABLE_GYR_AUTOCALIBRATION = 0x41
    # Mag settings
    SET_MAG_RANGE = 0x46
    GET_MAG_RANGE = 0x47
    START_MAG_CALIBRATION = 0x54
    STOP_MAG_CALIBRATION = 0x55
    SET_MAG_CALIBRATION_TIMEOUT = 0x56
    GET_MAG_CALIBRATION_TIMEOUT = 0x57
    # Filter mode
    SET_FILTER_MODE = 0x5A
    GET_FILTER_MODE = 0x5B
    # CAN (only if you use CAN variant)
    SET_CAN_START_ID = 0x6E
    GET_CAN_START_ID = 0x6F
    SET_CAN_BAUDRATE = 0x70
    GET_CAN_BAUDRATE = 0x71
    SET_CAN_DATA_PRECISION = 0x72
    GET_CAN_DATA_PRECISION = 0x73
    SET_CAN_MODE = 0x74


class Lpms3:
    PACKET_START = 0x3A
    TERM1 = 0x0D
    TERM2 = 0x0A


    def __init__(self, port: Optional[str] = None, baudrate: int = 921600, sensor_id: int = 1, timeout: float = 0.1):
        self.port_name = port
        self.baudrate = baudrate
        self.sensor_id = sensor_id
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None


    @staticmethod
    def listPort() -> None:
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                print(f"{p.device} - {p.description}")
                
        except Exception as e:
            print(f"Cannot list ports: {e}")
            

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None):
        if port is not None: self.port_name = port
        if baudrate is not None: self.baudrate = baudrate
        if self.port_name is None:
            raise ValueError("No port specified.")

        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )


    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        

    def checkOpen(self):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port not open.")


    @staticmethod
    def u162Byte(val: int) -> Tuple[int, int]:
        return val & 0xFF, (val >> 8) & 0xFF


    @staticmethod
    def lrcCalc(sensor_id: int, cmd: int, data: bytes) -> Tuple[int, int]:
        sid_lo, sid_hi = sensor_id & 0xFF, (sensor_id >> 8) & 0xFF
        cmd_lo, cmd_hi = cmd & 0xFF, (cmd >> 8) & 0xFF
        length = len(data)
        len_lo, len_hi = length & 0xFF, (length >> 8) & 0xFF
        total = sid_lo + sid_hi + cmd_lo + cmd_hi + len_lo + len_hi
        total += sum(data)
        total &= 0xFFFF
        return total & 0xFF, (total >> 8) & 0xFF


    def buildPackage(self, cmd: Lpms3Command, payload: bytes = b"") -> bytes:
        sid = self.sensor_id
        sid_lo, sid_hi = self.u162Byte(sid)
        cmd_lo, cmd_hi = self.u162Byte(int(cmd))
        length = len(payload)
        len_lo, len_hi = self.u162Byte(length)
        lrc_lo, lrc_hi = self.lrcCalc(sid, int(cmd), payload)
        packet = bytes(
            [
                self.PACKET_START,
                sid_lo,
                sid_hi,
                cmd_lo,
                cmd_hi,
                len_lo,
                len_hi,
            ]
        ) + payload + bytes([lrc_lo, lrc_hi, self.TERM1, self.TERM2])
        return packet


    def readData(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                raise TimeoutError("Timeout while reading from sensor.")
            
            buf += chunk
            
        return buf


    def flushBuffers(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
            except:
                pass
    
    
    def clearStreamData(self, timeout: float = 0.05, max_iterations: int = 10):
        if self.ser and self.ser.is_open:
            try:
                old_timeout = self.ser.timeout
                self.ser.timeout = timeout
                iterations = 0
                while iterations < max_iterations:
                    data = self.ser.read(1024)
                    if not data: break
                    iterations += 1

                self.ser.timeout = old_timeout

            except:
                pass


    def readPacket(self) -> Dict[str, Any]:
        self.checkOpen()
        max_attempts = 100
        attempts = 0
        # Find start byte 0x3A
        while attempts < max_attempts:
            b = self.ser.read(1)
            if not b: raise TimeoutError("Timeout waiting.")
            if b[0] == self.PACKET_START: break
            attempts += 1
        
        if attempts >= max_attempts:
            raise ValueError("Failed to find packet start byte")

        header = self.readData(6)
        sid_lo, sid_hi, cmd_lo, cmd_hi, len_lo, len_hi = header
        sensor_id = sid_lo | (sid_hi << 8)
        cmd = cmd_lo | (cmd_hi << 8)
        length = len_lo | (len_hi << 8)
        
        if length > 1024:
            raise ValueError(f"Invalid packet length: {length}")
        
        data = self.readData(length)
        lrc_lo, lrc_hi = self.readData(2)
        term1, term2 = self.readData(2)
        if term1 != self.TERM1 or term2 != self.TERM2:
            raise ValueError(f"Invalid termination bytes: {term1:02X} {term2:02X} (expected {self.TERM1:02X} {self.TERM2:02X})")

        # Verify LRC
        calc_lo, calc_hi = self.lrcCalc(sensor_id, cmd, data)
        if (lrc_lo, lrc_hi) != (calc_lo, calc_hi):
            raise ValueError(f"LRC mismatch: expected ({calc_lo:02X}, {calc_hi:02X}), got ({lrc_lo:02X}, {lrc_hi:02X})")

        return {
            "sensor_id": sensor_id,
            "cmd": cmd,
            "length": length,
            "data": data,
        }


    def sendData(self, cmd: Lpms3Command, payload: bytes = b"", expect_reply: bool = True) -> Optional[Dict[str, Any]]:
        self.checkOpen()
        pkt = self.buildPackage(cmd, payload)
        self.ser.write(pkt)
        self.ser.flush()
        if not expect_reply: return None
        return self.readPacket()

    
    def sendDataWithFlush(self, cmd: Lpms3Command, payload: bytes = b"", expect_reply: bool = True, clear_stream: bool = False) -> Optional[Dict[str, Any]]:
        self.checkOpen()
        if clear_stream:
            self.clearStreamData(timeout=0.05, max_iterations=5)

        self.flushBuffers()
        sleep(0.01)
        pkt = self.buildPackage(cmd, payload)
        self.ser.write(pkt)
        self.ser.flush()
        if not expect_reply: return None
        sleep(0.01)
        return self.readPacket()


    def commandMode(self, max_retries: int = 3) -> bool:
        for attempt in range(max_retries):
            try:
                if attempt > 0:
                    self.flushBuffers()
                    sleep(0.1)
                
                if attempt == 0:
                    self.clearStreamData(timeout=0.05, max_iterations=5)
                
                self.flushBuffers()
                sleep(0.01)                
                pkt = self.buildPackage(Lpms3Command.GOTO_COMMAND_MODE)
                self.ser.write(pkt)
                self.ser.flush()
                sleep(0.02)
                reply = self.readPacket()
                if reply and self.ackGet(reply): return True
                if attempt < max_retries - 1:
                    self.flushBuffers()
                    sleep(0.2)

            except (TimeoutError, ValueError) as e:
                if attempt < max_retries - 1:
                    self.flushBuffers()
                    sleep(0.2)

                else: return False
        
        return False


    def streamMode(self) -> bool:
        self.flushBuffers()
        sleep(0.01)
        reply = self.sendData(Lpms3Command.GOTO_STREAM_MODE)
        sleep(0.01)
        return self.ackGet(reply)


    def ackGet(self, reply: Optional[Dict[str, Any]]) -> bool:
        if reply is None:
            return False

        return reply["cmd"] == Lpms3Command.REPLY_ACK


    def getModel(self) -> str:
        pkt = self.sendData(Lpms3Command.GET_SENSOR_MODEL)
        if pkt["cmd"] != Lpms3Command.GET_SENSOR_MODEL:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")
        
        return pkt["data"].rstrip(b"\x00").decode(errors="ignore")


    def firmwareInfo(self) -> str:
        pkt = self.sendData(Lpms3Command.GET_FIRMWARE_INFO)
        if pkt["cmd"] != Lpms3Command.GET_FIRMWARE_INFO:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")
        
        return pkt["data"].rstrip(b"\x00").decode(errors="ignore")


    def setFreq(self, freq_hz: int) -> bool:
        payload = struct.pack("<I", freq_hz)
        reply = self.sendData(Lpms3Command.SET_STREAM_FREQ, payload)
        return self.ackGet(reply)
    

    def getFreq(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_STREAM_FREQ)
        if pkt["cmd"] != Lpms3Command.GET_STREAM_FREQ:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")
        
        (freq,) = struct.unpack("<I", pkt["data"])
        return freq


    def getSensorStatus(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_SENSOR_STATUS)
        if pkt["cmd"] != Lpms3Command.GET_SENSOR_STATUS:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (status,) = struct.unpack("<I", pkt["data"])
        return status


    def getSerialNumber(self) -> str:
        pkt = self.sendData(Lpms3Command.GET_SERIAL_NUMBER)
        if pkt["cmd"] != Lpms3Command.GET_SERIAL_NUMBER:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        return pkt["data"].rstrip(b"\x00").decode(errors="ignore")


    def getFilterVersion(self) -> str:
        pkt = self.sendData(Lpms3Command.GET_FILTER_VERSION)
        if pkt["cmd"] != Lpms3Command.GET_FILTER_VERSION:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        return pkt["data"].rstrip(b"\x00").decode(errors="ignore")


    def setImuTransmitData(self, transmit_data: int) -> bool:
        payload = struct.pack("<I", transmit_data)
        reply = self.sendData(Lpms3Command.SET_IMU_TRANSMIT_DATA, payload)
        return self.ackGet(reply)


    def getImuTransmitData(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_IMU_TRANSMIT_DATA)
        if pkt["cmd"] != Lpms3Command.GET_IMU_TRANSMIT_DATA:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (data,) = struct.unpack("<I", pkt["data"])
        return data


    def setImuId(self, imu_id: int) -> bool:
        payload = struct.pack("<I", imu_id)
        reply = self.sendData(Lpms3Command.SET_IMU_ID, payload)
        return self.ackGet(reply)


    def getImuId(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_IMU_ID)
        if pkt["cmd"] != Lpms3Command.GET_IMU_ID:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (imu_id,) = struct.unpack("<I", pkt["data"])
        return imu_id


    def setDegRadOutput(self, deg_rad: int) -> bool:
        payload = struct.pack("<I", deg_rad)
        reply = self.sendData(Lpms3Command.SET_DEGRAD_OUTPUT, payload)
        return self.ackGet(reply)


    def getDegRadOutput(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_DEGRAD_OUTPUT)
        if pkt["cmd"] != Lpms3Command.GET_DEGRAD_OUTPUT:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (deg_rad,) = struct.unpack("<I", pkt["data"])
        return deg_rad


    def setOrientationOffset(self, offset: Tuple[float, float, float]) -> bool:
        payload = struct.pack("<fff", offset[0], offset[1], offset[2])
        reply = self.sendData(Lpms3Command.SET_ORIENTATION_OFFSET, payload)
        return self.ackGet(reply)


    def resetOrientationOffset(self) -> bool:
        reply = self.sendData(Lpms3Command.RESET_ORIENTATION_OFFSET)
        return self.ackGet(reply)


    def setAccRange(self, range_val: int) -> bool:
        payload = struct.pack("<I", range_val)
        reply = self.sendData(Lpms3Command.SET_ACC_RANGE, payload)
        return self.ackGet(reply)


    def getAccRange(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_ACC_RANGE)
        if pkt["cmd"] != Lpms3Command.GET_ACC_RANGE:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (range_val,) = struct.unpack("<I", pkt["data"])
        return range_val


    def setGyrRange(self, range_val: int) -> bool:
        payload = struct.pack("<I", range_val)
        reply = self.sendData(Lpms3Command.SET_GYR_RANGE, payload)
        return self.ackGet(reply)


    def getGyrRange(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_GYR_RANGE)
        if pkt["cmd"] != Lpms3Command.GET_GYR_RANGE:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (range_val,) = struct.unpack("<I", pkt["data"])
        return range_val


    def startGyrCalibration(self) -> bool:
        reply = self.sendData(Lpms3Command.START_GYR_CALIBRATION)
        return self.ackGet(reply)


    def setEnableGyrAutocalibration(self, enable: int) -> bool:
        payload = struct.pack("<I", enable)
        reply = self.sendData(Lpms3Command.SET_ENABLE_GYR_AUTOCALIBRATION, payload)
        return self.ackGet(reply)


    def getEnableGyrAutocalibration(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_ENABLE_GYR_AUTOCALIBRATION)
        if pkt["cmd"] != Lpms3Command.GET_ENABLE_GYR_AUTOCALIBRATION:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (enable,) = struct.unpack("<I", pkt["data"])
        return enable


    def setMagRange(self, range_val: int) -> bool:
        payload = struct.pack("<I", range_val)
        reply = self.sendData(Lpms3Command.SET_MAG_RANGE, payload)
        return self.ackGet(reply)


    def getMagRange(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_MAG_RANGE)
        if pkt["cmd"] != Lpms3Command.GET_MAG_RANGE:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (range_val,) = struct.unpack("<I", pkt["data"])
        return range_val


    def startMagCalibration(self) -> bool:
        reply = self.sendData(Lpms3Command.START_MAG_CALIBRATION)
        return self.ackGet(reply)


    def stopMagCalibration(self) -> bool:
        reply = self.sendData(Lpms3Command.STOP_MAG_CALIBRATION)
        return self.ackGet(reply)


    def setMagCalibrationTimeout(self, timeout: float) -> bool:
        payload = struct.pack("<f", timeout)
        reply = self.sendData(Lpms3Command.SET_MAG_CALIBRATION_TIMEOUT, payload)
        return self.ackGet(reply)


    def getMagCalibrationTimeout(self) -> float:
        pkt = self.sendData(Lpms3Command.GET_MAG_CALIBRATION_TIMEOUT)
        if pkt["cmd"] != Lpms3Command.GET_MAG_CALIBRATION_TIMEOUT:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (timeout,) = struct.unpack("<f", pkt["data"])
        return timeout


    def setFilterMode(self, mode: int) -> bool:
        payload = struct.pack("<I", mode)
        reply = self.sendData(Lpms3Command.SET_FILTER_MODE, payload)
        return self.ackGet(reply)


    def getFilterMode(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_FILTER_MODE)
        if pkt["cmd"] != Lpms3Command.GET_FILTER_MODE:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (mode,) = struct.unpack("<I", pkt["data"])
        return mode


    def writeRegisters(self) -> bool:
        reply = self.sendData(Lpms3Command.WRITE_REGISTERS)
        return self.ackGet(reply)


    def restoreFactoryValue(self) -> bool:
        reply = self.sendData(Lpms3Command.RESTORE_FACTORY_VALUE)
        return self.ackGet(reply)


    def setCanStartId(self, start_id: int) -> bool:
        payload = struct.pack("<I", start_id)
        reply = self.sendData(Lpms3Command.SET_CAN_START_ID, payload)
        return self.ackGet(reply)


    def getCanStartId(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_CAN_START_ID)
        if pkt["cmd"] != Lpms3Command.GET_CAN_START_ID:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (start_id,) = struct.unpack("<I", pkt["data"])
        return start_id


    def setCanBaudrate(self, baudrate: int) -> bool:
        payload = struct.pack("<I", baudrate)
        reply = self.sendData(Lpms3Command.SET_CAN_BAUDRATE, payload)
        return self.ackGet(reply)


    def getCanBaudrate(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_CAN_BAUDRATE)
        if pkt["cmd"] != Lpms3Command.GET_CAN_BAUDRATE:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")

        (baudrate,) = struct.unpack("<I", pkt["data"])
        return baudrate


    def setCanDataPrecision(self, precision: int) -> bool:
        payload = struct.pack("<I", precision)
        reply = self.sendData(Lpms3Command.SET_CAN_DATA_PRECISION, payload)
        return self.ackGet(reply)


    def getCanDataPrecision(self) -> int:
        pkt = self.sendData(Lpms3Command.GET_CAN_DATA_PRECISION)
        if pkt["cmd"] != Lpms3Command.GET_CAN_DATA_PRECISION:
            raise RuntimeError(f"Unexpected reply cmd: {pkt['cmd']:04X}")
            
        (precision,) = struct.unpack("<I", pkt["data"])
        return precision


    def setCanMode(self, mode: int) -> bool:
        payload = struct.pack("<I", mode)
        reply = self.sendData(Lpms3Command.SET_CAN_MODE, payload)
        return self.ackGet(reply)


    def streamData(self):
        self.checkOpen()
        while True:
            yield self.readPacket()

    
    def parseVec3(self, buf: bytes, offset: int):
        x, y, z = struct.unpack_from("<fff", buf, offset)
        return (x, y, z), offset + 12


    def parseVec4(self, buf: bytes, offset: int):
        w, x, y, z = struct.unpack_from("<ffff", buf, offset)
        return (w, x, y, z), offset + 16


    def parse32f(self, data: bytes) -> dict:
        offset = 0
        out = {}
        def hasBytes(n: int) -> bool:
            return len(data) - offset >= n

        if not hasBytes(4):
            raise ValueError(f"IMU payload too short: {len(data)} bytes")

        (ts_ticks,) = struct.unpack_from("<I", data, offset)
        offset += 4
        out["timestamp"] = ts_ticks * 0.002  # 500 Hz counter -> seconds


        def setVec3(name: str):
            nonlocal offset
            if hasBytes(12):
                vals, offset_new = self.parseVec3(data, offset)
                offset = offset_new
                out[name] = vals


        def setVec4(name: str):
            nonlocal offset
            if hasBytes(16):
                vals, offset_new = self.parseVec4(data, offset)
                offset = offset_new
                out[name] = vals


        def setFloat(name: str):
            nonlocal offset
            if hasBytes(4):
                (val,) = struct.unpack_from("<f", data, offset)
                offset += 4
                out[name] = val

        setVec3("acc_raw")
        setVec3("acc_calib")
        setVec3("gyr_raw")
        setVec3("gyr_bias")
        setVec3("gyr_calib")
        setVec3("mag_raw")
        setVec3("mag_calib")
        setVec3("ang_vel")
        setVec4("quat")
        setVec3("euler")
        setVec3("lin_acc")
        setFloat("pressure")
        setFloat("altitude")
        setFloat("temperature")
        return out


    def readOnce(self) -> dict:
        pkt = self.sendData(Lpms3Command.GET_IMU_DATA)
        if pkt["cmd"] != Lpms3Command.GET_IMU_DATA:
            raise RuntimeError(f"Unexpected reply cmd: {hex(pkt['cmd'])}")
        
        return self.parse32f(pkt["data"])


    def readStream(self):
        for pkt in self.streamData():
            if pkt["cmd"] == Lpms3Command.GET_IMU_DATA:
                yield self.parse32f(pkt["data"])


def head4MagAcc(frame, declination_deg=0.0):
    if "mag_calib" in frame: mx, my, mz = frame["mag_calib"]
    elif "mag_raw" in frame: mx, my, mz = frame["mag_raw"]
    else: return None
    if "acc_calib" in frame: ax, ay, az = frame["acc_calib"]
    elif "acc_raw" in frame: ax, ay, az = frame["acc_raw"]
    else: return None
    norm_a = math.sqrt(ax*ax + ay*ay + az*az)
    if norm_a == 0: return None
    ax /= norm_a
    ay /= norm_a
    az /= norm_a
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = mx * math.sin(roll)*math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll)*math.cos(pitch)
    heading = math.degrees(math.atan2(my2, mx2))
    heading = (heading + 360.0) % 360.0
    heading = (heading + declination_deg) % 360.0
    return heading


def head4Mag(frame, declination_deg=0.0):
    if "mag_calib" in frame: mx, my, mz = frame["mag_calib"]
    elif "mag_raw" in frame: mx, my, mz = frame["mag_raw"]
    else: return None
    heading = math.degrees(math.atan2(my, mx))
    heading = (heading + 360.0) % 360.0
    heading = (heading + declination_deg) % 360.0
    return heading
# mpu6500.py
# Minimal CircuitPython driver for the InvenSense MPU6500
# Acceleration in m/s^2, gyro in rad/s, temperature in °C.

import time
import math
from adafruit_bus_device.i2c_device import I2CDevice

class MPU6500:
    _REG_WHO_AM_I     = 0x75
    _REG_PWR_MGMT_1   = 0x6B
    _REG_ACCEL_CONFIG = 0x1C
    _REG_GYRO_CONFIG  = 0x1B
    _REG_ACCEL_XOUT_H = 0x3B  # Burst read 14 bytes covers accel, temp, gyro
    _WHO_AM_I_EXPECTED = 0x70

    _ACCEL_SENS = {  # LSB/g
        2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0,
    }
    _GYRO_SENS = {   # LSB/(deg/s)
        250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4,
    }

    def __init__(self, i2c, address=0x68, accel_range=2, gyro_range=250, verify=True):
        self._dev = I2CDevice(i2c, address)
        self.address = address

        self._buf1 = bytearray(1)
        self._buf2 = bytearray(2)
        self._buf14 = bytearray(14)

        if verify:
            who = self._read_u8(self._REG_WHO_AM_I)
            # Some variants may report a different WHO_AM_I; we won’t hard-fail
            self.whoami_mismatch = (who != self._WHO_AM_I_EXPECTED)
        else:
            self.whoami_mismatch = False

        # Wake up device
        self._write_u8(self._REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # Configure ranges
        self._set_accel_range(accel_range)
        self._set_gyro_range(gyro_range)

    @property
    def acceleration(self):
        ax, ay, az, _, _, _ = self._read_all_raw()
        g = 9.80665
        sf = self._accel_sens
        return (ax / sf * g, ay / sf * g, az / sf * g)

    @property
    def gyro(self):
        _, _, _, gx, gy, gz = self._read_all_raw()
        # raw -> deg/s -> rad/s
        k = math.pi / 180.0
        return (
            (gx / self._gyro_sens) * k,
            (gy / self._gyro_sens) * k,
            (gz / self._gyro_sens) * k,
        )

    @property
    def temperature(self):
        _, _, _, _, _, _, temp_raw = self._read_all_raw_with_temp()
        return (temp_raw / 340.0) + 36.53

    # Configuration helpers
    def _set_accel_range(self, g_range):
        if g_range not in self._ACCEL_SENS:
            raise ValueError("Invalid accel_range. Choose 2, 4, 8, or 16 g")
        fs_sel = {2:0, 4:1, 8:2, 16:3}[g_range] << 3
        current = self._read_u8(self._REG_ACCEL_CONFIG)
        current = (current & ~0x18) | fs_sel
        self._write_u8(self._REG_ACCEL_CONFIG, current)
        self._accel_sens = self._ACCEL_SENS[g_range]

    def _set_gyro_range(self, dps_range):
        if dps_range not in self._GYRO_SENS:
            raise ValueError("Invalid gyro_range. Choose 250, 500, 1000, or 2000 dps")
        fs_sel = {250:0, 500:1, 1000:2, 2000:3}[dps_range] << 3
        current = self._read_u8(self._REG_GYRO_CONFIG)
        current = (current & ~0x18) | fs_sel
        self._write_u8(self._REG_GYRO_CONFIG, current)
        self._gyro_sens = self._GYRO_SENS[dps_range]

    # Raw read helpers
    def _read_all_raw(self):
        self._burst_read_into(self._REG_ACCEL_XOUT_H, self._buf14)
        ax = self._int16(self._buf14[0],  self._buf14[1])
        ay = self._int16(self._buf14[2],  self._buf14[3])
        az = self._int16(self._buf14[4],  self._buf14[5])
        gx = self._int16(self._buf14[8],  self._buf14[9])
        gy = self._int16(self._buf14[10], self._buf14[11])
        gz = self._int16(self._buf14[12], self._buf14[13])
        return ax, ay, az, gx, gy, gz

    def _read_all_raw_with_temp(self):
        self._burst_read_into(self._REG_ACCEL_XOUT_H, self._buf14)
        ax = self._int16(self._buf14[0],  self._buf14[1])
        ay = self._int16(self._buf14[2],  self._buf14[3])
        az = self._int16(self._buf14[4],  self._buf14[5])
        temp = self._int16(self._buf14[6], self._buf14[7])
        gx = self._int16(self._buf14[8],  self._buf14[9])
        gy = self._int16(self._buf14[10], self._buf14[11])
        gz = self._int16(self._buf14[12], self._buf14[13])
        return ax, ay, az, gx, gy, gz, temp

    @staticmethod
    def _int16(msb, lsb):
        val = (msb << 8) | lsb
        return val - 0x10000 if val & 0x8000 else val

    # Low-level (I2CDevice manages locking)
    def _read_u8(self, register):
        with self._dev as i2c:
            self._buf1[0] = register
            i2c.write(self._buf1)
            i2c.readinto(self._buf1)
            return self._buf1[0]

    def _write_u8(self, register, value):
        with self._dev as i2c:
            self._buf2[0] = register
            self._buf2[1] = value & 0xFF
            i2c.write(self._buf2)

    def _burst_read_into(self, start_register, buf):
        with self._dev as i2c:
            self._buf1[0] = start_register
            i2c.write(self._buf1)
            i2c.readinto(buf)
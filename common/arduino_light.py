import struct
from enum import IntEnum

import wpilib


class LedMode(IntEnum):
    Ignore = 0
    Solid = 1
    Swipe = 2
    BlinkSlow = 3
    BlinkFast = 4
    Rainbow = 5


class I2CArduinoLight:
    def __init__(self, i2c_port: wpilib.I2C.Port, i2c_id):
        self.i2c_device = wpilib.I2C(i2c_port, i2c_id)

    def set_leds(self, mode, red, green, blue):
        """
        Set RGB color. RBG Value from 0 to 255
        """
        red = int(max(0, min(255, red)) / 10)
        green = int(max(0, min(255, green)) / 10)
        blue = int(max(0, min(255, blue)) / 10)

        data = struct.pack(
            "BBBB", mode, red, green, blue
        )  # purple ? white ? teal? orange ?
        self.i2c_device.writeBulk(data)

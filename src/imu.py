from machine import UART
import struct

def _check_sum(data: bytearray):
    return sum(data[0:-1]) % 256 == data[-1]


HEADER = 0x55
COMMANDS = [0x51, 0x52, 0x53, 0x54]
ACCEL_WEIGHT = 32768*16*9.81


class IMU:

    def __init__(self):
        self.uart = UART(2, 9600)
        self._buffer = bytearray(11)
        self.buffer_index = 0
        self.header_is_read = False
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def _update_data(self):
        if not _check_sum(self._buffer):
            print("wrong crc")
            print(list(self._buffer))
            return
        if self._buffer[1] == 0x53:
            values = struct.unpack('bbhhhh', self._buffer)
            self.roll = values[2]/32768*180
            self.pitch = values[3]/32768*180
            self.yaw = values[4]/32768*180
            t = values[5]/100

    def parse_data(self, data: bytes):
        if data is None:
            return

        for symbol in data:
            if symbol == 0x55:
                self.header_is_read = True

            if symbol in COMMANDS and self.header_is_read:
                self._update_data()
                self.buffer_index = 1
                self.header_is_read = False

            self._buffer[self.buffer_index % 11] = symbol
            self.buffer_index += 1

    def read_data(self):
        self.parse_data(self.uart.read())


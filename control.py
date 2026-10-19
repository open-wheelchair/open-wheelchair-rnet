import serial
import binascii
import time
import struct

from cobs import cobs

TYPE_JOYSTICK = 1

def encode(buf):
    checksum = binascii.crc32(buf) ^ 0xFFFF_FFFF
    raw = buf + struct.pack('<I', checksum)
    return cobs.encode(raw) + b'\x00'

def map_to_uint(dir):
    if dir >= 0:
        return dir
    else:
        return 256 + dir

def joystick_msg(forward: int, right: int):
    assert -100 <= forward <= 100
    assert -100 <= right <= 100
    return bytes([TYPE_JOYSTICK, map_to_uint(right), map_to_uint(forward)])

def get_dir(): # returns forward, right
    t = int(time.time() / 2) % 2
    if t:
        return 40, 0
    else:
        return -40, 0



with serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_02b5bc8ab9d2eb11a912c149e93fd3f1-if00-port0', baudrate=922190) as ser:
    print(ser.name)         # check which port was really used
    while True:
        begin = time.time()
        while time.time() - begin < 0.05:
            in_waiting = ser.in_waiting
            if in_waiting:
                print(ser.read_all())
        forward, right = get_dir()
        ser.write(encode(joystick_msg(forward, right)))



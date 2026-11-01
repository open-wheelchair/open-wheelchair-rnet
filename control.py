#!/usr/bin/env python3
import binascii
import time
import struct

# pip3 install pyserial cobs
from cobs import cobs
import serial

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

def joystick_msg(forward: int, right: int, is_seating: bool, seating_axis: int):
    assert -100 <= forward <= 100
    assert -100 <= right <= 100

    assert 0 <= seating_axis <= 0xff and (seating_axis == 0 or seating_axis & 0x20)

    return bytes([TYPE_JOYSTICK, map_to_uint(right), map_to_uint(forward), int(is_seating), seating_axis])

def get_msg(elapsed):
    moves = [
        joystick_msg(40, 0, False, 0), # forward
        joystick_msg(0, 0, True, 0x21), # change to seating mode, axis 0x21
        joystick_msg(100, 0, True, 0x21), # move chair up
        joystick_msg(-100, 0, True, 0x21), # move chair down
        joystick_msg(0, 0, False, 0), # change to drive mode
        joystick_msg(-40, 0, False, 0) # backward
    ]

    # 2 seconds per move
    t = int(elapsed / 2) % len(moves)
    return moves[t]


with serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1083HT2-if00-port0', baudrate=115200, exclusive=True) as ser:
    print(ser.name)         # check which port was really used
    starttime = time.time()
    while True:
        begin = time.time()
        while time.time() - begin < 0.05:
            in_waiting = ser.in_waiting
            if in_waiting:
                print(ser.read_all())
        msg = get_msg(time.time() - starttime)
        ser.write(encode(msg))

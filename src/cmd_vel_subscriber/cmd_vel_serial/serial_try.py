import serial
import time
import threading
import struct
import numpy as np


def format_message(cmd_array):
    formatted_numbers = []
    for num in cmd_array:
        if num >= 100:
            num = 99.999
        if num <= -100:
            num = -99.99
    
        if num >= 0 and num < 10:
                formatted_numbers.append(f"{num:.4f} ")

        elif num >= 10:
                formatted_numbers.append(f"{num:.3f} ")

        elif num < 0 and num > -10:
            formatted_numbers.append(f"{num:.3f} ")

        elif num <= -10:
            formatted_numbers.append(f"{num:.2f} ")

    float_string = "".join(formatted_numbers).rstrip()
    byte_array = float_string.encode('utf-8')
    return byte_array


def format_float(array):
    
    bts = []
    for num in array:
        for byte in struct.pack('<f',num):
            bts.append(byte)
    
    b=bytearray(bts)

    return b


def main():
    end = True
    pitch_disp = 5.0
    yaw_disp = 5.0
    latest_cmd_vel = None
    buffer_lock = threading.Lock()
    serial_port1 = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=230400,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
    )

    serial_port2 = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=230400,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
    )



    while (end):
        #firstChar=serial_port.read(1)
        #if serial_port.inWaiting() > 0:
             #if ord(firstChar) == 37:
        pitch_disp = pitch_disp + 0.01
        yaw_disp = yaw_disp + 0.01
        #print(f"port read: {serial_port.read(69)}")
        formatted_data = formatted_data = format_float(np.array([pitch_disp, 2.0+pitch_disp, 3.0, 0.0, 0.0], dtype=np.float32))
        serial_port1.write(formatted_data)
        serial_port2.write(formatted_data)
        print((f"formatted data {formatted_data}"))
        #print((f"Data sent: {pitch_disp}, {yaw_disp}"))
        time.sleep(0.1)


if __name__ == '__main__':
    main()

#formatted_data = format_float(np.array([self.yaw, self.pitch, self.shoot_frequency, 0.0, 0.0], dtype=np.float32))    
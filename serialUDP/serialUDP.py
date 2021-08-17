#!/usr/bin/python3

import time
import serial
import socket
from time import sleep
from threading import Thread
import threading

UDP_IP = 'localhost'
UDP_PORT = 5005
SERIAL_PORT = "/dev/ttyUSB0"

print("Running UART to UDP bridge")

def serial2UDP(buf):

    while True:
        # _bytes = ser.in_waiting
        _bytes = ser.inWaiting()
        if _bytes:
            #read serial and convert to hex
            in_bin = ser.read(_bytes)
            #ser.flush()
            #in_hex = hex(int(in_bin.encode('hex'), 16))
            in_hex = in_bin
            #print("[UART] read data")
            #send to UDP port
            sock.sendto(in_hex, (UDP_IP, UDP_PORT))
            #print("[UDP] write data")
        else:
            # print("No serial data")
            pass
    sleep(0.01)

def udp2Serial(buf):
  while True:
    try:
        #read data from UDP
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        print("[UDP] read data")
        #send to serial
        ser.write(data)
        ser.flush()
        # print("[UART] write data")
    except socket.error:
            # print("[UDP] No data recv")
            pass
    sleep(0.01)

# sleep(10)
f_ser_config = True
while f_ser_config:
    try:
        ser = serial.Serial(
        port = SERIAL_PORT,
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
        )

        ser.setRTS(False)
        ser.setDTR(True)
        ser.setDTR(False)

        f_ser_config= False
    except:
        print("[UART] config failed")
        sleep(3)
        f_ser_config = True
print("Serial config done ")

sock = socket.socket(socket.AF_INET, # Internet
                      socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('localhost', 5005))

sock.setblocking(0)
print("UDP config done")

count = 0
_buf = [1,2,3]


try:
    t1= threading.Thread(target=serial2UDP, args=(_buf,))
    t2= threading.Thread(target=udp2Serial, args=(_buf,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    pass
except:
    print("Thread error")

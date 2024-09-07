#!/usr/bin/env python3
import time
import serial

serialPort = '/dev/ttyACM0'
serialBaudrate = 115200

serialObject = serial.Serial(
    port=serialPort, 
    baudrate=serialBaudrate,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, 
    timeout=0.01
) 

time.sleep(10)
initalstring = "!Initon#"
serialObject.write(bytes(str(initalstring), encoding='utf-8'))

autostring = "goauto"
serialObject.write(bytes(str(autostring), encoding='utf-8'))

while True:
    # pygame.event.pump()
    data = serialObject.readline().decode(encoding='utf-8')
    if data == "ACK\r\n": 
        print("acknowledage")
        ack = True
    elif data == "KCA\r\n": 
        print("stopped")
        ack = True
    # Input : !pos1:pos2:pos3:pos4:pos5:pos6#
    # Example : 0:0:0:0:90:0
    # manual, gohome, gofold
    string = input("Nhập một chuỗi: ")
    print('send: ', string)
    serialObject.write(bytes(str(string), encoding='utf-8'))


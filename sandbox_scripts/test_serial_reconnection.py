#!/usr/bin/env python3

import serial
import time

def serialRead(serialPort):    
    # ser = serial.Serial(serialPort, 115200, timeout=0.04)
    ser = None

    while(True):
        try:
            if(ser == None):
                print("Trying to reconnect")
                ser = serial.Serial(serialPort, 115200, timeout=0.04)
                print("Reconnected")

            output = ser.read()

            print(output.decode('utf-8'))
            try:
                print(int(output.decode('utf-8')))
            except:
                print("garbage data")
                pass

        except serial.serialutil.SerialException:
            if(not(ser == None)):
                ser.close()
                ser = None
                print("Disconnecting")

            print("No Connection")
            time.sleep(0.01)


if __name__ == '__main__':
    serialPort = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0"
    serialRead(serialPort)


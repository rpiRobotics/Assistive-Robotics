
from __future__ import absolute_import

from . import ust_commands as cmds
import serial
import time
import rospy
from rospy.core import logwarn


class UST20LN_Handler():
    """
    Create a hokuyo device object for communication
    """

    def __init__(self):
        self.port = ""
        self.baudrate = 115200
        self.ser = None
    
    def connect(self, port, baudrate = 115200):
        """
        Attempt to establish connection with the scanner
        If the attempt fails, the method will return False otherwise, True.
        """
        self.port = port
        self.baudrate = baudrate
        
        while True:
            try: # attempt to create a serial object and check its status
                if(self.ser == None):
                    rospy.loginfo("Trying to reconnect to serial")
                    self.ser = serial.Serial(
                        port = self.port,
                        baudrate = self.baudrate,
                        parity = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        timeout = 0.026
                    )
                    rospy.loginfo("Connected to Hokuyo serial")
                    self.stop_ranging()
                    return
                else:
                    return
                    
            except serial.serialutil.SerialException:
                if(not(self.ser == None)):
                    self.ser.close()
                    self.ser = None
                    rospy.logwarn("Disconnecting from Hokuyo serial")
                rospy.logwarn("Hokuyo Serial disconnected")
                time.sleep(0.026)
                        
    
    def send_command(self, command, timeout=0.026):
        try:
            self.connect(self.port,self.baudrate)

            raw_command = "%s\n"%(command.command)
            self.ser.write(raw_command.encode())
            # rospy.loginfo("sending: " + raw_command)
            self.ser.reset_input_buffer()

            data = b''
            start_time = time.time()
            if command.answer_expected and command.answer != '':    #precise answer expected, search for it !
                while time.time() - start_time < timeout:
                    if self.ser.in_waiting:
                        data+=self.ser.read()
                        data = data.split(b'\n')[-1]
                        if command.answer.encode() in data:
                            break
                return data
            elif command.answer_expected:   # answer expected but be don't known which : return the first one (until \n)
                while time.time() - start_time < timeout:
                    if self.ser.in_waiting:
                        data+=self.ser.read()
                        if b'\n' in data:
                            data = data.split(b'\n')[0]
                            break
                return data
            else:
                return b''

        except serial.serialutil.SerialException:
            if(not(self.ser == None)):
                self.ser.close()
                self.ser = None
                rospy.logwarn("Disconnecting from Hokuyo serial")
            rospy.logwarn("Hokuyo Serial disconnected")
            time.sleep(0.026)
            self.connect(self.port,self.baudrate)
    
    def stop_ranging(self):
        self.send_command(cmds.STOP_RANGING)

    def start_ranging(self):
        self.stop_ranging()
        self.send_command(cmds.START_RANGING)

    def stop(self):
        self.stop_ranging()
    
    def get_measures(self):
        """
        returns measures under the form (timestamp, [(distance, quality), ...])
        timestamp : time in seconds since the LIDAR startup
        distance range : 0 - ??? (mm)
        quality range : 0 - ??? 
        eg: (102.123456, [(552, 1244), (646, 1216), (676, 1270), ...])
        """
        raw_bytes=self.ser.read(cmds.MEASURE_LENGHT_UST_20LN)
        data = raw_bytes.split(b':')
        # print(data)
        timestamp = int(data[1], 16)/10**6
        measurement_bytes = data[3]
        measurements = [(int(measurement_bytes[i:i+4],16), int(measurement_bytes[i+4:i+8],16))  for i in range(0, len(measurement_bytes)-8, 8)]
        return (timestamp, measurements)
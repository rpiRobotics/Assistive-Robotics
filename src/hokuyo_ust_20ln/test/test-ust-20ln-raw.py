from serial import Serial
import time
from collections import namedtuple

class UST:
    
    Command = namedtuple('Command',['command', 'answer_expected', 'answer'])

    START_RANGING = Command('#GT15466', False, '')
    STOP_RANGING = Command('#ST5297', True, '#ST00A845')
    ID = Command('#IN0D54', True, '')
    ID2 = Command('#CLC2DD', True, '')

    PLOP = Command('#GR0EEE1', True, '')
    
    MESURE_LENGHT = 8679 # = 1081steps *8bytes + 31 bytes  # For ust-05LN  4359 bytes = 541steps * 8bytes + 31 bytes 

    def __init__(self, port="/dev/ttyACM0", baudrate=115200): #/dev/ttyACM0 or COM7
        self.ser = Serial(port, baudrate)
        self.stop_ranging()

    def send_command(self, command, timeout=2):
        self.ser.write(command.command.encode()+b'\n')    # writes command to LIDAR
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

    def stop_ranging(self):
        self.send_command(self.STOP_RANGING)

    def start_ranging(self):
        self.stop_ranging()
        self.send_command(self.START_RANGING)

    def stop(self):
        self.stop_ranging()
    
    def get_measures(self):
        """
        returns measures under the form (timestamp, [(distance, quality), ...])
        timestamp : time in seconds since the LIDAR startup
        distance range : 0 - 65635
        valur range : 0 - ??? (65635 max)
        eg: (102.123456, [(552, 1244), (646, 1216), (676, 1270), ...])
        """
        raw_bytes=self.ser.read(ust.MESURE_LENGHT)
        data = raw_bytes.split(b':')
        # print(data)
        timestamp = int(data[1], 16)/10**6
        measurement_bytes = data[3]
        measurements = [(int(measurement_bytes[i:i+4],16), int(measurement_bytes[i+4:i+8],16))  for i in range(0, len(measurement_bytes)-8, 8)]
        return (timestamp, measurements)
    
    def get_data(self, nb_bytes):
        data = self.ser.read(nb_bytes)
        data = data.split(b':')
        data = data[3]
        aa = [(int(data[i:i+4],16), int(data[i+4:i+8],16))  for i in range(0, len(data)-8, 8)]
        print('data received')
    

if __name__ == "__main__":
    ust = UST()
    try:
        ret = ust.send_command(ust.ID)
        print(ret.decode())
        ust.start_ranging()
        while True:
            data = ust.get_measures()
            if len(data[1]) == 1081 : # Number of steps for measurements for 05LN = 541, for 20LN = 1081
                # print("ok at {:.06f} s".format(data[0]))
                # print(data[1][540])
                print(len(data[1]))
            else:
                print(len(data[1]))
    finally:
        ust.stop()


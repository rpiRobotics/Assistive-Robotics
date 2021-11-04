import serial
import time
import rospy
from rospy.core import logwarn
# message
class RoboteqHandler:
    """
    Create a roboteq device object for communication
    """

    def __init__(self):
        self.port = ""
        self.baudrate = 115200
        self.ser = None
    
    def connect(self, port, baudrate = 115200):
        """
        Attempt to establish connection with the controller
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
                        timeout = 0.04
                    )
                    rospy.loginfo("Connected to serial")
                    return
                else:
                    return
                    
            except serial.serialutil.SerialException:
                if(not(self.ser == None)):
                    self.ser.close()
                    self.ser = None
                    rospy.logwarn("Disconnecting from serial")
                rospy.logwarn("Serial disconnected")
                time.sleep(0.04)
            

    def request_handler(self, request= ""):
        """
        Sends a command and a parameter, 
        """
        try:
            self.connect(self.port,self.baudrate)

            self.ser.reset_input_buffer()

            raw_command = "%s\r"%(request)
            self.ser.write(raw_command.encode())
            # rospy.loginfo("sending: " + raw_command)

            char_echo = self.ser.read_until('\r') # This is the char echo
            # rospy.loginfo("recieve char_echo: " + char_echo)
            
            result = self.ser.read_until('\r') # Actual response
            # rospy.loginfo("recieve result: " + result)

            if char_echo != raw_command:
                rospy.logwarn("char_echo: '" + str(char_echo)  + "' is not the same as the raw command: '" + str(raw_command) + "'" )

            return result
        except serial.serialutil.SerialException:
            if(not(self.ser == None)):
                self.ser.close()
                self.ser = None
                rospy.logwarn("Disconnecting from serial")
            rospy.logwarn("Serial disconnected")
            time.sleep(0.04)
            self.connect(self.port,self.baudrate)
            
    
    def send_command(self, command, first_parameter = "", second_parameter = ""):
        if first_parameter != "" and second_parameter != "":
            message = "%s %s %s "%(command,first_parameter,second_parameter)
        if first_parameter != "" and second_parameter == "":
            message = "%s %s "%(command,first_parameter)
        if first_parameter == "" and second_parameter == "":
            message = "%s "%(command)
        
        response = self.request_handler(message)

        if str(response) != "+\r":
            rospy.logwarn("Motor command is not Acknowledged. The response was: '" + str(response) + "'")

        return response

    def read_value(self, command= "", parameter = ""):
        """
        Constructs a message and sends it to the controller.
        param: command (str)
        param: parameter (str/int)
        returns: answer from the controller, data from request commands, or echo from action commands.
        """
        request = "%s %s"%(command,parameter)
        response = self.request_handler(request)
        return response

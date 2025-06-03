import rclpy
from roboteq_core.srv import Encoder
from rclpy.node import Node
from std_msgs.msg import Int16
import serial
import os
import configparser
import sys
roboteq_obj = serial.Serial(
port='/dev/ttyTHS0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1)

RETRIES = 10
CONFIG_FILE = "config.cfg"  # name of config file

class motor_driver(Node):

    def __init__(self):
        super().__init__('cmd_roboteq2')

        # Get config file dir
        config_file_path = os.getcwd() 
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))
        # Read config file 
        config = configparser.ConfigParser()
        if not config.read(config_file_path):
            self.get_logger().error(f"Config file {CONFIG_FILE} not found or invalid.")
            rclpy.shutdown()
            return


        command_subscriber_topic = config['Node']['command_subscriber_topic'] 

        self.inCmd = 0.0
        self.subscription = self.create_subscription(
            Int16,
            command_subscriber_topic,
            self.cmd_callback,
            5)
        self.subscription  # prevent unused variable warning

    def move_motors(self, val):
        payload1 = "!G 1 " + str(-val) + "_"
        payload2 = "!G 2 " + str(val) + "_"
        roboteq_obj.write(str.encode(payload1))
        roboteq_obj.write(str.encode(payload2))
            
    def read_encoder(self, absolute):
        payload1 = "?C" + "R" if absolute else "" + " 1" 
        payload2 = "?C" + "R" if absolute else "" + " 2"
        roboteq_obj.write(str.encode(payload1))
        roboteq_obj.write(str.encode(payload2))

        encoder1 = encoder2 = 0
        read_timer = 0.001
        for retry in range(RETRIES):
            resp = self.connect.handle_serial_request(self.read_cmds["encoder"], read_timer)
            if not resp:
                continue
            resp = resp.split('\r')
            if len(resp) > 1 and ("C=" in resp[1]):
                resp = resp[1][2:].split(":")
                try:
                    encoder1 = int(resp[0])
                    encoder2 = int(resp[1])
                    break
                except (ValueError, IndexError) as e:
                    print(f"Retrying. Exception: {e}", file=sys.stderr)
        print(f"encoder1: {encoder1}")
        print(f"encoder2: {encoder2}")


    def cmd_callback(self, msg):
        inCmd = msg.data
        self.move_motors(inCmd)

class read_encoder(Node):

    def __init__(self):
        super().__init__('read_encoder')
        self.srv = self.create_service(encoder, 'read_encoder', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = motor_driver()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

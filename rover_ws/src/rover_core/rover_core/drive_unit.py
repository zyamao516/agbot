import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray
import os
import configparser
import serial
import time

CONFIG_FILE = "robot_config.cfg"
TIMER_CALLBACK_FREQUENCY_SECONDS = 0.05

class MotorDriver(Node):
    
    def __init__(self):
        # Get config file dir
        config_file_path = os.getcwd() 
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE)) 
        # Read config file 
        config = configparser.ConfigParser()
        config.read(config_file_path)
        
        # Name vars for readability
        robot_name              = config['Robot']['name']
        self.roboteq_objs       = list()
        drive_unit_id           = config['Roboteq']['drive_unit_ids'].split(', ')
        ports                   = config['Roboteq']['ports'].split(', ')
        baudrates               = config['Roboteq']['baudrates'].split(', ')
        encoder_topic           = config['Roboteq']['encoder_topic']
        drive_unit_topic        = config['Robot']['drive_unit_topic']
        self.encoder_cmd        = config['Roboteq']['encoder_command']
        queue_length            = int(config['Robot']['queue_length'])
        subscription_topic_name = f'{robot_name}/{drive_unit_id}/{drive_unit_topic}'

        if len(ports) != len(baudrates):
            raise ValueError("number of dive unit ids, ports, and baudrates don't match")
        
        for index in range(len(ports)):
            self.roboteq_objs.append(serial.Serial(
                port=ports[index],
                baudrate=int(baudrates[index]),
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1))

        super().__init__(config['Roboteq']['node_name'])
        
        self.subscription = self.create_subscription(
            Int16,
            subscription_topic_name,
            self.cmd_callback,
            queue_length)
        self.subscription  # prevent unused variable warning

        self.encoder_publisher = self.create_publisher(Int32MultiArray, f"/{robot_name}/{drive_unit_id}/{encoder_topic}", queue_length)
        self.timer = self.create_timer(TIMER_CALLBACK_FREQUENCY_SECONDS, self.encoder_callback)

    def cmd_callback(self, msg):
        for roboteq in self.roboteq_objs:
            roboteq.write(str.encode(f"!G 1 {str(-msg.data)}_"))
            roboteq.write(str.encode(f"!G 2 {str(msg.data)}_"))

    def encoder_callback(self, read_timer):
        message = Int32MultiArray()
        for roboteq in self.roboteq_objs:
            roboteq.write(self.encoder_cmd.encode())
            time.sleep(read_timer)
            resp = roboteq.read_all().decode('utf-8', "ignore").split("\r")
            if len(resp) > 1 and ("C=" in resp[1]):
                resp = resp[1][2:].split(":")
                try:
                    for idx in range(len(resp)):
                        message.data[idx]= int(resp[idx])
                    break
                except (ValueError, IndexError) as e:
                    print(f"Retrying. Exception: {e}")
                self.encoder_publisher.publish(message)



    
def main(args=None):
    rclpy.init(args=args)
    roboteq_node = MotorDriver()
    rclpy.spin(roboteq_node)
    roboteq_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import configparser
from rover_core.kinematics import drive_unit_evaluation, get_expression_length
from rover_core.dead_mans_switch import Robot_Enable

ENABLE_STATE_TOPIC = "enable_state"
PLAY_MODE_COMMAND_TOPIC = "cmd_vel"
CONFIG_FILE = "robot_config.cfg"

class RoverProcessor(Node):


    def __init__(self):


        #------------VARIABLE INITIALISATION-------------


        # Initialize the class variables
        self.enabled = False
        self.state = "Neutral"
        self.motor_gain = 1.0
        self.linear_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.angular_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.drive_unit_publishers = []
        self.dead_mans_switch = Robot_Enable()
        self.dead_mans_switch.contactor_ctrl(False)

        # Get config file dir
        config_file_path = os.getcwd()
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))  
        # Read config file
        config = configparser.ConfigParser()
        if not config.read(config_file_path):
            self.get_logger().error(f"Config file {CONFIG_FILE} not found or invalid.")
            rclpy.shutdown()
            return

        # Assign new vars for code readability
        node_name = "core"
        robot_name = config['Robot']['name']
        drive_unit_topic = config['Robot']['drive_unit_topic']
        control_topic = config['Robot']['control_topic']
        queue_length = int(config['Robot']['queue_length'])
        drive_unit_count = int(config['Movebase_Kinematics']['drive_unit_count'])
        self.motor_gain = float(config['Movebase_Kinematics']['gain'])
        self.enable_topic_name = f'{robot_name}/{ENABLE_STATE_TOPIC}'
        self.enabled = False

        if drive_unit_count != get_expression_length():
            self.get_logger().error(f"Drive unit count {drive_unit_count} not same as expression count {get_expression_length()}")
            rclpy.shutdown()
            return

        # Initialize the ROS 2 node with the robot name and node name
        super().__init__(f'{robot_name}_{node_name}')

        
        #------------SUBSCRIBERS------------


        # Enable state subscriber
        self.enable_state_subscriber = self.create_subscription(
            String,
            f'/{robot_name}/{ENABLE_STATE_TOPIC}',  # Replace with your actual topic name
            self.enable_state_callback,
            queue_length,
        )
        self.enable_state_subscriber  # Prevent unused variable warning

        # Command subscriber
        self.control_subscriber = self.create_subscription(
            Twist,
            f'/{robot_name}/{control_topic}',
            self.move_cmd_callback,
            queue_length)
        self.control_subscriber
        
        # Timer to periodically check if the enable_state publisher is alive
        self.liveliness_timer = self.create_timer(0.5, self.check_liveliness)


        #-------------PUBLISHERS------------


        # Create publishers for each drive unit
        for x in range(drive_unit_count):
            publisher = self.create_publisher(Float32, f'{robot_name}/drive_unit_{x}/{drive_unit_topic}', queue_length)
            self.drive_unit_publishers.append(publisher)

    def check_liveliness(self):
        publishers_info = self.get_publishers_info_by_topic(self.enable_topic_name, False)
        if len(publishers_info) == 0 and self.enabled:
            # Publisher is disconnected, disable the robot
            self.enabled = False
            self.dead_mans_switch.contactor_ctrl(False)
            self.get_logger().warn("Enable state publisher is lost, disabling robot.")
    
    def enable_state_callback(self, msg):
        self.enabled = True
        self.dead_mans_switch.contactor_ctrl(True)
        self.state = msg.data
    
    def move_cmd_callback(self, msg):
        # Update linear and angular velocities from the Twist message
        self.linear_velocity['x'] = msg.linear.x
        self.linear_velocity['y'] = msg.linear.y
        self.linear_velocity['z'] = msg.linear.z
        self.angular_velocity['x'] = msg.angular.x
        self.angular_velocity['y'] = msg.angular.y
        self.angular_velocity['z'] = msg.angular.z

        # Publish velocities to drive units
        for x in range(len(self.drive_unit_publishers)):
            message = Float32()
            if self.enabled or self.state == "Neutral":
                message.data = drive_unit_evaluation(self.linear_velocity, self.angular_velocity, self.motor_gain, x)
            else:
                message.data = 0.0
            self.drive_unit_publishers[x].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RoverProcessor()
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(node)
    try:
        # Spin the executor to handle callbacks
        executor.spin()
    finally:
        # Clean up
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

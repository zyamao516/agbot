import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import os
import configparser
import sys

MAINTAIN = False    # a debug macro for those who are computationally challenged and is reading this code for the first time
                    # enable it to see what the member vars hold
CONFIG_FILE = "config.cfg"  # name of config file
TIMER_CALLBACK_FREQUENCY_SECONDS = 0.05

class Multirobot_Controller(Node):

    def __init__(self):
        super().__init__('Multirobot_Controller')
        # Get config file dir
        config_file_path = os.getcwd() 
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))
        # Read config file 
        config = configparser.ConfigParser()
        if not config.read(config_file_path):
            self.get_logger().error(f"Config file {CONFIG_FILE} not found or invalid.")
            rclpy.shutdown()
            return
        
        # initialise config vars from config file for readability
        command_publish_topic       = config['Node']['command_publisher_topic']
        command_subscriber_topic    = config['Node']['command_subscriber_topic'] 
        self.queue_length           = int(config['Node']['queue'])
        self.display                = bool(config['Node']['display'])
        self.topic_name             = config['Topics']['topic_name']
        self.publisher_values       = ["Neutral", "Teach", "Play"]
        self.robot_list             = config['Robots']['robots'].split(', ')
        self.robot_states           = dict()
        self.cmd_publishers         = dict()
        self.cmd_subscribers        = dict()
        self.enable_publisher_dict  = dict()
        self.enable_button          = int(config['Joy']['enable_button'])
        self.enable_all_button      = int(config['Joy']['enable_all_button'])
        self.last_robot             = 0
        self.cycle_robots_button    = int(config['Joy']['cycle_robots_button'])
        self.cycle_mode_button      = int(config['Joy']['cycle_mode_button'])
        self.linear_x_axis          = int(config['Joy']['linear_x_axis'])
        self.angular_z_axis         = int(config['Joy']['angular_z_axis'])
        self.linear_gain            = float(config['Joy']['linear_gain'])
        self.angular_gain           = float(config['Joy']['angular_gain'])
        self.current_robot          = 0
        self.robot_toggle_flag      = False
        self.mode_toggle_flag       = False
        self.enable_toggle_flag     = False

        self.joystick_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            self.queue_length)
        self.joystick_subscriber

        # initialises robot states with the default state of robots
        for robot in self.robot_list:
            self.robot_states[robot] = {'Mode' : self.publisher_values[0], 'Enabled' : True} # used dict for readability, you're welcome
            self.cmd_publishers[robot] = self.create_publisher(Twist, f"/{robot}/{command_publish_topic}", self.queue_length)
            self.cmd_subscribers[robot] = self.create_subscription(Twist, f"/{robot}/{command_subscriber_topic}", self.create_command_callback(robot), self.queue_length) 
        
        # Debug macro
        if MAINTAIN:
            print(f"Topic name: {self.topic_name}")
            print(f"Publisher values: {self.publisher_values}")
            print(f"Robot list: {self.robot_list}")
            print(f"Robot states: {self.robot_states}")

        self.timer = self.create_timer(TIMER_CALLBACK_FREQUENCY_SECONDS, self.timer_callback)
        print("\033[2J",flush=True)
        self.print_all_robot_states()

    def __del__(self):
        #print("\033[?25h",end="")
        sys.stdout.flush()

    def format_mode(self, mode):
        match mode:
            case "Neutral":
                return f"\033[1m\033[0;33m{mode}\033[0m"
            case "Teach":
                return f"\033[1m\033[0;35m{mode}\033[0m"
            case "Play":
                return f"\033[1m\033[0;36m{mode}\033[0m"
            case _:
                return ""

    def format_enabled(self, enabled):
        match enabled:
            case False:
                return f"\033[1m\033[0;31m{enabled}\033[0m"
            case True:
                return f"\033[1m\033[0;32m{enabled}\033[0m"
            case _:
                return ""

    def print_robot_states(self, index):
        if not self.display:
            return
        if index == 0:
            print("\033[H", end="", flush=True)  # Move to the top-left corner for index 0
        else:
            print(f"\033[H\033[{index}B", end="", flush=True) 
        robot = self.robot_list[index]
        state = self.robot_states[robot]
        print("\033[2K",end="")
        if self.current_robot == index:
            print("\033[4m",end="")
        print(f"\033[0G\033[1m\033[0;34m({index}) {robot}\033[0m | Mode : {self.format_mode(state['Mode'])} | State : {self.format_enabled(state['Enabled'])}\033[{len(self.robot_list)-index}B", flush = True)
        #sys.stdout.flush()

    def print_all_robot_states(self):
    
        for index in range(len(self.robot_list)):
            self.print_robot_states(index)

    def joy_callback(self, msg):
        message = Twist()

        # cycle through selected robot
        if msg.buttons[self.cycle_robots_button] and not self.robot_toggle_flag:
            prev_robot_index = self.current_robot
            self.current_robot = (self.current_robot+1)%len(self.robot_list)
            self.robot_toggle_flag = True
            self.print_robot_states(prev_robot_index)
            self.print_robot_states(self.current_robot)
        elif not msg.buttons[self.cycle_robots_button]:
            self.robot_toggle_flag = False

        # cycle though mode of selected robot
        if msg.buttons[self.cycle_mode_button] and not self.mode_toggle_flag:
            current_mode = self.robot_states[self.robot_list[self.current_robot]]['Mode']  
            self.robot_states[self.robot_list[self.current_robot]]['Mode'] = self.publisher_values[(self.publisher_values.index(current_mode)+1)%len(self.publisher_values)] 
            self.mode_toggle_flag = True
            self.print_robot_states(self.current_robot)
        elif not msg.buttons[self.cycle_mode_button]:
            self.mode_toggle_flag = False

        # enable robot though dead mans switch
        if msg.buttons[self.enable_button]:
            if self.current_robot != self.last_robot:
                self.robot_states[self.robot_list[self.last_robot]]['Enabled'] = False
                self.print_robot_states(self.last_robot)
                self.last_robot = self.current_robot
            message.linear.x = self.linear_gain*msg.axes[self.linear_x_axis]
            message.angular.z = self.angular_gain*msg.axes[self.angular_z_axis]
            self.robot_states[self.robot_list[self.current_robot]]['Enabled'] = True
            self.enable_toggle_flag = True
            self.print_robot_states(self.current_robot)
        elif msg.buttons[self.enable_all_button]:
            message.linear.x = self.linear_gain*msg.axes[self.linear_x_axis]
            message.angular.z = self.angular_gain*msg.axes[self.angular_z_axis]
            for robot in self.robot_states.values():
                robot['Enabled'] = True
                self.enable_toggle_flag = True
            self.print_all_robot_states()
        else:
            for robot in self.robot_states.values():
                robot['Enabled'] = False
            if self.enable_toggle_flag:
                self.enable_toggle_flag = False
                self.print_all_robot_states()

        for robot in self.robot_list:
            if self.robot_states[robot]['Enabled'] and self.robot_states[robot]['Mode'] == self.publisher_values[2]: # if in play mode
                self.cmd_publishers[robot].publish(message)

    def create_command_callback(self, robot):
        # Define a callback for a specific robot
        def command_callback(msg):
            message = Twist()
            if self.robot_states[robot]['Enabled'] and self.robot_states[robot]['Mode'] == self.publisher_values[1]: # if in Teach mode
                message = msg
            elif self.robot_states[robot]['Enabled'] and self.robot_states[robot]['Mode'] == self.publisher_values[2]: # if in play mode
                return
            self.cmd_publishers[robot].publish(message)
        return command_callback

    def timer_callback(self):
        for robot in self.robot_list:
            # if state is false and publisher isnt created
            # do nothing, for all is right and just
            if not self.robot_states[robot]['Enabled'] and robot not in self.enable_publisher_dict:
                continue
            # else if state is false and publisher exists, destroy publisher and remove from dict
            elif not self.robot_states[robot]['Enabled'] and robot in self.enable_publisher_dict:
                self.destroy_publisher(self.enable_publisher_dict[robot])
                del self.enable_publisher_dict[robot] 
                continue
            # else if state is true and not in enable_publisher_dict
            # create a new publisher and add it to the dict
            elif self.robot_states[robot]['Enabled'] and robot not in self.enable_publisher_dict:
                self.enable_publisher_dict[robot] = self.create_publisher(String, f"/{robot}/{self.topic_name}", self.queue_length)
            # if code falls through to here the robot is enabled and a publisher exists in enable_publisher_dict
            msg = String()
            msg.data = self.robot_states[robot]['Mode']
            self.enable_publisher_dict[robot].publish(msg)

def main(args=None):
    # clear screen
    print("\033[2J",end="")
    rclpy.init(args=args)
    node = Multirobot_Controller()
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(node)
    try:
        # Spin the executor to handle callbacks
        executor.spin()
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, shutting down...")
    finally:
        # Clean up
        executor.shutdown()
        node.destroy_node()

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except rclpy.RCLError:
            print("Error: rcl_shutdown already called.")


if __name__ == '__main__':
    main()

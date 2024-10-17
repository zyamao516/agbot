import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os
import configparser

MAINTAIN = False    # a debug macro for those who are computationally challenged and is reading this code for the first time
                    # enable it to see what the member vars hold
TOPIC_QUEUE_LENGTH = 10
TIMER_CALLBACK_FREQUENCY_SECONDS = 0.1

class Multirobot_Controller(Node):
    # Config vars
    topic_name = str()
    publisher_values = list()   # holds string literals of states, done for readability. Its for the mechs ok? i would have used a uint8 too jeez
    robot_list = list()         # also holds string literals but of robot names like r1 or r2, done to dynamically create topics with the right name
    # Initialised var
    robot_states = dict()
    # Ros2 publisher var, holds publishers in a list
    publisher_dict = dict()

    def __init__(self):
        # Get config file dir
        config_file_path = os.path.join(
                get_package_share_directory('multirobot_controller'),
                'config',
                'config.cfg'
                )
        # Read config file
        config = configparser.ConfigParser()
        config.read(config_file_path)

        self.topic_name =       config['Topics']['topic_name']
        self.publisher_values = config['Topics']['publisher_values'].split(', ')
        self.robot_list =       config['Robots']['robots'].split(', ')

        # Error checking
        assert self.topic_name is not None, "[Config Error] Invalid topic name"
        assert self.publisher_values is not None and len(self.publisher_values) > 0, "[Config Error] Invalid publisher values"
        assert self.robot_list is not None and len(self.robot_list) > 0, "[Config Error] Invalid robot list"

        # initialises robot states with the default state of robots
        for robot in self.robot_list:
            self.robot_states[robot] = {'Mode' : self.publisher_values[0], 'Enabled' : True} # used dict for readability, you're welcome
        
        # Debug macro
        if MAINTAIN:
            print(f"Topic name: {self.topic_name}")
            print(f"Publisher values: {self.publisher_values}")
            print(f"Robot list: {self.robot_list}")
            print(f"Robot states: {self.robot_states}")

        super().__init__('Multirobot_Controller')
        self.timer = self.create_timer(TIMER_CALLBACK_FREQUENCY_SECONDS, self.timer_callback)

    def timer_callback(self):
        for robot in self.robot_list:
            # if state is false and publisher isnt created
            # do nothing, for all is right and just
            if not self.robot_states[robot]['Enabled'] and robot not in self.publisher_dict:
                continue
            # else if state is false and publisher exists, destroy publisher and remove from dict
            elif not self.robot_states[robot]['Enabled'] and robot in self.publisher_dict:
                self.destroy_publisher(self.publisher_dict[robot])
                del publisher_dict[robot] 
                continue
            # else if state is true and not in publisher_dict
            # create a new publisher and add it to the dict
            elif self.robot_states[robot]['Enabled'] and robot not in self.publisher_dict:
                self.publisher_dict[robot] = self.create_publisher(String, f"/{robot}/{self.topic_name}", TOPIC_QUEUE_LENGTH)
            # if code falls through to here the robot is enabled and a publisher exists in publisher_dict
            msg = String()
            msg.data = self.robot_states[robot]['Mode']
            self.publisher_dict[robot].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Multirobot_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class mpc(Node):

    def __init__(self):
        super().__init__('mpc')

    def baby(self):
        a = 1.0
        return a


    def solve_mpc(self, state):
        a = self.baby()
        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        velocity_input.data = [a, a]
        steering_input.data = [0.7, 0.5, 0.5, 0.5, 0.0]
        return velocity_input,steering_input


import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

import csv
import math
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState


class TorqueSensing(Node):

    def __init__(self):
        super().__init__('torque_sensing')
        self.time_step = [0.0]
        self.time = time.time()
        self.set_subscribers_publishers()
        self.i = 0
        self.iter = 0.0
        #define forces
        self.fl_steer_force = [0.0]; self.fr_steer_force = [0.0]
        self.rl_steer_force = [0.0]; self.rr_steer_force = [0.0]
        self.pivot_force = [0.0]
        self.fl_wheel_force = [0.0]; self.fr_wheel_force = [0.0]
        self.rl_wheel_force = [0.0]; self.rr_wheel_force = [0.0]
        
    
    # Callbacks Section
    # STEER CALLBACKS
    def fl_steer_cb(self, msg):
        # Front Left steer 
        Mz = msg.wrench.torque.z
        self.fl_steer_force.append(Mz)
    def fr_steer_cb(self, msg):
        # Front Right steer 
        Mz = msg.wrench.torque.z
        self.fr_steer_force.append(Mz)
    def rl_steer_cb(self, msg):
        # Rear Left steer 
        Mz = msg.wrench.torque.z
        self.rl_steer_force.append(Mz)
    def rr_steer_cb(self, msg):
        # Rear Right steer 
        Mz = msg.wrench.torque.z
        self.rr_steer_force.append(Mz)
    def pivot_cb(self, msg):
        # Front Right steer 
        Mz = msg.wrench.torque.z
        self.pivot_force.append(Mz)
    # WHEEL CALLBACKS        
    def fl_wheel_cb(self, msg):
        # Front Left wheel 
        Mz = msg.wrench.torque.z
        self.fl_wheel_force.append(Mz)
    def fr_wheel_cb(self, msg):
        # Front Right wheel 
        Mz = msg.wrench.torque.z
        self.fr_wheel_force.append(Mz)
    def rl_wheel_cb(self, msg):
        # Rear Left wheel 
        Mz = msg.wrench.torque.z
        self.rl_wheel_force.append(Mz)
    def rr_wheel_cb(self, msg):
        # Rear Right wheel 
        Mz = msg.wrench.torque.z
        self.rr_wheel_force.append(Mz)
       # self.iter += 1
        #if self.iter > 1000:
         #   self.save()
          #  print('saved')
           # rclpy.shutdown()

        

    # Callbacks Section
    def save(self):
        A = [len(self.fl_steer_force), len(self.fr_steer_force),
             len(self.rl_steer_force), len(self.rr_steer_force),
             len(self.pivot_force)]  
        length_a = min(A)
        fl_steer = self.fl_steer_force[1:length_a]; fr_steer = self.fr_steer_force[1:length_a]
        rl_steer = self.rl_steer_force[1:length_a]; rr_steer = self.rr_steer_force[1:length_a]
        pivot = self.pivot_force[1:length_a]
        steer_force = np.column_stack((fl_steer, fr_steer,
                                       rl_steer, rr_steer,
                                       pivot))
        
        B = [len(self.fl_wheel_force), len(self.fr_wheel_force),
             len(self.rl_wheel_force), len(self.rr_wheel_force),
             len(self.pivot_force)]  
        length_b = min(B)
        fl_wheel = self.fl_wheel_force[1:length_b]; fr_wheel = self.fr_wheel_force[1:length_b]
        rl_wheel = self.rl_wheel_force[1:length_b]; rr_wheel = self.rr_wheel_force[1:length_b]
        wheel_force = np.column_stack((fl_wheel, fr_wheel,
                                       rl_wheel, rr_wheel))
        current_dir = os.path.dirname(os.path.realpath(__file__))
        relative_folder_path = "../results/"
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'steer_torque.txt'),
            steer_force, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'wheel_torque.txt'),
            wheel_force, fmt='%f', delimiter='\t')
        self.get_logger().info("torque saved")
                        

    def switch_cb(self, msg):
        if msg.data: 
            print('saving')
            self.save()
            print('saved')
            rclpy.shutdown()


    def set_subscribers_publishers(self):
        self.switch_sub = self.create_subscription(Bool, '/switch_status', self.switch_cb, 10)
        self.fl_wheel_sub = self.create_subscription(WrenchStamped, '/fl_wheel_sensor', self.fl_wheel_cb, 10)
        self.fr_wheel_sub = self.create_subscription(WrenchStamped, '/fr_wheel_sensor', self.fr_wheel_cb, 10)
        self.rl_wheel_sub = self.create_subscription(WrenchStamped, '/rl_wheel_sensor', self.rl_wheel_cb, 10)
        self.rr_wheel_sub = self.create_subscription(WrenchStamped, '/rr_wheel_sensor', self.rr_wheel_cb, 10)
        self.pivot_sub = self.create_subscription(WrenchStamped, '/pivot_sensor', self.pivot_cb, 10)
        self.fl_steer_sub = self.create_subscription(WrenchStamped, '/fl_steer_sensor', self.fl_steer_cb, 10)
        self.fr_steer_sub = self.create_subscription(WrenchStamped, '/fr_steer_sensor', self.fr_steer_cb, 10)
        self.rl_steer_sub = self.create_subscription(WrenchStamped, '/rl_steer_sensor', self.rl_steer_cb, 10)
        self.rr_steer_sub = self.create_subscription(WrenchStamped, '/rr_steer_sensor', self.rr_steer_cb, 10)



def main(args=None):
    rclpy.init(args=args)
    torque = TorqueSensing()
    rclpy.spin(torque)
    torque.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

import csv
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from mpc_controller.mpc_node import PathTrackingMPC as mpc


class TorqueSensing(Node):

    def __init__(self):
        super().__init__('torque_sensing')
        self.mpc = mpc()
        self.time_step = [0.0]
        self.velocity_input = Float64MultiArray()
        self.steering_input = Float64MultiArray()
        self.time = time.time()
        self.set_subscribers_publishers()
        self.i = 0
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
        self.check_run()

    def check_run(self):
        print(mpc.switch)
        if mpc.switch:
            self.save()
            rclpy.shutdown()

    

    # Callbacks Section
    def save(self):
        steer_force = np.column_stack((self.fl_steer_force,
                                       self.fr_steer_force,
                                       self.rl_steer_force,
                                       self.rr_steer_force,
                                       self.pivot_force))
        wheel_force = np.column_stack((self.fl_wheel_force,
                                       self.fr_wheel_force,
                                       self.rl_wheel_force,
                                       self.rr_wheel_force))
        current_dir = os.path.dirname(os.path.realpath(__file__))
        relative_folder_path = "../results/"
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'steer_torque.txt'),
            steer_force, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'wheel_torque.txt'),
            wheel_force, fmt='%f', delimiter='\t')
        print("saved!")
                        

      

                        

 
 #  def fr_velocity_cb(self, msg):
 #       # Front Right Wheel Velocity
        #self.torque_fl
        # .append()
  ##      index = msg.name.index("front_right_wheel_joint")
     #   speed = msg.velocity[index-1]
    #    self.speed_fr.append(speed)
     #   print(speed)

    # End: Callbacks Section

    def set_subscribers_publishers(self):
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
    dmpc = TorqueSensing()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

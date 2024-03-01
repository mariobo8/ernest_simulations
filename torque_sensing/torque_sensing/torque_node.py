#!/usr/bin/env python3
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


class TorqueSensing(Node):

    def __init__(self):
        super().__init__('torque_sensing')

        self.time_step = [0.0]
        self.force_fr = [0.0]
        self.velocity_input = Float64MultiArray()
        self.steering_input = Float64MultiArray()
        self.switch = False
        self.time = time.time()
        self.set_subscribers_publishers()
        self.i = 0
        self.speed_fr = [0.0]
        self.force_n = [0.0]
        
    # Callbacks Section
    def fr_velocity_cb(self, msg):

        #self.torque_fl
        # .append()
        index = msg.name.index("front_right_wheel_joint")
        speed = msg.velocity[index-1]
        self.speed_fr.append(speed)
        print(speed)

    # Callbacks Section
    def fl_wheel_cb(self, msg):
        return
    
    def steer_cb(self, msg):
        force_n = msg.wrench.force.z
        self.force_n.append(force_n)



    # Callbacks Section
    def fr_wheel_cb(self, msg):

        force_fr = msg.wrench.force.z
        self.force_fr.append(force_fr)


        self.velocity_input.data = [1/0.14, 1/0.14]
        self.steering_input.data = [-1.0, -1.0, 0.0, 0.0, 0.0]
        self.vel_pub.publish(self.velocity_input)
        self.steer_pub.publish(self.steering_input)
        print("publishing %f" %self.i)
        self.i = self.i + 1
        if self.i*0.001 > 1/0.14: 
            np.savetxt('fr_force.txt',
                self.force_fr, fmt='%f', delimiter='\t')
            np.savetxt('n_force.txt',
                self.force_n, fmt='%f', delimiter='\t')
            np.savetxt('fr_vel.txt',
                self.speed_fr, fmt='%f', delimiter='\t')
                 
            print("saved!")
            self.velocity_input.data = [0.0, 0.0]
            self.steering_input.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.vel_pub.publish(self.velocity_input)
            self.steer_pub.publish(self.steering_input)
            rclpy.shutdown()
                        
        
    # Callbacks Section
    def rl_wheel_cb(self, msg):
        #self.torque_fl.append()
        torque_rl = msg.wrench.torque.x


        
    def rr_wheel_cb(self, msg):

        #self.torque_fl.append()
        torque_rr = msg.wrench.torque.x
                        



        
    def pivot_cb(self, msg):

        #self.torque_fl.append()
        torque_pivot = msg.wrench.torque.x
                        
        #print(torque_pivot)#, torque_fr, torque_rl, torque_rr)
       # if self.switch: 
        #    np.savetxt('fwsinput.txt',
         #       self.input_sequence, fmt='%f', delimiter='\t')
         #   print("saved!")

 


    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.fl_wheel_sub = self.create_subscription(WrenchStamped, '/fl_wheel_sensor', self.fl_wheel_cb, 10)
        self.fr_wheel_sub = self.create_subscription(WrenchStamped, '/fr_wheel_sensor', self.fr_wheel_cb, 10)
        self.rl_wheel_sub = self.create_subscription(WrenchStamped, '/rl_wheel_sensor', self.rl_wheel_cb, 10)
        self.rr_wheel_sub = self.create_subscription(WrenchStamped, '/rr_wheel_sensor', self.rr_wheel_cb, 10)
        self.fl_velociy_sub = self.create_subscription(JointState, '/joint_states', self.fr_velocity_cb, 10)
        self.pivot_sub = self.create_subscription(WrenchStamped, '/pivot_sensor', self.pivot_cb, 10)
        self.steer_sub = self.create_subscription(WrenchStamped, '/steer_sensor', self.steer_cb, 10)
        
        
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

   
    

def main(args=None):
    rclpy.init(args=args)
    dmpc = TorqueSensing()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node

from .ackermann_kinematics import AckermannKinematics as demo_mpc
import math
from .utils import *
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class PathTrackingMPC(Node):

    def __init__(self):
        super().__init__('path_tracking_MPC')
        self.mpc_inst = demo_mpc()
        self.N = 40
        self.rate = self.create_rate(10)
        self.start = True
        self.state = np.zeros((3, 1))
        self.xp0 = ca.DM([-5, 0, 0, 0])
        self.up0 =ca.DM([0, 0, 0])


        [self.solver, self.args, self.n_states, self.n_controls, self.f, self.f2] \
            = self.mpc_inst.set_solver()
        self.set_subscribers_publishers()
        

    # Callbacks Section
    def pose_sub_cb(self, msg):
        start_time = time.time()

        self.pose_ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        x = msg.pose.pose.position.x - 5
        y = msg.pose.pose.position.y 
        
        [psi, _, _] = self.quat_2_eul(msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w)
        
        
        self.state = np.array([[x],[y],[psi], [0]])
        
        #self.get_logger().info("input received and dio %f" % self.state[0])
        [velocity_input, steering_input] = self.create_control_message()

        end_time = time.time()
        elapsed_time = end_time - start_time
        print("time step %f" % elapsed_time)
        if elapsed_time > 0.1:time.sleep(0.0)
        else: time.sleep(0.1 - elapsed_time)
        self.vel_pub.publish(velocity_input)
        self.steer_pub.publish(steering_input)

 


    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(Odometry, '/p3d/odom', self.pose_sub_cb, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

    def create_control_message(self):
        [input, new_xp0, new_up0] = self.mpc_inst.solve_mpc(self.solver ,self.state , self.args, self.n_states, self.n_controls, self.xp0, self.up0, self.f2)
        #print("state")
        #print(self.state)  
        #print("reference state")    
        #print(self.xp0)
        #print("reference")
        #print(new_xp0)
        #print(self.mpc_inst.xp0)
        self.xp0 = new_xp0
        self.up0 = new_up0
        #print(new_xp0)
        #self.get_logger().info("speed %f" % input[0])
        #self.get_logger().info("steer %f" % input[1])
        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        input_1 = input[0]/0.15
        input_2 = input[1]/0.15
        
        velocity_input.data = [float(input_1), float(input_1), float(input_2), float(input_2)]
        steering_input.data = [float(-input[2]), float(-input[2]), float(-input[3]), float(-input[3]), float(-input[4])]
        
        return velocity_input, steering_input
    
    
  #  def run(self):
  #      
  #      [velocity_input, steering_input] = self.create_control_message(self.state)
  #      self.get_logger().info("sto a pubblica")
  #      self.vel_pub.publish(velocity_input)
  #      self.steer_pub.publish(steering_input)
  #      self.get_logger().info("ho pubblicato")
 #       #time.sleep( )



def main(args=None):
    rclpy.init(args=args)
    dmpc = PathTrackingMPC()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

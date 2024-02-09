#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node

from .path_tracking_mpc_node import mpc as demo_mpc
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class PathTrackingMPC(Node):

    def __init__(self):
        super().__init__('path_tracking_MPC')

        # Initialize basic parameters
        #self.dt = 10
        self.rate = self.create_rate(10)
        self.start = True
        self.state = np.zeros((3, 1))
        #self.t0 = 0.0
        #self.twist = None
        #self.pose = None

        # Data timestamps and validity 
        #self.ts_threshold = 1.0
        #self.pose_ts = 0.0
        #self.twist_ts = 0.0

        # Set publishers and subscribers

        self.set_subscribers_publishers()
        
        # Change onboard timeout
        #new_timeout = Float64Srv.Request()
        #new_timeout.data = 1.5
        #ans = self.pmc_timeout(new_timeout)
        #if not ans.success:
        #    self.get_logger().error("Couldn't change PMC timeout.")
        #    exit()
        #else:
        #    self.get_logger().info("Timeout updated.")
    def quat_2_eul(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    # Callbacks Section
    def pose_sub_cb(self, msg):
        
        self.pose_ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        [_, _, psi] = self.quat_2_eul(msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w)
        self.state = np.array([[x],[y],[psi]])
        #self.get_logger().info("input received and dio %f" % self.state[0])
        [velocity_input, steering_input] = self.create_control_message(self.state)
        self.get_logger().info("sto a pubblica")
        self.vel_pub.publish(velocity_input)
        self.steer_pub.publish(steering_input)
        self.get_logger().info("ho pubblicato")
        time.sleep(0.1)


    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(Odometry, '/p3d/odom', self.pose_sub_cb, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

    def create_control_message(self, state):
        mpc_inst = demo_mpc()
        self.get_logger().info("sto anna arisolve")
        input = mpc_inst.mpc_solve(state)
        self.get_logger().info("ho risolto %f" % input[0])
        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        velocity_input.data = [float(input[0]), float(input[0])]
        steering_input.data = [float(input[1]), float(input[1]), 0.0, 0.0, 0.0]
        
        return velocity_input, steering_input
    
    
  #  def run(self):
  #      
  #      [velocity_input, steering_input] = self.create_control_message(self.state)
  #      self.get_logger().info("sto a pubblica")
  #      self.vel_pub.publish(velocity_input)
  #      self.steer_pub.publish(steering_input)
  #      self.get_logger().info("ho pubblicato")
 #       #time.sleep(0.1)



def main(args=None):
    rclpy.init(args=args)
    dmpc = PathTrackingMPC()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

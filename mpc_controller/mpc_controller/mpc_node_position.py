import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

import csv
from .pivot_4ws_kinematics import Pivot4wsKinematics as pfws_mpc
from .pivot_kinematics import Pivot4wsKinematics as p_mpc
from .fws_kinematics import fwsKinematics as fws_mpc
from .sym_fws_kinematics import fwsKinematics as sym_fws_mpc
from .ackermann_kinematics import AckermannKinematics as ack_mpc

import math
from .utils import *
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class PathTrackingMPC(Node):

    def __init__(self):
        super().__init__('path_tracking_MPC')
        self.mpc_inst = pfws_mpc()
        self.xp_0 = self.mpc_inst.start
        self.alpha_0 = 0.0 
        self.input_sequence = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.state_sequence = [self.xp_0, 0.0, 0.0, 0.0]
        self.time_step = [0.0]
        self.b = 0.4
        self.switch = False
        [self.solver, self.args, self.n_states, self.n_controls, self.f] \
            = self.mpc_inst.set_solver()
        self.set_subscribers_publishers()
    
    def save_data(self):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        relative_folder_path = "../results/position_controller/4ws"
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'input.txt'),
            self.input_sequence, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'state.txt'),
            self.state_sequence, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'time_step.txt'),
            self.time_step, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'reference.txt'),
            self.mpc_inst.ref, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'prediction.txt'),
            self.mpc_inst.pred, fmt='%f', delimiter='\t')
        print("saved!")


    # Callbacks Section
    def pose_sub_cb(self, msg):
        start_time = time.time()

        #self.pose_ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        x = msg.pose.pose.position.x + self.xp_0
        y = msg.pose.pose.position.y 
        
        [psi, _, _] = quat_2_eul(msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z,
                                 msg.pose.pose.orientation.w)
        
        
        self.mpc_inst.x0 = np.array([float(x),float(y),float(psi), float(0.0)])
        [self.velocity_input, self.steering_input] = self.create_control_message()

        end_time = time.time()
        elapsed_time = end_time - start_time
        print("time step %f" % elapsed_time)
        if elapsed_time > 0.1:time.sleep(0.0)
        else: time.sleep(0.1 - elapsed_time)
        self.time_step.append(elapsed_time)
        self.vel_pub.publish(self.velocity_input)
        self.steer_pub.publish(self.steering_input)
        if self.switch: 
            self.save_data()
            time.sleep(3)
            #rclpy.shutdown()

 
    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_sub_cb, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

    def create_control_message(self):
        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        if self.mpc_inst.x0[0] >= -0.02:
            velocity_input.data = [0.0, 0.0, 0.0, 0.0]
            steering_input.data = [0.0, 0.0, 0.0, 0.0]#, 0.0] 
            self.switch = True            
            
        else:
            [input, new_xp0, new_up0] = self.mpc_inst.solve_mpc(self.solver, 
                                        self.mpc_inst.x0 , self.args, self.n_states, 
                                        self.n_controls, self.mpc_inst.x_r, self.mpc_inst.up0)
            self.mpc_inst.x_r = new_xp0
            self.mpc_inst.up0 = new_up0
            #alpha_dot = input[4] - self.alpha_0
            #self.alpha_0 = input[4]

            self.input_sequence = np.vstack([self.input_sequence, input])
            self.state_sequence = np.vstack([self.state_sequence, self.mpc_inst.x0])
            
            velocity_input.data = [input[0]/0.15, input[1]/0.15, input[2]/0.15, input[3]/0.15] 
            print(velocity_input.data)
            steering_input.data = [-input[4], -input[5], -input[6], -input[7]]#, -input[8]]
        return velocity_input, steering_input
    

def main(args=None):
    rclpy.init(args=args)
    dmpc = PathTrackingMPC()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

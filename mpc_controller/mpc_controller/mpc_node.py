import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

import csv

from .models.pivot_4ws_kinematics import Pivot4wsKinematics as pfws_mpc
from .models.pivot_kinematics import Pivot4wsKinematics as p_mpc
from .models.pivot_kinematics_new_approach import Pivot4wsKinematics as np_mpc
from .models.fws_kinematics import fwsKinematics as fws_mpc
from .models.sym_fws_kinematics import fwsKinematics as s_fws_mpc
from .models.ackermann_kinematics import AckermannKinematics as ack_mpc

import math
from .utils.utils import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Odometry


class PathTrackingMPC(Node):

    def __init__(self):
        super().__init__('path_tracking_MPC')
        self.mpc_inst = pfws_mpc()
        self.xp_0 = self.mpc_inst.start
        self.alpha_0 = 0.0 
        self.input_sequence = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.input_bicycle = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.state_sequence = [self.xp_0, 0.0, 0.0, 0.0]
        self.time_step = [0.0]
        self.b = 0.4
        self.switch = False
        [self.solver, self.args, self.n_states, self.n_controls, self.f] \
            = self.mpc_inst.set_solver()
        self.set_subscribers_publishers()

    
    def save_data(self):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        relative_folder_path = "../results/5_dof/passive"
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'input.txt'),
            self.input_sequence, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'input_bicycle.txt'),
            self.input_bicycle, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'state.txt'),
            self.state_sequence, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'time_step.txt'),
            self.time_step, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'reference.txt'),
            self.mpc_inst.ref, fmt='%f', delimiter='\t')
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'prediction.txt'),
            self.mpc_inst.pred, fmt='%f', delimiter='\t')
        self.get_logger().info("saved")


    def pose_sub_cb(self, msg):
        start_time = time.time()
        #self.get_logger().info("going to solve")

        #self.pose_ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        x = msg.pose.pose.position.x + self.xp_0
        y = msg.pose.pose.position.y 
        
        [psi, _, _] = quat_2_eul(msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z,
                                 msg.pose.pose.orientation.w)
        
        
        self.mpc_inst.x0 = np.array([float(x),float(y),float(psi), float(0.0)])
        [self.velocity_input, self.steering_input] = self.create_control_message()
        #self.velocity_input = Float64MultiArray()
        #self.steering_input = Float64MultiArray()
        #self.velocity_input.data = [0.0, 0.0, 0.0, 0.0]
        #self.steering_input.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        #self.get_logger().info("solved!")
        end_time = time.time()
        elapsed_time = end_time - start_time
        self.get_logger().info("time step %f" % elapsed_time)
        if elapsed_time > 0.1:time.sleep(0.0)
        else: time.sleep(0.2 - elapsed_time)
        self.time_step.append(elapsed_time)
        print("Publishing ref")
        self.ref_vel_pub.publish(self.velocity_input)
        self.ref_steer_pub.publish(self.steering_input)
        #publish the status of the switch on a topic
        self.switch_pub.publish(Bool(data=self.switch))

        if self.switch: 
            self.get_logger().info("saving")
            self.save_data()
            time.sleep(3)
            rclpy.shutdown()
        


    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_sub_cb, 10)
        self.ref_vel_pub = self.create_publisher(Float64MultiArray, '/ref_vel_commands',10)
        self.ref_steer_pub = self.create_publisher(Float64MultiArray, '/ref_steer_commands',10)
        self.switch_pub = self.create_publisher(Bool, '/switch_status',10)


    def create_control_message(self):
        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        if self.mpc_inst.x0[0] >= -0.02:
            velocity_input.data = [0.0, 0.0, 0.0, 0.0]
            steering_input.data = [0.0, 0.0, 0.0, 0.0, 0.0] 
            self.switch = True            
            
        else:
            [input, new_xp0, new_up0, bicycle] = self.mpc_inst.solve_mpc(self.solver, 
                                        self.mpc_inst.x0 , self.args, self.n_states, 
                                        self.n_controls, self.mpc_inst.x_r, self.mpc_inst.up0)
            self.mpc_inst.x_r = new_xp0
            self.mpc_inst.up0 = new_up0
            #alpha_dot = input[4] - self.alpha_0
            #self.alpha_0 = input[4]

            self.input_bicycle = np.vstack([self.input_bicycle, bicycle])
            self.input_sequence = np.vstack([self.input_sequence, input])
            self.state_sequence = np.vstack([self.state_sequence, self.mpc_inst.x0])
            
            velocity_input.data = [input[0]/0.15, input[1]/0.15, input[2]/0.15, input[3]/0.15] 
            print(velocity_input.data)
            steering_input.data = [-input[4], -input[5], -input[6], -input[7], -input[8]]
            #self.get_logger().info("ble ble %s" % str(steering_input.data))

        return velocity_input, steering_input
    

def main(args=None):
    rclpy.init(args=args)
    mpc = PathTrackingMPC()
    rclpy.spin(mpc)
    mpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

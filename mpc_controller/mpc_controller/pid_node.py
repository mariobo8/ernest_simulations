import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np



class PidController(Node):

    def __init__(self):        
        super().__init__('path_tracking_MPC')
        self.vel = Float64MultiArray()
        self.steer = Float64MultiArray()
        self.set_subscribers_publishers()
        self.steer_pos = [0.0]
        self.drive = 0.0
        self.ref_vel = [0.0, 0.0, 0.0, 0.0]
        self.ref_steer = [0.0, 0.0, 0.0, 0.0, 0.0]
        #self.ref_steer = [0.0, 0.0, 0.0, 0.0, 0.0]
        #pid stuff
        self.iter = 0
        self.fl_e_prev = 0; self.fl_e_tot = 0; self.prev_fl = 0
        self.fr_e_prev = 0; self.fr_e_tot = 0; self.prev_fr = 0
        self.rl_e_prev = 0; self.rl_e_tot = 0; self.prev_rl = 0
        self.rr_e_prev = 0; self.rr_e_tot = 0; self.prev_rr = 0
        #wheel velocity
        self.kpv = 10.1; self.kdv = 0.0; self.kiv = 0.0
        self.dt = 0.001

        self.s_fl_e_prev = 0; self.s_fl_e_tot = 0; self.prev_s_fl = 0 
        self.s_fr_e_prev = 0; self.s_fr_e_tot = 0; self.prev_s_fr = 0
        self.s_rl_e_prev = 0; self.s_rl_e_tot = 0; self.prev_s_rl = 0
        self.s_rr_e_prev = 0; self.s_rr_e_tot = 0; self.prev_s_rr = 0
        self.pivot_e_prev = 0; self.pivot_e_tot = 0; self.prev_pivot = 0
        #fl_steering_wheel
        self.kps_fl = 35; self.kds_fl = 1; self.kis_fl = 0.0
        #fr_steering_wheel
        self.kps_fr = 20; self.kds_fr = 0.8; self.kis_fr = 0.0
        #rl_steering_wheel
        self.kps_rl = 60; self.kds_rl = 4; self.kis_rl = 0.0    
        #rr_steering_wheel
        self.kps_rr = 22.0; self.kds_rr = 0.9; self.kis_rr = 0.0
        #pivot_joint
        self.kpp = 4000; self.kdp = 150; self.kip = 0.0

        

    def ref_vel_sub_cb(self, msg):
        self.ref_vel = msg.data
        return
    def ref_steer_sub_cb(self, msg):
        self.ref_steer = msg.data
        return

    def fb_sub_cb(self, msg):

        #e_vel = Float64MultiArray(); e_steer = Float64MultiArray()
        [self.vel, self.steer] = self.order_joints(msg)
        
        [torque_vel, torque_steer] = self.pid_controller()
    

        #torque_vel.data = ([0.0, 0.0, 0.0, 0.0])
        #torque_steer.data = ([0.0,0.0, 0.0, 0.0, 0.0])
        #self.get_logger().info("Publishing command")
        self.pub_wheel.publish(torque_vel)
        self.pub_steer.publish(torque_steer)
        self.iter += 1
        #if self.iter > 2000:
        #    self.save() 
        #    print("saved")
        #    rclpy.shutdown()

    def save(self):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        relative_folder_path = "../results/pid_calibration"
        np.savetxt(os.path.join(current_dir, relative_folder_path, 'step.txt'),
                   self.steer_pos, fmt='%f')
        
    def pid_controller(self):
        torque_steer = Float64MultiArray(); torque_vel = Float64MultiArray()

        
        [fl_wheel, self.fl_e_prev, self.fl_e_tot] = self.pid_set(self.ref_vel[0], self.vel.data[0],
                                                                 self.kpv, self.kdv, self.kiv, self.fl_e_prev, self.fl_e_tot, self.prev_fl)
        self.prev_fl = fl_wheel
        [fr_wheel, self.fr_e_prev, self.fr_e_tot] = self.pid_set(self.ref_vel[1], self.vel.data[1], 
                                                                 self.kpv, self.kdv, self.kiv, self.fr_e_prev, self.fr_e_tot, self.prev_fr)
        self.prev_fr = fr_wheel
        [rl_wheel, self.rl_e_prev, self.rl_e_tot] = self.pid_set(self.ref_vel[2], self.vel.data[2], 
                                                                 self.kpv, self.kdv, self.kiv, self.rl_e_prev, self.rl_e_tot, self.prev_rl)
        self.prev_rl = rl_wheel
        [rr_wheel, self.rr_e_prev, self.rr_e_tot] = self.pid_set(self.ref_vel[3], self.vel.data[3],
                                                                 self.kpv, self.kdv, self.kiv, self.rr_e_prev, self.rr_e_tot, self.prev_rr)
        self.prev_rr = rr_wheel
        
        torque_vel.data = [fl_wheel, fr_wheel, rl_wheel, rr_wheel]

        [fl_steer, self.s_fl_e_prev, self.s_fl_e_tot] = self.pid_set(self.ref_steer[0], self.steer.data[0],
                                                                 self.kps_fl, self.kds_fl, self.kis_fl, self.s_fl_e_prev, self.s_fl_e_tot, self.prev_s_fl)
        self.prev_s_fl = fl_steer
        [fr_steer, self.s_fr_e_prev, self.s_fr_e_tot] = self.pid_set(self.ref_steer[1], self.steer.data[1], 
                                                                 self.kps_fr, self.kds_fr, self.kis_fr, self.s_fr_e_prev, self.s_fr_e_tot, self.prev_s_fr)
        self.prev_s_fr = fr_steer
        [rl_steer, self.s_rl_e_prev, self.s_rl_e_tot] = self.pid_set(self.ref_steer[2], self.steer.data[2], 
                                                                 self.kps_rl, self.kds_rl, self.kis_rl, self.s_rl_e_prev, self.s_rl_e_tot, self.prev_s_rl)
        self.prev_s_rl = rl_steer
        [rr_steer, self.s_rr_e_prev, self.s_rr_e_tot] = self.pid_set(self.ref_steer[3], self.steer.data[3],
                                                                 self.kps_rr, self.kds_rr, self.kis_rr, self.s_rr_e_prev, self.s_rr_e_tot, self.prev_s_rr)
        self.prev_s_rr = rr_steer
        [pivot, self.pivot_e_prev, self.pivot_e_tot] = self.pid_set(self.ref_steer[4], self.steer.data[4],
                                                                 self.kpp, self.kdp, self.kip, self.pivot_e_prev, self.pivot_e_tot, self.prev_pivot)
        self.prev_pivot = pivot
        self.steer_pos.append(self.steer.data[4])
        torque_steer.data = [fl_steer, fr_steer, rl_steer, rr_steer, pivot]
        #print(torque_steer.data)
        #torque_vel.data = ([15.0, 15.0, 15.0, 15.0])
        #print(torque_vel.data)
        #torque_steer.data = ([0.0, 0.0, 0.0, 0.0, pivot])
        return torque_vel, torque_steer

    def pid_set(self, ref, value, kp, kd, ki, e_prev, e_tot, prev_control):
        e_0 = ref - value
        control = kp*e_0 + kd*(e_0 - e_prev) / self.dt + ki*(e_0 + e_tot)
        if control > prev_control + 5.0: control = prev_control + 5.0
        elif control < prev_control - 5.0: control = prev_control - 5.0
        if control > 100: control = 100.0
        elif control < -100: control = -100.0        
        e_prev = e_0; e_tot += e_0  

        return control, e_prev, e_tot


    def order_joints(self,msg):
        fb_vel = Float64MultiArray()
        fb_steer = Float64MultiArray()
        #velocities
        index = msg.name.index("front_left_wheel_joint")
        fl_wheel = msg.velocity[index]
        #fl_wheel_torque = msg.effrot[index]  
     
        index = msg.name.index("front_right_wheel_joint")
        fr_wheel = msg.velocity[index] 
        #fr_wheel_torque = msg.effrot[index]

        index = msg.name.index("rear_left_wheel_joint")
        rl_wheel = msg.velocity[index]
        #rl_wheel_torque = msg.effrot[index]   
      
        index = msg.name.index("rear_right_wheel_joint")
        rr_wheel = msg.velocity[index]
        #rr_wheel_torque = msg.effrot[index] 

        #positions
        index = msg.name.index("front_left_steer_joint")
        fl_steer = msg.position[index] 
        #rl_steer_torque = msg.effrot[index]       
        index = msg.name.index("front_right_steer_joint")
        fr_steer = msg.position[index]
        #rl_steer_torque = msg.effrot[index] 
        index = msg.name.index("rear_left_steer_joint")
        rl_steer = msg.position[index]
        #rl_steer_torque = msg.effrot[index]         
        index = msg.name.index("rear_right_steer_joint")
        rr_steer = msg.position[index]
        #rl_steer_torque = msg.effrot[index]  
        index = msg.name.index("pivot_joint")
        pivot = msg.position[index] 
        fb_vel.data = [fl_wheel, fr_wheel, rl_wheel, 
                          rr_wheel]
        fb_steer.data = [fl_steer, fr_steer, 
                           rl_steer, rr_steer, pivot]
        
        return fb_vel, fb_steer
    def set_subscribers_publishers(self):
        self.fb_sub = self.create_subscription(JointState,'/joint_states',self.fb_sub_cb,10)
        self.ref_steer_sub = self.create_subscription(
                            Float64MultiArray, '/ref_vel_commands', self.ref_vel_sub_cb, 10)
        self.ref_vel_sub = self.create_subscription(
                            Float64MultiArray, '/ref_steer_commands', self.ref_steer_sub_cb, 10)
        self.pub_wheel = self.create_publisher(Float64MultiArray, '/wheel_controller/commands',10)
        self.pub_steer = self.create_publisher(Float64MultiArray, '/steer_controller/commands',10)

def main(args=None):
    rclpy.init(args=args)
    pid = PidController()
    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

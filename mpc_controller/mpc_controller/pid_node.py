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
        self.err = 0.0
        self.drive = 0.0
        self.ref_vel = [0.0, 0.0, 0.0, 0.0]
        self.ref_steer = [0.0, 0.0, 0.0, 0.0, 0.0]

        #pid stuff
        self.fl_e_prev = 0; self.fl_e_tot = 0
        self.fr_e_prev = 0; self.fr_e_tot = 0
        self.rl_e_prev = 0; self.rl_e_tot = 0
        self.rr_e_prev = 0; self.rr_e_tot = 0
        self.kpv = 10.1; self.kdv = 0.0; self.kiv = 0.0
        self.dt = 0.001

        self.s_fl_e_prev = 0; self.s_fl_e_tot = 0
        self.s_fr_e_prev = 0; self.s_fr_e_tot = 0
        self.s_rl_e_prev = 0; self.s_rl_e_tot = 0
        self.s_rr_e_prev = 0; self.s_rr_e_tot = 0
        self.pivot_e_prev = 0; self.pivot_e_tot = 0
        self.kps = 10; self.kds = 0.001; self.kis = 0.00000
        self.kpp = 1000

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
        #torque_steer.data = ([0.0, 0.0, 0.0, 0.0, 0.0])
        #self.get_logger().info("Publishing command")
        self.pub_wheel.publish(torque_vel)
        self.pub_steer.publish(torque_steer)

    def pid_controller(self):
        kps = 0.15
        kpp = 1.1
        torque_steer = Float64MultiArray(); torque_vel = Float64MultiArray()

        
        [fl_wheel, self.fl_e_prev, self.fl_e_tot] = self.pid_set(self.ref_vel[0], self.vel.data[0],
                                                                 self.kpv, self.kdv, self.kiv, self.fl_e_prev, self.fl_e_tot)
        
        [fr_wheel, self.fr_e_prev, self.fr_e_tot] = self.pid_set(self.ref_vel[1], self.vel.data[1], 
                                                                 self.kpv, self.kdv, self.kiv, self.fr_e_prev, self.fr_e_tot)
        [rl_wheel, self.rl_e_prev, self.rl_e_tot] = self.pid_set(self.ref_vel[2], self.vel.data[2], 
                                                                 self.kpv, self.kdv, self.kiv, self.rl_e_prev, self.rl_e_tot)
        [rr_wheel, self.rr_e_prev, self.rr_e_tot] = self.pid_set(self.ref_vel[3], self.vel.data[3],
                                                                 self.kpv, self.kdv, self.kiv, self.rr_e_prev, self.rr_e_tot)
        
        torque_vel.data = [fl_wheel, fr_wheel, rl_wheel, rr_wheel]

        [fl_steer, self.s_fl_e_prev, self.s_fl_e_tot] = self.pid_set(self.ref_steer[0], self.steer.data[0],
                                                                 self.kps, self.kds, self.kis, self.s_fl_e_prev, self.s_fl_e_tot)
        print(self.ref_steer[0], self.steer.data[0])
        print(fl_steer)
        [fr_steer, self.s_fr_e_prev, self.s_fr_e_tot] = self.pid_set(self.ref_steer[1], self.steer.data[1], 
                                                                 self.kps, self.kds, self.kis, self.fr_e_prev, self.s_fr_e_tot)
        [rl_steer, self.s_rl_e_prev, self.s_rl_e_tot] = self.pid_set(self.ref_steer[2], self.steer.data[2], 
                                                                 self.kps, self.kds, self.kis, self.s_rl_e_prev, self.s_rl_e_tot)
        [rr_steer, self.s_rr_e_prev, self.s_rr_e_tot] = self.pid_set(self.ref_steer[3], self.steer.data[3],
                                                                 self.kps, self.kds, self.kis, self.s_rr_e_prev, self.s_rr_e_tot)
        [pivot, self.pivot_e_prev, self.pivot_e_tot] = self.pid_set(self.ref_steer[4], self.steer.data[4],
                                                                 self.kpp, self.kds, self.kis, self.pivot_e_prev, self.pivot_e_tot)
        
        torque_steer.data = [fl_steer, fr_steer, rl_steer, rr_steer, pivot]
        #print(torque_steer.data)
        #torque_vel.data = ([15.0, 15.0, 15.0, 15.0])
        #print(torque_vel.data)
        #torque_steer.data = ([0.0, 0.0, 0.0, 0.0, 0.0])
        return torque_vel, torque_steer

    def pid_set(self, ref, value, kp, kd, ki, e_prev, e_tot):
        e_0 = ref - value
        control = kp*e_0 + kd*(e_0 - e_prev) / self.dt + ki*(e_0 + e_tot)
        if control > 50: control = 50.0
        e_prev = e_0; e_tot += e_0  
        return control, e_prev, e_tot


    def order_joints(self,msg):
        fb_vel = Float64MultiArray()
        fb_steer = Float64MultiArray()
        #velocities
        index = msg.name.index("front_left_wheel_joint")
        fl_wheel = msg.velocity[index]           
        index = msg.name.index("front_right_wheel_joint")
        fr_wheel = msg.velocity[index] 
        index = msg.name.index("rear_left_wheel_joint")
        rl_wheel = msg.velocity[index]          
        index = msg.name.index("rear_right_wheel_joint")
        rr_wheel = msg.velocity[index] 
        #positions
        index = msg.name.index("front_left_steer_joint")
        fl_steer = msg.position[index]           
        index = msg.name.index("front_right_steer_joint")
        fr_steer = msg.position[index] 
        index = msg.name.index("rear_left_steer_joint")
        rl_steer = msg.position[index]           
        index = msg.name.index("rear_right_steer_joint")
        rr_steer = msg.position[index] 
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

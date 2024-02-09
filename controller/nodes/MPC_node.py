#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import SetBool as SetBoolSrv
from std_srvs.srv import Float64 as Float64Srv


class SimpleControlExample(Node):

    def __init__(self):
        super().__init__('path_tracking_MPC')

        # Initialize basic parameters
        self.dt = 1
        self.rate = self.create_rate(5)
        self.start = False
        self.state = np.zeros((13, 1))
        self.state[9] = 1
        self.t0 = 0.0
        self.twist = None
        self.pose = None

        # Data timestamps and validity threshold
        self.ts_threshold = 1.0
        self.pose_ts = 0.0
        self.twist_ts = 0.0

        # Set publishers and subscribers
        self.set_services()
        self.set_subscribers_publishers()

        # Change onboard timeout
        new_timeout = Float64Srv.Request()
        new_timeout.data = 1.5
        ans = self.pmc_timeout(new_timeout)
        if not ans.success:
            self.get_logger().error("Couldn't change PMC timeout.")
            exit()
        else:
            self.get_logger().info("Timeout updated.")

        self.run()

    # Callbacks Section
    def pose_sub_cb(self, msg=PoseStamped()):
        self.pose_ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.pose = np.array([[msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w]]).T
        self.state[0:3] = self.pose[0:3]
        self.state[6:10] = self.pose[3:7]

    def twist_sub_cb(self, msg=TwistStamped()):
        self.twist_ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.twist = np.array([[msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z,
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z]]).T
        self.state[3:6] = self.twist[0:3]
        self.state[10:13] = self.twist[3:6]

    def start_srv_callback(self, req=SetBoolSrv.Request()):
        state = req.data
        ans = SetBoolSrv.Response()
        if state:
            ans.success = True
            ans.message = "Node started!"

            self.t0 = self.get_clock().now().to_msg().sec
            obc = SetBoolSrv.Request()
            obc.data = False
            self.onboard_ctl(obc)
            self.start = True
        else:
            ans.success = True
            ans.message = "Node stopped!"
            obc = SetBoolSrv.Request()
            obc.data = True
            self.onboard_ctl(obc)
            self.start = False

        return ans

    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(PoseStamped, 'pose_topic', self.pose_sub_cb, 10)
        self.twist_sub = self.create_subscription(TwistStamped, 'twist_topic', self.twist_sub_cb, 10)

        self.control_pub = self.create_publisher(FamCommand, 'control_topic', 1)
        self.flight_mode_pub = self.create_publisher(FlightMode, 'flight_mode', 1)

    def set_services(self):
        self.onboard_ctl = self.create_client(SetBoolSrv, 'onboard_ctl_enable_srv')
        self.pmc_timeout = self.create_client(SetFloat, 'pmc_timeout_srv')

        self.start_service = self.create_service(SetBoolSrv, 'start_srv', self.start_srv_callback)

        while not self.onboard_ctl.wait_for_service(timeout_sec=1.0) or not self.pmc_timeout.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            continue

    def check_data_validity(self):
        pos_val = False
        vel_val = False

        if time.time() - self.pose_ts < self.ts_threshold:
            pos_val = True
        if time.time() - self.twist_ts < self.ts_threshold:
            vel_val = True

        if not pos_val or not vel_val:
            self.get_logger().warn(f"Skipping control. Validity flags:\nPos: {pos_val}; Vel: {vel_val}")

        return pos_val and vel_val

    def create_control_message(self):
        u = FamCommand()

        u.header.frame_id = 'body'
        u.header.stamp = self.get_clock().now().to_msg()

        u.wrench.force.x = self.u_traj[0]
        u.wrench.force.y = self.u_traj[1]
        u.wrench.force.z = self.u_traj[2]
        u.wrench.torque.x = self.u_traj[3]
        u.wrench.torque.y = self.u_traj[4]
        u.wrench.torque.z = self.u_traj[5]

        u.status = 3
        u.control_mode = 2

        return u

    def create_flight_mode_message(self):
        fm = FlightMode()

        fm.name = "difficult"
        fm.control_enabled = False
        fm.att_ki = Vec(0.002, 0.002, 0.002)
        fm.att_kp = Vec(4.0, 4.0, 4.0)
        fm.omega_kd = Vec(3.2, 3.2, 3.2)

        fm.pos_kp = Vec(.6, .6, .6)
        fm.pos_ki = Vec(0.0001, 0.0001, 0.0001)
        fm.vel_kd = Vec(1.2, 1.2, 1.2)

        fm.speed = 3

        fm.tolerance_pos = 0.2
        fm.tolerance_vel = 0
        fm.tolerance_att = 0.3490
        fm.tolerance_omega = 0
        fm.tolerance_time = 1.0

        fm.hard_limit_accel = 0.0200
        fm.hard_limit_omega = 0.5236
        fm.hard_limit_alpha = 0.2500
        fm.hard_limit_vel = 0.4000

        return fm

    def run(self):
        while rclpy.ok():
            if not self.start:
                self.rate.sleep()
                self.get_logger().info("Sleeping...")
                continue

            t = time.time() - self.t0
            val = self.check_data_validity()
            if not val:
                self.rate.sleep()
                continue

            tin = time.time()
            self.u_traj = np.zeros((6, ))
            tout = time.time() - tin
            self.get_logger().info(f"Time for control: {tout}")

            u = self.create_control_message()
            fm = self.create_flight_mode_message()

            self.control_pub.publish(u)
            self.flight_mode_pub.publish(fm)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    dmpc = SimpleControlExample()
    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
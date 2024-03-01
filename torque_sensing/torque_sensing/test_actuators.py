import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class StopMotors(Node):

    def __init__(self):
        super().__init__('stop_motors')

        self.set_subscribers_publishers()
        self.iter = 0


    # Callbacks Section
    def pose_sub_cb(self, msg):


        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        velocity_input.data = [3.0, 3.0, 10.0, 10.0]
        steering_input.data = [1.5, 1.5, 0.0, 0.0, 0.0]
        self.vel_pub.publish(velocity_input)
        self.steer_pub.publish(steering_input)
        self.iter += 1
 


    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(Odometry, '/p3d/odom', self.pose_sub_cb, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)



def main(args=None):
    rclpy.init(args=args)
    dmpc = StopMotors()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

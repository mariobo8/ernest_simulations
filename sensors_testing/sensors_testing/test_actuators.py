import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class TestActuators(Node):

    def __init__(self):
        super().__init__('stop_motors')

        self.set_subscribers_publishers()
        self.iter = 0.0
        self.switch = False


    # Callbacks Section
    def pose_sub_cb(self, msg):


        velocity_input = Float64MultiArray()
        steering_input = Float64MultiArray()
        velocity_input.data = [0.0, 0.0, 0.0, 0.0]
        steering_input.data = [self.iter, self.iter, 0.0, 0.0, 0.0]
        self.vel_pub.publish(velocity_input)
        self.steer_pub.publish(steering_input)
        self.iter += 0.001

        if self.iter > 1.5: 
            self.switch = True
            print(self.switch)
 


    # End: Callbacks Section

    def set_subscribers_publishers(self):
        self.pose_sub = self.create_subscription(Odometry, '/p3d/odom', self.pose_sub_cb, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)



def main(args=None):
    rclpy.init(args=args)
    dmpc = TestActuators()

    rclpy.spin(dmpc)
    dmpc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

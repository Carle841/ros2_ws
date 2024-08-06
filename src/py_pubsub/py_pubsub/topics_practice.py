import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

class TopicsPractice(Node):

    def __init__(self):
        super().__init__('topics_practice')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.countdown = 5
        self.state = 'countdown'
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        
    def pose_callback(self, msg):
        self.pose = msg    

    def timer_callback(self):
        if self.state == 'countdown':
            self.get_logger().info(f'Countdown: {self.countdown}')
            self.countdown -= 1
            if self.countdown < 0:
                self.state = 'drawing_spiral'
                self.get_logger().info('Drawing spiral')
                self.spiral_size = 0.5

        elif self.state == 'drawing_spiral':
            self.get_logger().info('Drawing spiral')
            msg = Twist()
            msg.linear.x = self.spiral_size
            msg.angular.z = 1.5
            self.spiral_size += 0.05
            self.publisher_.publish(msg)

            if self.pose and (self.pose.x < 1.0 or self.pose.x > 7.0 or self.pose.y < 1.0 or self.pose.y > 7.0):
                self.state = 'going_straight'
                self.get_logger().info('Going straight')

        elif self.state == 'going_straight':
            self.get_logger().info('Going straight')
            msg = Twist()
            msg.linear.x = 2.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicsPractice()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

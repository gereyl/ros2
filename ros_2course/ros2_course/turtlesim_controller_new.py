import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')

        self.declare_parameter('speed', 1.0)
        self.declare_parameter('omega', 20.0)

        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def cb_pose(self, msg):
        self.pose = msg


    def go_straight(self, distance):
        # Start moving towards the target
        current_distance = 0.0
        rate = self.create_rate(10)  # 10 Hz update rate
        start_x, start_y = self.pose.x, self.pose.y
        Kp = 1.0  # Proportional gain

        while rclpy.ok():
            # Calculate the remaining distance to go
            dx = self.pose.x - start_x
            dy = self.pose.y - start_y
            current_distance = math.sqrt(dx**2 + dy**2)
            distance_error = distance - current_distance

            if abs(distance_error) < 0.01:  # If the turtle is close enough to the target
                break

            # Update the speed based on the distance error
            speed = Kp * distance_error
            vel_msg = Twist()
            vel_msg.linear.x = max(min(speed, self.get_parameter('speed').get_parameter_value().double_value), -self.get_parameter('speed').get_parameter_value().double_value)
            self.twist_pub.publish(vel_msg)

            rate.sleep()

        # Stop the turtle
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)


    def turn(self, angle):
        # Start turning towards the target angle
        current_angle = 0.0
        rate = self.create_rate(10)  # 10 Hz update rate
        start_theta = self.pose.theta
        Kp = 1.0  # Proportional gain

        while rclpy.ok():
            # Calculate the remaining angle to turn
            dtheta = self.pose.theta - start_theta
            current_angle = math.degrees(dtheta) % 360.0
            angle_error = (angle - current_angle + 360) % 360

            if angle_error > 180:  # Choose the shortest rotation direction
                angle_error -= 360

            if abs(angle_error) < 1:  # If the turtle is close enough to the target
                break

            # Update the angular speed based on the angle error
            angular_speed = Kp * angle_error
            vel_msg = Twist()
            vel_msg.angular.z = max(min(math.radians(angular_speed), self.get_parameter('omega').get_parameter_value().double_value), -self.get_parameter('omega').get_parameter_value().double_value)
            self.twist_pub.publish(vel_msg)

            rate.sleep()

        # Stop the turtle
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)

    def draw_square(self, a):
        for i in range(4):
            self.go_straight(a)
            self.turn(90)

    def draw_poly(self, N, a):
        angle = 360 / N
        for i in range(N):
            self.go_straight(a)
            self.turn(angle)


    def go_to(self, x, y):
        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = self.pose.theta

        print("Going to (", x, ", ", y, ")")
        angle = (((math.atan2(y - y0, x - x0) - theta_0) * 180) / math.pi)
        print("angle ", angle)
        self.turn(angle)

        distance = math.sqrt(((x- x0) * (x - x0)) + ((y - y0) * (y - y0)))
        self.go_straight(distance)
        print("Arrived to (", self.pose.x, ", ", self.pose.y, ")")

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()
    #tc.turn(20.0,90.0)
    #tc.go_straight(1.0,4.0)

    tc.draw_poly(1.0, 90.0, 6, 3.0)

    #tc.go_to(2, 8)
    #tc.go_to(2, 2)
    #tc.go_to(3, 4)
    #tc.go_to(6, 2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

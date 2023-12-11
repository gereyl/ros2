import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(
                            Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

    # New method for TurtlesimController
    def cb_pose(self, msg):
        self.pose = msg

    def go_to(self, speed, omega, x, y):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y-y0, x-x0))
        angle = theta_1 - theta_0
        distance = math.sqrt(((x - x0) * (x - x0)) + (y - y0) * (y - y0))

        # Execute movement
        self.turn(omega, angle)
        self.go_straight(speed, distance)


    def go_to_ar(self, x, y):
        # Arányos szabályozó konstansainak beállítása
        Kp_angular = 20.0  # Arányos szabályozó erősítés a forduláshoz
        Kp_linear = 2.0   # Arányos szabályozó erősítés a sebességhez

        # Várakozik a pozícióra, ha még nem érkezett meg
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Várakozás a pozícióra...')
            rclpy.spin_once(self)

        # Miután megvan a pozíció, kiszámoljuk az irányt és a távolságot
        theta_0 = self.pose.theta
        x0 = self.pose.x
        y0 = self.pose.y

        # Szög kiszámítása az új pozíció és az aktuális pozíció között
        angle_to_goal = math.atan2(y - y0, x - x0)

        # Távolság kiszámítása az új pozíció és az aktuális pozíció között
        distance_to_goal = math.sqrt((x - x0)**2 + (y - y0)**2)

        # Forduljon az új pozíció felé
        angle_diff = self.normalize_angle(angle_to_goal - theta_0)
        print(angle_diff)

        # Szögsebesség beállítása arányos szabályozással
        angular_speed = Kp_angular * angle_diff

        # Egyenes vonalú sebesség beállítása arányos szabályozással
        linear_speed = Kp_linear * distance_to_goal

        # A sebességek korlátozása, hogy ne legyenek túl nagyok
        max_angular_speed = math.radians(180)  # Maximum 180 fok/perc
        max_linear_speed = 1.0  # Maximum 1 méter/perc

        angular_speed = max(min(angular_speed, max_angular_speed), -max_angular_speed)
        linear_speed = max(min(linear_speed, max_linear_speed), -max_linear_speed)

        # A mozgás végrehajtása
        if angular_speed != 0.0:
            self.turn(math.degrees(angular_speed), math.degrees(angle_diff))
        if linear_speed != 0.0:
            self.go_straight(linear_speed, distance_to_goal)

        self.get_logger().info(f'Elérte a célt: ({x}, {y})')


    def normalize_angle(self, angle):
        """
        Normalizálja a szöget -pi és pi közé.
        """
        while angle >= math.pi:
            angle -= 2 * math.pi
        while angle <= -math.pi:
            angle += 2 * math.pi
        return angle

    def go_straight(self, speed, distance):
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        loop_rate = self.create_rate(1000, self.get_clock()) # Hz

        T = abs(distance / speed)

        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)   # loop rate

        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)


    def turn(self, omega, angle):
        vel_msg = Twist()

        omega = float(omega)
        angle = float(angle)

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = -math.radians(omega)


        loop_rate = self.create_rate(1000, self.get_clock()) # Hz

        T = abs(angle / omega)

        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        #start_orientation = self.pose.theta
        #current_orientation = start_orientation
        #angle_turned = 0.0

        # Publish msg while the calculated time is up
        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)   # loop rate
            #current_orientation = self.pose.theta
            #angle_turned = current_orientation - start_orientation

            # Normalize the angle turned to be between [-pi, pi]
            #angle_turned = angle_turned % (2.0 * math.pi)
            #if angle_turned > math.pi:
               # angle_turned -= 2.0 * math.pi

            # Check if the turtle has reached the target angle
            #if abs(angle_turned) >= abs(angle):
              #  break

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')

    def draw_poly(self, speed, omega, N, a):

        angle = 360.0 / N
        for i in range(N):
            self.go_straight(speed, a)
            self.turn(omega, angle)


    def draw_koch_line(self, distance, depth, speed, omega):
        if depth == 0:
            self.go_straight(speed, distance)
        else:
            distance /= 3.0
            depth -= 1
            self.draw_koch_line(distance, depth, speed, omega)  # Előre
            self.turn(omega, 60)  # Balra fordul 60 fokot
            self.draw_koch_line(distance, depth, speed, omega)  # Előre
            self.turn(omega, -120)  # Jobbra fordul 120 fokot
            self.draw_koch_line(distance, depth, speed, omega)  # Előre
            self.turn(omega, 60)  # Balra fordul 60 fokot
            self.draw_koch_line(distance, depth, speed, omega)  # Előre

    def draw_koch_snowflake(self, distance, depth, speed, omega):
        for _ in range(3):
            self.draw_koch_line(distance, depth, speed, omega)  # Rajzol egy oldalt
            self.turn(omega, -120)  # Fordulás a következő oldalhoz

    def teleport_without_drawing(self, x, y, theta):
        # Make sure the service clients are created
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Lift the pen
        pen_request = SetPen.Request()
        pen_request.off = 1  # Set to 1 to lift the pen
        pen_future = self.pen_client.call_async(pen_request)
        rclpy.spin_until_future_complete(self, pen_future)

        # Teleport the turtle
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = x
        teleport_request.y = y
        teleport_request.theta = theta
        teleport_future = self.teleport_client.call_async(teleport_request)
        rclpy.spin_until_future_complete(self, teleport_future)

        # Put the pen down again if needed
        pen_request.off = 0  # Set to 0 to put the pen down
        pen_future = self.pen_client.call_async(pen_request)
        rclpy.spin_until_future_complete(self, pen_future)

    def set_initial_pose(self, x, y, theta):
        # Wait for the service to be available
        self.get_logger().info('Waiting for the "teleport_absolute" service...')
        self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Create a request with the desired pose
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Initial pose set successfully!')
        else:
            self.get_logger().error('Failed to call service "teleport_absolute"')

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()
    #tc.go_straight(1.0, 4.0)
    #tc.draw_poly(1.0, 100.0, 6, 2.0)

    tc.set_initial_pose(2.0, 8.0, 0.0)

    #tc.go_to(1.0, 20.0, 5, 10)
    #tc.go_to_ar(2,8)
    #tc.go_to_ar(2,2)
    #tc.go_to_ar(3,4)
    #tc.go_to_ar(6,2)

    # A Koch hópehely paramétere
    speed = 1.0  # A mozgás sebessége
    omega = 20.0  # A fordulási sebesség fok/perc
    distance = 5.0  # A vonalak hossza
    depth =4 # A fraktál mélysége

    # Koch hópehely rajzolásaS
    tc.draw_koch_snowflake(distance, depth, speed, omega)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

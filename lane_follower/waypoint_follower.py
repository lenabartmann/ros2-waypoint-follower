#!/usr/bin/env python3

# Import ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Message types for velocity commands and odometry feedback
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math


class SimpleWaypointFollower(Node):
    """
    A simple waypoint-based navigation node.

    The robot:
    - Subscribes to odometry data
    - Computes the angle and distance to the current waypoint
    - Uses a proportional controller (P-controller) for angular velocity
    - Moves sequentially through a predefined list of waypoints
    """

    def __init__(self):
        # Initialize ROS node with name
        super().__init__('simple_waypoint_follower')

        # Publisher: sends velocity commands to the robot
        # Topic: /originbot_1/cmd_vel
        # Queue size: 10 messages
        self.cmd_pub = self.create_publisher(
            Twist,
            '/originbot_1/cmd_vel',
            10
        )

        # Subscriber: receives odometry messages from the robot
        # Topic: /originbot_1/odom
        self.sub_odom = self.create_subscription(
            Odometry,
            '/originbot_1/odom',
            self.odom_callback,   # Function called when new odometry arrives
            10
        )

        # Predefined list of waypoints (x, y) in world coordinates
        self.waypoints = [
            (8.2, 1.1), (8.0, 14.8), (5.2, 19.1), (2.0, 21.1), (-3.0, 20.2),
            (-5.3, 17.5), (-5.7, 13.0), (-0.1, 6.4), (2.1, 1.8),
            (0.0, -4.2), (-2.6, -4.0), (-8.8, 3.8), (-13.0, 3.4),
            (-15.3, 0.3), (-11.1, -8.1), (1.6, -26.7), (6.1, -27.6),
            (8.9, -25.3), (14.1, -24.5), (19.3, -30.1), (27.1, -31.3),
            (32.3, -30.1), (35.9, -26.3), (35.0, -21.1), (31.2, -20.2),
            (25.6, -24.0), (21.6, -22.3), (21.0, -16.7), (19.4, -13.1),
            (12.4, -11.9), (10.0, -11.2), (8.4, -7.5), (8.4, -2.8),
            (8.2, 1.1)  # Final waypoint closes the loop
        ]

        # Index of current target waypoint
        self.current_wp = 0

        # Flag indicating whether final waypoint was reached
        self.reached_last = False

        self.get_logger().info("SimpleWaypointFollower started")


    def odom_callback(self, msg):
        """
        This function is called every time a new Odometry message is received.
        It computes control commands based on current robot pose.
        """

        # If final waypoint was already reached, do nothing
        if self.reached_last:
            return

        # Extract current robot position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation quaternion
        orientation = msg.pose.pose.orientation

        # Convert quaternion to yaw angle (heading angle)
        yaw = self.quaternion_to_yaw(orientation)

        # Get current waypoint coordinates
        wx, wy = self.waypoints[self.current_wp]

        # Compute vector from robot to waypoint
        dx = wx - x
        dy = wy - y

        # Compute desired heading angle toward waypoint
        # + pi was added due to coordinate system correction
        target_angle = math.atan2(dy, dx) + math.pi

        # Normalize angle to range [-pi, pi]
        target_angle = self.normalize_angle(target_angle)

        # Angular error between robot heading and target direction
        angle_error = self.normalize_angle(target_angle - yaw)

        # Euclidean distance to waypoint
        distance = math.sqrt(dx * dx + dy * dy)

        # Create velocity message
        twist = Twist()

        # If waypoint not yet reached
        if distance > 0.2:

            # If robot is strongly misaligned (> ~30°)
            if abs(angle_error) > 0.5:

                # Stop forward motion
                twist.linear.x = 0.0

                # Proportional angular controller (P-controller)
                # Kp = 0.8
                twist.angular.z = 0.8 * angle_error

            else:
                # Move forward while correcting orientation

                # Reduce forward speed if angle error increases
                twist.linear.x = 0.4 * (1.0 - abs(angle_error))

                # Proportional angular controller
                # Kp = 0.6
                twist.angular.z = 0.6 * angle_error

            # Limit maximum angular velocity for stability
            max_ang = 1.0
            twist.angular.z = max(
                -max_ang,
                min(max_ang, twist.angular.z)
            )

        else:
            # Waypoint reached -> switch to next
            self.current_wp += 1

            if self.current_wp >= len(self.waypoints):
                # Final waypoint reached -> stop robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.reached_last = True
                self.get_logger().info("Final waypoint reached. Stopping.")

            else:
                # Intermediate waypoint reached
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(
                    f"Waypoint {self.current_wp} reached. Moving to next."
                )

        # Publish velocity command
        self.cmd_pub.publish(twist)


    def quaternion_to_yaw(self, q):
        """
        Converts a quaternion into a yaw angle (rotation around Z-axis).

        Formula derived from quaternion-to-Euler conversion.
        """

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)


    def normalize_angle(self, angle):
        """
        Normalizes an angle to the range [-pi, pi].

        Prevents discontinuities when angle crosses +/- pi.
        """

        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Create node instance
    node = SimpleWaypointFollower()

    # Keep node running and processing callbacks
    rclpy.spin(node)

    # Clean shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

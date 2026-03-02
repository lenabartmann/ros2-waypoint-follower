#!/usr/bin/env python3
# Teleoperation node for controlling a mobile robot using keyboard input in ROS2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses  # library for terminal-based keyboard input
import time

class Teleop(Node):
    def __init__(self, stdscr):
        """
        Initialize the Teleop node.
        stdscr: curses standard screen object for terminal input/output.
        """
        super().__init__('teleop_manual_node')  # Name of the ROS2 node
        # Create a publisher for velocity commands on '/originbot_1/cmd_vel'
        self.publisher = self.create_publisher(Twist, '/originbot_1/cmd_vel', 10)
        
        self.stdscr = stdscr
        stdscr.nodelay(True)  # Don't wait for Enter; non-blocking input
        
        # Print instructions at the top-left of the terminal
        stdscr.addstr(0, 0, "Usage: w/v = forward/backward, a/d = left/right, q = stop, ESC = quit")
        
        # Start the main teleop loop
        self.run()

    def run(self):
        """
        Main loop for reading keyboard input and sending velocity commands.
        Runs continuously until ESC is pressed.
        """
        twist = Twist()  # Initialize a Twist message for velocities

        while True:
            try:
                key = self.stdscr.getkey()  # Read a key from the terminal
            except:
                key = None  # No key pressed (non-blocking)
            
            # Reset velocities to zero at the start of each loop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            # Map keys to robot motion
            if key in ['w', 'W']:  # Forward
                twist.linear.x = 0.5
            elif key in ['v', 'V']:  # Backward
                twist.linear.x = -0.5
            elif key in ['a', 'A']:  # Rotate left
                twist.angular.z = 1.0
            elif key in ['d', 'D']:  # Rotate right
                twist.angular.z = -1.0
            elif key in ['q', 'Q']:  # Stop robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == '\x1b':  # ESC key pressed exit loop
                break

            # Publish the velocity command to the robot
            self.publisher.publish(twist)

            # Sleep for 0.1 seconds to run the loop at ~10 Hz
            time.sleep(0.1)

def main(args=None):
    """
    Main entry point for the script.
    Initializes ROS2, wraps the Teleop class with curses, and shuts down ROS2 on exit.
    """
    rclpy.init(args=args)  # Initialize ROS2 Python client library
    # Initialize curses and pass the screen object to Teleop
    curses.wrapper(lambda stdscr: Teleop(stdscr))
    rclpy.shutdown()  # Cleanly shut down ROS2

if __name__ == '__main__':
    main()  # Run main() if script is executed directly

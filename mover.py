#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios

# Configurações de velocidade
MAX_LIN_VEL = 0.22  # Velocidade máxima linear
MAX_ANG_VEL = 2.84  # Velocidade máxima angular
LIN_VEL_STEP_SIZE = 0.1  # Passo de incremento para velocidade linear
ANG_VEL_STEP_SIZE = 0.1  # Passo de incremento para velocidade angular

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        self.get_logger().info(
            """
            Control Your Robot!
            ---------------------------
            Moving around:
                ↑
            ←       →
                ↓
            
            ↑ : move forward
            ↓ : move backward
            ← : turn left
            → : turn right
            Space key : zero the robot
            Enter key : shut down the system
            """
        )
        
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        key = self.getKey()
        if key:
            self.process_key(key)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                additional_chars = sys.stdin.read(2)
                return key + additional_chars
            return key
        return ''

    def process_key(self, key):
        if key == '\x1b[A':  # Up arrow
            self.current_linear_vel = min(MAX_LIN_VEL, self.current_linear_vel + LIN_VEL_STEP_SIZE)
        elif key == '\x1b[B':  # Down arrow
            self.current_linear_vel = max(-MAX_LIN_VEL, self.current_linear_vel - LIN_VEL_STEP_SIZE)
        elif key == '\x1b[D':  # Left arrow
            self.current_angular_vel = min(MAX_ANG_VEL, self.current_angular_vel + ANG_VEL_STEP_SIZE)
        elif key == '\x1b[C':  # Right arrow
            self.current_angular_vel = max(-MAX_ANG_VEL, self.current_angular_vel - ANG_VEL_STEP_SIZE)
        elif key == ' ':  # Space key
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.get_logger().info("Velocities zeroed.")
        elif key == '\n':  # Enter key
            self.get_logger().info("Enter key pressed - shutting down.")
            rclpy.shutdown()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            sys.exit()
        else:
            self.get_logger().info(f"Unrecognized key pressed: {key}")

        self.publish_twist()

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.current_linear_vel
        twist.angular.z = self.current_angular_vel
        self.publisher.publish(twist)
        self.get_logger().info(f"Linear Vel: {self.current_linear_vel}, Angular Vel: {self.current_angular_vel}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

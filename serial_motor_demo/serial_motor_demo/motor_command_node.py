import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand
import math

class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')

        # Create a subscriber to listen to Twist messages
        self.create_subscription(
            Twist, 
            'cmd_vel',  # Topic name might be different based on your setup
            self.twist_callback, 
            10
        )
        
        # Create a publisher to send out MotorCommand messages
        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

        # Especificaciones del robot
        self.declare_parameter('wheel_radius', 0.0553, ignore_override=True)
        self.declare_parameter('wheel_base', 0.284, ignore_override=True)

        # Retrieve parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        
        self.get_logger().info('Motor Command Node Started')
        
    def twist_callback(self, msg):
        try:
            # Assuming linear.x controls the speed and angular.z controls the steering
            speed = msg.linear.x
            rotation = msg.angular.z

            

            mot_1_speed = (speed + (rotation * self.wheel_base / 2)) / self.wheel_radius
            mot_2_speed = (speed - (rotation * self.wheel_base / 2)) / self.wheel_radius

            # Convert to rad/s
            mot_1_speed = mot_1_speed / (2 * math.pi)
            mot_2_speed = mot_2_speed / (2 * math.pi)
            
            # Create and publish the motor command
            motor_msg = MotorCommand()
            motor_msg.is_pwm = False
            motor_msg.mot_1_req_rad_sec = mot_1_speed
            motor_msg.mot_2_req_rad_sec = mot_2_speed
            self.publisher.publish(motor_msg)

            self.get_logger().info(f'Motor Commands sent: mot_1 = {mot_1_speed:.2f} rad/s, mot_2 = {mot_2_speed:.2f} rad/s')

        except Exception as e:
            self.get_logger().error(f'Error in twist_callback: {str(e)}')



def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

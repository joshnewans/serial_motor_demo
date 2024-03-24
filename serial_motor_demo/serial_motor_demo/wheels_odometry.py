import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion, Point, TransformStamped
from nav_msgs.msg import Odometry
from utils.transformations import euler2quaternion
from math import sin, cos
import tf2_ros

from serial_motor_demo_msgs.msg import MotorVels


class OdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.subscription = self.create_subscription(
            MotorVels, 'motor_vels', self.listener_callback, 10)
        self.publisher = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_time = self.get_clock().now()

        # Estado inicial del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Especificaciones del robot
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.0553).value
        self.wheel_base = self.declare_parameter('wheel_base', 0.284).value

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calcular velocidades lineales de las ruedas
        v_left = msg.mot_1_rad_sec * self.wheel_radius
        v_right = msg.mot_2_rad_sec * self.wheel_radius

        # Calcular la velocidad lineal y angular del robot
        V = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.wheel_base

        # Actualizar la posición y orientación
        self.x += V * cos(self.theta) * dt
        self.y += V * sin(self.theta) * dt
        self.theta += omega * dt

        # Crear y llenar el mensaje de odometría
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        orientation=euler2quaternion(0, 0, self.theta)
        odom.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
        )
            
        # Aquí puedes agregar la velocidad (Twist) si es necesario

        self.publisher.publish(odom)

        # Enviar transformación
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        quaternion = euler2quaternion(0, 0, self.theta)
        # Use the quaternion for rotation
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.odom_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

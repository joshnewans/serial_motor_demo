from matplotlib.pyplot import title
import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
import time
import math



class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.declare_parameter('encoder_cpr', value=0)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")

        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")

    
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {mot_1_pwm} {mot_2_pwm}")

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        self.send_command(f"m {mot_1_ct_per_loop} {mot_2_ct_per_loop}")

    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            # counts per loop = req rads/sec X revs/rad X counts/rev X secs/loop 
            scaler = (1 / (2*math.pi)) * self.get_parameter('encoder_cpr').value * (1 / self.get_parameter('loop_rate').value)
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * scaler
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def send_command(self, cmd_string):
        print("Sent: " + cmd_string)



        


def main(args=None):
    
    rclpy.init(args=args)

    motor_driver = MotorDriver()


    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)

        # motor_gui.update_image()

        # cv2.waitKey(2)
        time.sleep(0.01)


    # tk.mainloop()
    motor_driver.destroy_node()
    rclpy.shutdown()



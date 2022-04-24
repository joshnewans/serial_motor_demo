from matplotlib.pyplot import title
import rclpy
from rclpy.node import Node
import time

from tkinter import *
import math

from serial_motor_demo_msgs.msg import MotorCommand


class MotorGui(Node):

    def __init__(self):
        super().__init__('motor_gui')

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

        self.tk = Tk()
        self.tk.title("Serial Motor GUI")
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        

        Label(root, text="Serial Motor GUI").pack()

        mode_frame = Frame(root)
        mode_frame.pack(fill=X)


        self.mode_lbl = Label(mode_frame, text="ZZZZ")
        self.mode_lbl.pack(side=LEFT)
        self.mode_btn = Button(mode_frame, text="ZZZZ", command=self.switch_mode)
        self.mode_btn.pack(expand=True)

        self.set_mode(True)


        slider_max_frame = Frame(root)
        slider_max_frame.pack(fill=X)
        Label(slider_max_frame, text="Max Rev/sec").pack(side=LEFT)
        self.slider_max_val = 255
        self.slider_max_val_txt = f"{self.slider_max_val}"
        self.slider_max_val_box = Entry(slider_max_frame, textvariable=self.slider_max_val_txt)
        self.slider_max_val_box.pack(expand=True)


        m1_frame = Frame(root)
        m1_frame.pack(fill=X)
        Label(m1_frame, text="Motor 1").pack(side=LEFT)
        self.m1 = Scale(m1_frame, from_=0, to=42, orient=HORIZONTAL)
        self.m1.pack(side=LEFT, fill=X, expand=True)

        m2_frame = Frame(root)
        m2_frame.pack(fill=X)
        Label(m2_frame, text="Motor 2").pack(side=LEFT)
        self.m2 = Scale(m2_frame, from_=0, to=200, orient=HORIZONTAL)
        self.m2.pack(side=LEFT, fill=X, expand=True)

        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Send Cont.', command=self.show_values).pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Send', command=self.show_values).pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Mot', command=self.show_values).pack(side=LEFT)
        

        Button(root, text='Show', command=self.show_values).pack()

        enc_frame = Frame(root)
        enc_frame.pack(fill=X)

        self.enc_read_btn = Button(enc_frame, text='Read Encoders', command=self.show_values)
        self.enc_read_btn.pack(side=LEFT)

        self.enc_read_lbl = Label(enc_frame, text="XXX")
        self.enc_read_lbl.pack(side=LEFT)



    def show_values(self):
        print (self.m1.get(), self.m2.get())

    def send_motor_once(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        if (self.pwm_mode):
            msg.mot_1_req_rad_sec = float(self.m1.get())
            msg.mot_2_req_rad_sec = float(self.m2.get())
        else:
            msg.mot_1_req_rad_sec = float(self.m1.get()*2*math.pi)
            msg.mot_2_req_rad_sec = float(self.m2.get()*2*math.pi)

        self.publisher.publish(msg)

    def set_mode(self, new_mode):
        self.pwm_mode = new_mode
        if (self.pwm_mode):
            self.mode_lbl.config(text="Current Mode: PWM")
            self.mode_btn.config(text="Switch to Feedback Mode")
        else:
            self.mode_lbl.config(text="Current Mode: Feedback")
            self.mode_btn.config(text="Switch to PWM Mode")


    def switch_mode(self):
        self.set_mode(not self.pwm_mode)
        



    def update(self):
        self.tk.update()





def main(args=None):
    
    rclpy.init(args=args)

    motor_gui = MotorGui()


    rate = motor_gui.create_rate(2)
    
    while rclpy.ok():
        rclpy.spin_once(motor_gui)

        motor_gui.update()

        # motor_gui.update_image()

        # cv2.waitKey(2)
        time.sleep(0.01)


    # tk.mainloop()
    motor_gui.destroy_node()
    rclpy.shutdown()



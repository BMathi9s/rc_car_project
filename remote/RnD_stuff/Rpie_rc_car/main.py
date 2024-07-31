# main.py
from motor_driver import MotorDriver
from pyPS4Controller.controller import Controller
import time

speed_c = 1
sleep = 0

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.motor1 = MotorDriver(17, 18)
        self.motor2 = MotorDriver(22, 27)

    # def on_R3_up  (self, value):
    #     speed = value / 32768
    #     direction = 1
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(direction)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(direction)
    #     time.sleep(sleep)

    # def on_R3_down  (self, value):
    #     speed = value / 32768
    #     direction = 0
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(direction)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(direction)
    #     time.sleep(sleep)
        
    # def on_R3_left(self, value):
    #     speed = value / 32768
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(1)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(0)
    #     time.sleep(sleep)
        
    # def on_R3_right(self, value):
    #     speed = value / 32768
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(0)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(1)
    #     time.sleep(sleep)
        
        
    # def on_L3_up(self, value):
    #     speed = value / 32768
    #     direction = 1
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(direction)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(direction)
    #     time.sleep(sleep)
        
        
    # def on_L3_down(self, value):
    #     speed = value / 32768
    #     direction = 0
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(direction)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(direction)
    #     time.sleep(sleep)
        
    # def on_R3_left(self, value):
    #     speed = value / 32768
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(1)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(0)
    #     time.sleep(sleep)
        
    # def on_L3_right(self, value):
    #     speed = value / 32768
    #     self.motor1.set_speed(abs(speed))
    #     self.motor1.set_dir(0)
    #     self.motor2.set_speed(abs(speed))
    #     self.motor2.set_dir(1)
    #     time.sleep(sleep)
        

    def on_up_arrow_press(self):
        self.motor1.set_speed(speed_c)
        self.motor1.set_dir(1)
        self.motor2.set_speed(speed_c)
        self.motor2.set_dir(1)
        time.sleep(sleep)

    def on_down_arrow_press(self):
        self.motor1.set_speed(speed_c)
        self.motor1.set_dir(0)
        self.motor2.set_speed(speed_c)
        self.motor2.set_dir(0)
        time.sleep(sleep)

    def on_left_arrow_press(self):
        self.motor1.set_speed(speed_c)
        self.motor1.set_dir(0)
        self.motor2.set_speed(speed_c)
        self.motor2.set_dir(1)
        time.sleep(sleep)

    def on_right_arrow_press(self):
        self.motor1.set_speed(speed_c)
        self.motor1.set_dir(1)
        self.motor2.set_speed(speed_c)
        self.motor2.set_dir(0)
        time.sleep(sleep)

    def on_left_right_arrow_release(self):
        self.motor1.set_speed(0)
        self.motor2.set_speed(0)
        
    def on_up_down_arrow_release(self):
        self.motor1.set_speed(0)
        self.motor2.set_speed(0)
        
    # def on_L3_x_at_rest(self):
    #     self.motor1.set_speed(0)
    #     self.motor2.set_speed(0)
    # def on_L3_y_at_rest(self):
    #     self.motor1.set_speed(0)
    #     self.motor2.set_speed(0)
    
    # def on_R3_x_at_rest(self):
    #     self.motor1.set_speed(0)
    #     self.motor2.set_speed(0)
    
    # def on_R3_y_at_rest(self):
    #     self.motor1.set_speed(0)
    #     self.motor2.set_speed(0)
     

def main():
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()

if __name__ == "__main__":
    main()

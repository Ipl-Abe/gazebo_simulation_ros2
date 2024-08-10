#!/usr/bin/env python3

import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
import os
import time
import math

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription
        
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        # self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        if self.robot_m5:
            port = self.robot_m5
        else:
            print("hardware is not connected!")
            # port = self.robot_wio
        self.get_logger().info("port:%s, baud:%d" % (port, 115200))
        self.mc = MyCobot(port, 115200)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)
        
    def listener_callback(self, msg):
        data_list = []
        data_dict = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0,"joint4": 0.0, "joint5": 0.0, "joint6": 0.0}
        
        for i in range(len(msg.position)):        
            radians_to_angles = round(math.degrees(msg.position[i]), 2)
            data_dict[msg.name[i]] = radians_to_angles
        for key, value in data_dict.items():
            data_list.append(value)
        print('data_list: {}'.format(data_list))
        self.mc.send_angles(data_list, 25)
        
def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# encoding: utf-8
# @Author: liuyuan
# @Date: 2025/01/20
# sonar ros2 package

import math
import time
import queue
import rclpy
import signal
import threading
import numpy as np
import pandas as pd
import cv2
from app.common import Heart
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import UInt16, Bool
from interfaces.srv import SetInt64, SetFloat64List

from sdk.sonar import Sonar
from ros_robot_controller_msgs.msg import RGBState,RGBStates

class AvoidanceNode(Node):

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.sonar = Sonar()
        self.running = True
        self.heart = None #  Heartbeat package object
        
        self.name = name
        self.image_sub = None
        self.sonar_sub = None
        self.threshold = 30  # centimeters
        self.speed = 0.4 
        self.distance = 500
        self.distance_data = []
        self.bridge = CvBridge()
        self.is_running = False
        self.turn = True
        self.forward = True
        self.stopMotor = True
        self.lock = threading.RLock()
        
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # Publish image processing result
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # Chassis control
        self.sonar_rgb_pub = self.create_publisher(RGBStates, 'sonar_controller/set_rgb', 10)
        
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # Enter the game/mode
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # Exit the game/mode
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # Start the game/mode
        self.create_service(SetFloat64List, '~/set_param', self.set_parameters_srv_callback)  # Set parameters
        self.heart = Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # Heartbeat package
     
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

        self.exit_allowed = True  # Add flag to allow execution of exit_srv_callback
        
    
    def get_node_state(self, request, response):
        response.success = True
        return response
        
    def reset_value(self,):
        self.threshold = 30
        self.speed = 0.4
        self.distance = 500
        self.stopMotor = True
        self.is_running = False
        self.forward = True
        self.turn = True
    
    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'avoidance enter')
        self.sonar_rgb_controller()
        
        
        if self.image_sub is None:            
            self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)  # Subscribe to camera
        if self.sonar_sub is None:
            self.sonar_sub = self.create_subscription(Int32, 'sonar_controller/get_distance', self.distance_callback, 10)
        self.reset_value()
        self.exit_allowed = True  # Allow execution of exit_srv_callback
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'avoidance exit')

        with self.lock:
            if not self.exit_allowed:  # Check if execution is allowed
                self.get_logger().info("exit_srv_callback blocked by flag.")
                response.success = True
                response.message = "exit blocked"
                return response

            self.exit_allowed = False  # Prevent re-execution

            # Below is the original exit service logic
            self.is_running = False
            self.reset_value()
            self.mecanum_pub.publish(Twist())

            try:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
                if self.sonar_sub is not None:
                    self.destroy_subscription(self.sonar_sub)
                    self.sonar_sub = None
            except Exception as e:
                self.get_logger().error(str(e))
           # Stop and cleanup heartbeat
            if self.heart is not None:
                self.heart.destroy()
                self.heart = None
        
        response.success = True
        response.message = "exit"
        return response
        
    def sonar_rgb_controller(self):
        led1 = RGBState()
        led1.index = 0
        led1.red = 0
        led1.green = 0
        led1.blue = 255

        led2 = RGBState()
        led2.index = 1
        led2.red = 0
        led2.green = 0
        led2.blue = 255
        msg = RGBStates()
        msg.states = [led1,led2]
        self.sonar_rgb_pub.publish(msg)
                      
    
    def set_parameters_srv_callback(self, request, response):
        '''
        Set obstacle avoidance threshold and speed parameters
        :param req:
        :return:
        '''
        new_parameters = request.data
        new_threshold, new_speed = new_parameters
        self.get_logger().info("\033[1;32mn_t:{:2f}, n_s:{:2f}\033[0m".format(new_threshold, new_speed))
        if not 10 <= new_threshold <= 50:
            response.success = False
            response.message = "New threshold ({:.2f}) is out of range (10 ~ 50)".format(new_threshold)
            return response
        if not new_speed > 0:
            response.success = False
            response.message = "Invalid speed"
            return response

        with self.lock:
            self.threshold = new_threshold
            self.speed = round(new_speed / 80, 1)            
            self.speed = self.speed
        return response
        
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.is_running = request.data
            self.empty = 0
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response
    
    def distance_callback(self,msg):
        self.distance = msg.data
        
        twist = Twist()
        if self.is_running:                
            if self.distance / 10.0 <= self.threshold:   # Check if distance threshold is reached
                twist.angular.z = 11.0

                if self.turn: # Implement a check to prevent duplicate commands
                    self.turn = False
                    self.forward = True
                    self.stopMotor = True
            else:
                twist.linear.x = float(self.speed)
                twist.angular.z = 0.0
                if self.forward: # Implement a check to prevent duplicate commands
                    self.turn = True
                    self.forward = False
                    self.stopMotor = True
            self.mecanum_pub.publish(twist)
        else:
            if self.stopMotor: # 做一个判断防止重复发指令(implement a check to prevent duplicate commands)
                self.stopMotor = False
            self.turn = True
            self.forward = True
            #self.mecanum_pub.publish(Twist())
        
            
    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]
        
        dist = self.distance / 10.0 # 获取超声波传感器距离数据(get ultrasonic sensor distance data)
        self.distance_data.append(dist) # 距离数据缓存到列表(cache distance data into a list)
        data = pd.DataFrame(self.distance_data)
        data_ = data.copy()
        u = data_.mean()  # 计算均值(calculate the mean value)
        std = data_.std()  # 计算标准差(calculate standard deviation)
    
        data_c = data[np.abs(data - u) <= std]
        distance = data_c.mean()[0]
        if len(self.distance_data) == 5: # 多次检测取平均值(take the average of multiple detections)
            self.distance_data.remove(self.distance_data[0])
        
        result_image = np.copy(rgb_image)  # 显示结果用的画面 (the image used to display the result)
        # 把超声波测距值打印在画面上(print the ultrasonic distance measurement on the screen)
        result_image = cv2.putText(result_image, "Dist:%.1fcm"%distance, (30, 480-30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)  
            
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
    
    def destroy_node(self):
        if self.heart is not None:
            self.heart.destroy()
            self.heart = None
        super().destroy_node()


def main():
    node = AvoidanceNode('avoidance_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        node.sonar.setPixelColor(0, (0, 0, 0))
        node.sonar.setPixelColor(1, (0, 0, 0))        
        rclpy.shutdown()
        print('shutdown')
    finally:
        node.destroy_node()
        print('shutdown finish')

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import sys, os

sys.path.insert(0,'/home/ai2s/ros_catkin_ws/install' + '/lib/python3/dist-packages/')

import rospy
from rms_connect.srv import *
import robomaster
import time
from robomaster import robot, gimbal, conn

from MyQR import myqr
from PIL import Image

from imu_handler import IMUHandler
from battery_handler import BatteryHandler
from position_handler import PositionHandler
from wheel_controller import WheelController
from camera_handler import CameraHandler
from gimbal_controller import GimbalController
from gimbal_angle_handler import GimbalInfoHandler
from esc_info_handler import EscInfoHandler
from led_controller import LedController

class ConnectionHandler:
    def __init__(self):
        rospy.init_node('rm_connect', anonymous=True)
        self.robo = robot.Robot()
        self.robo.initialize(conn_type='sta')
        self.robo.set_robot_mode(mode=robot.CHASSIS_LEAD)
        self.led = self.robo.led
        self.imu_handler = IMUHandler()
        self.battery_handler = BatteryHandler()
        self.position_handler = PositionHandler(self.imu_handler)
        self.wheel_controller = WheelController(self.robo)
        self.gimbal_controller = GimbalController(self.robo)
        self.gimbal_info_handler = GimbalInfoHandler()
        self.esc_info_handler = EscInfoHandler()
        self.led_controller = LedController(self.robo)
        rospy.wait_for_service('rgb_led_control')
        try:
            rgb_led_control = rospy.ServiceProxy('rgb_led_control', RgbControl)
            rgb_led_control(0,255,0)
        except rospy.ServiceException as e:
            print("RGB Service call failed: %e", e)

        # Camera stuff
        self.camera_handler = CameraHandler(self.robo)
        
        self.rate = rospy.Rate(30)

        version = self.robo.get_version()
        rospy.loginfo("Robot version: {0}".format(version))
        
        self.robo.gimbal.moveto(pitch=0, yaw=0).wait_for_completed()

        self.robo.chassis.sub_imu(freq=20, callback=self.imu_handler.imu_info_handler)
        self.robo.chassis.sub_esc(freq=5, callback=self.esc_info_handler.esc_info_handler)

        self.robo.chassis.sub_attitude(freq=10, callback=self.imu_handler.attitude_info_handler)

        self.robo.battery.sub_battery_info(5, self.battery_handler.battery_info_handler)
        self.robo.chassis.sub_position(freq=10, callback=self.position_handler.position_info_handler)        
        self.robo.gimbal.sub_angle(freq=5, callback=self.gimbal_info_handler.gimbal_info_handler)
        
        while (not rospy.is_shutdown()):
            self.camera_handler.publish_image()
            self.wheel_controller.publish_message()
            self.gimbal_controller.publish_message()
            self.rate.sleep()

        self.led.set_led(comp=led.COMP_ALL, r=0, g=255, b=255, effect=led.EFFECT_ON)     
        self.robo.chassis.unsub_imu()
        self.robo.chassis.unsub_esc()
        self.robo.chassis.unsub_attitude()
        self.robo.battery.unsub_battery_info()
        self.robo.chassis.unsub_position()
        self.robo.gimbal.unsub_angle()

        # self.camera_task.cancel()

        self.robo.close()


if __name__ == '__main__':
    try:
        ConnectionHandler()
    except KeyboardInterrupt:
        rospy.signal_shutdown()
        
    
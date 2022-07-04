#!/usr/bin/env python3
import rospy
import robomaster
import time
from robomaster import robot
from imu_handler import IMUHandler
from battery_handler import BatteryHandler
from position_handler import PositionHandler
from wheel_controller import WheelController

class ConnectionHandler:
    def position_info_handler(self, info):
        x, y, z = info
        rospy.loginfo("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
        
    def esc_info_handler(self, info):
        wheel_speeds, wheel_angles, timestamps, states = info

    def __init__(self):
        rospy.init_node('rm_connect', anonymous=True)
        self.robo = robot.Robot()
        
        self.imu_handler = IMUHandler()
        self.battery_handler = BatteryHandler()
        self.position_handler = PositionHandler(self.imu_handler)
        self.wheel_controller = WheelController(self.robo)

        self.robo.initialize(conn_type='sta')
        self.robo.set_robot_mode(mode=robot.CHASSIS_LEAD)

        self.rate = rospy.Rate(60)

        version = self.robo.get_version()
        rospy.loginfo("Robot version: {0}".format(version))

        self.robo.chassis.sub_imu(freq=20, callback=self.imu_handler.imu_info_handler)
        self.robo.chassis.sub_esc(freq=5, callback=self.esc_info_handler)

        self.robo.chassis.sub_attitude(freq=10, callback=self.imu_handler.attitude_info_handler)

        self.robo.battery.sub_battery_info(5, self.battery_handler.battery_info_handler)
        self.robo.chassis.sub_position(freq=10, callback=self.position_handler.position_info_handler)        
        while (not rospy.is_shutdown()):
            self.rate.sleep()
              
        self.robo.chassis.unsub_imu()
        self.robo.chassis.unsub_esc()
        self.robo.chassis.unsub_attitude()
        self.robo.battery.unsub_battery_info()

        self.robo.close()


if __name__ == '__main__':
    ConnectionHandler()
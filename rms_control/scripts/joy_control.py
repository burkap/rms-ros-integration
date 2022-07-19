#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoyControl:
    def wheel_control(self, data):
        linear_x = data.axes[4]
        linear_y = data.axes[3]

        linear_sum = (linear_x ** 2 + linear_y ** 2) ** 0.5
        
        if linear_sum > 0:
            linear_x /= linear_sum
            linear_y /= linear_sum

        angular_z = data.axes[0]

        w1 = (linear_x + angular_z + linear_y) / 2 * 200 * (1 + 0.5 * data.buttons[5])
        w2 = (linear_x - angular_z - linear_y) / 2 * 200 * (1 + 0.5 * data.buttons[5])
        w3 = (linear_x - angular_z + linear_y) / 2 * 200 * (1 + 0.5 * data.buttons[5])
        w4 = (linear_x + angular_z - linear_y) / 2 * 200 * (1 + 0.5 * data.buttons[5])
        rospy.loginfo(f"{w1} {w2} {w3} {w4}")
        msg = Float64MultiArray()
        msg.data = [w1, w2, w3, w4]
        self.wheel_publisher.publish(msg)

    def gimbal_control(self, data):
        yaw = - data.axes[6]
        pitch = data.axes[7]
        
        msg = Float64MultiArray()
        msg.data = [pitch, yaw]
        self.gimbal_publisher.publish(msg)
    
    def on_joy_command(self, data):
        self.wheel_control(data)
        self.gimbal_control(data)

    def __init__(self):
        rospy.init_node('rms_joy_control', anonymous=True)
        self.sub = rospy.Subscriber("/joy", Joy, self.on_joy_command)
        self.wheel_publisher = rospy.Publisher("/robomaster/control/wheel_speed", Float64MultiArray, queue_size=10)
        self.gimbal_publisher = rospy.Publisher("/robomaster/control/gimbal", Float64MultiArray, queue_size=10)
        self.gimbal_coeff = 10
        self.rate = rospy.Rate(30)
           
        while (not rospy.is_shutdown()):
            self.rate.sleep()
              

if __name__ == '__main__':
    JoyControl()

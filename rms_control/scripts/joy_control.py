#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoyControl:
    def on_joy_command(self, data):
        
        linear_x = data.axes[4]
        linear_y = data.axes[3]

        linear_sum = (linear_x ** 2 + linear_y ** 2) ** 0.5
        
        if linear_sum > 0:
            linear_x /= linear_sum
            linear_y /= linear_sum

        angular_z = data.axes[0]

        w1 = (linear_x + angular_z + linear_y) / 2 * 500 * (1 + 0.5 * data.buttons[5])
        w2 = (linear_x - angular_z - linear_y) / 2 * 500 * (1 + 0.5 * data.buttons[5])
        w3 = (linear_x - angular_z + linear_y) / 2 * 500 * (1 + 0.5 * data.buttons[5])
        w4 = (linear_x + angular_z - linear_y) / 2 * 500 * (1 + 0.5 * data.buttons[5])
        rospy.loginfo(f"{w1} {w2} {w3} {w4}")
        msg = Float64MultiArray()
        msg.data = [w1, w2, w3, w4]
        self.publisher.publish(msg)

    def __init__(self):
        rospy.init_node('rms_joy_control', anonymous=True)
        
        self.sub = rospy.Subscriber("/joy", Joy, self.on_joy_command)
        self.publisher = rospy.Publisher("/robomaster/control/wheel_speed", Float64MultiArray, queue_size=10)

        self.rate = rospy.Rate(10)
           
        while (not rospy.is_shutdown()):

            self.rate.sleep()
              

if __name__ == '__main__':
    JoyControl()
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialArray
from std_msgs.msg import Float64MultiArray
import random
from itertools import cycle

class VisionModule:
    def on_command(self, data):
        margin = 80
        msg = Float64MultiArray()
        pitch = 0
        yaw = 0
        if len(data.fiducials) > 0:
            try:
                fiducial = next(x for x in data.fiducials if x.fiducial_id == self.id)
            except StopIteration:
                return
            x0, x1, x2, x3 = fiducial.x0, fiducial.x1, fiducial.x2, fiducial.x3
            y0, y1, y2, y3 = fiducial.y0, fiducial.y1, fiducial.y2, fiducial.y3
            bb_x = (x0+x1+x2+x3) / 4
            bb_y = (y0+y1+y2+y3) / 4

            self.error_x = bb_x - 320
            self.error_y = bb_y - 180

            yaw = self.error_x * self.kp_x
            pitch = -self.error_y * self.kp_y
            print(self.error_x, self.error_y)
        msg.data = [pitch, yaw]
        
        self.gimbal_publisher.publish(msg)
            

    def __init__(self):
        self.kp_x = 0.60
        self.kp_y = 0.50
        
        aa = [84, 42, 7]
        self.it = cycle(aa)
        self.error_x = 0
        self.error_y = 0

        self.id = 42

        rospy.init_node('rm_vision', anonymous=True)
        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.on_command)
        self.gimbal_publisher = rospy.Publisher("/robomaster/control/gimbal", Float64MultiArray, queue_size=10)

        self.rate = rospy.Rate()

        while not rospy.is_shutdown():
            if abs(self.error_x) < 20 and abs(self.error_y) < 20:
                self.id = next(self.it)
            print(f"Current id to look {self.id}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        VisionModule()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard interrupt")
        
    
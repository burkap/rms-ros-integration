#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Float64MultiArray

class VisionModuleYolo:
    def __init__(self):
        rospy.init_node('yolo_tracking')
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.on_track)
        self.gimbal_publisher = rospy.Publisher('/robomaster/control/gimbal', Float64MultiArray, queue_size=10)
        
        # Use class_ or id attribute, same thing
        self.class_ = "Person"
        self.id = 0
        
        # Parameters of the controller
        self.error_x = 0
        self.error_y = 0
        
        self.kp_x = 0.1
        self.kp_y = 0.1
        
        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            print(f"Current class to look {self.class_}")
            self.rate.sleep()
        
    def on_track(self, data):
        msg = Float64MultiArray()
        pitch = 0
        yaw = 0
        
        if len(data.bounding_boxes) > 0:
            try:
                object = next(x for x in data.bounding_boxes if x.id == self.id)
            except StopIteration:
                return
            
            x_min, x_max = object.xmin, object.xmax
            y_min, y_max = object.ymin, object.ymax
            
            bb_x = (x_min + x_max)/2
            bb_y = (y_min + y_max)/2
            
            self.error_x = bb_x - 320
            self.error_y = bb_y - 180
            
            yaw = self.error_x * self.kp_x
            pitch = -self.error_y * self.kp_y
            
        msg.data = [pitch, yaw]
        self.gimbal_publisher.publish(msg)
        
        
if __name__ == '__main__':
    try:
        VisionModuleYolo()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Keyboard Interrupt')
            
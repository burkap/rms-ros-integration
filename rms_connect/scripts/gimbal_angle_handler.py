import rospy
from std_msgs.msg import Float64MultiArray

class GimbalInfoHandler:
    def __init__(self):
        self.float64_pub = rospy.Publisher("/robomaster/feedback/gimbal_info", Float64MultiArray, queue_size=10)
        
    
    def gimbal_info_handler(self, info):
        pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = info
        msg = Float64MultiArray()
        
        msg.data = [pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle]

        self.float64_pub.publish(msg)
    
        
        
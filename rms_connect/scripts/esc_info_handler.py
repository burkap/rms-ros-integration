import rospy
from std_msgs.msg import Float64MultiArray

class EscInfoHandler:
    def __init__(self):
        self.float64_pub = rospy.Publisher("/robomaster/feedback/esc_info", Float64MultiArray, queue_size=10)
    
    def esc_info_handler(self, info):
        wheel_speeds, wheel_angles, timestamps, states = info
        msg = Float64MultiArray()
        
        msg.data = [wheel_speeds, wheel_angles, timestamps, states]
        
        self.float64_pub.publish(msg)
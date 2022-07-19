import rospy
from std_msgs.msg import String

class BatteryHandler:
    def __init__(self):
        self.battery_publisher = rospy.Publisher("battery_info", String, queue_size=10)

    def battery_info_handler(self, info):
        self.battery_publisher.publish(String(str(info)))
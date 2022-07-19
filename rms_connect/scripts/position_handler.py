import rospy
from tf.transformations import quaternion_from_euler
import tf
from imu_handler import IMUHandler
from std_msgs.msg import Float64MultiArray


class PositionHandler:
    def __init__(self, IMUHandler):
        self.broadcaster = tf.TransformBroadcaster()
        self.float64_pub = rospy.Publisher("/robomaster/feedback/position_info", Float64MultiArray, queue_size=10)
        self.imu_handler = IMUHandler
    
    def position_info_handler(self, info):
        x, y, z = info
        
        msg = Float64MultiArray()
        msg.data = [x, y, z]
        self.float64_pub.publish(msg)
        quat = quaternion_from_euler(self.imu_handler.roll, self.imu_handler.pitch, self.imu_handler.yaw)
        
        self.broadcaster.sendTransform((x,y,0),
                                       quat,
                                       rospy.Time.now(),
                                       "base_link",
                                       "map")
    
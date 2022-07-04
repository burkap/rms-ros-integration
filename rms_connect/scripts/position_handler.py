import rospy
from tf.transformations import quaternion_from_euler
import tf
from imu_handler import IMUHandler


class PositionHandler:
    def __init__(self, IMUHandler):
        self.broadcaster = tf.TransformBroadcaster()
        self.imu_handler = IMUHandler
    
    def position_info_handler(self, info):
        x, y, z = info
        # rospy.loginfo("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
        
        quat = quaternion_from_euler(self.imu_handler.roll, self.imu_handler.pitch, self.imu_handler.yaw)
        
        self.broadcaster.sendTransform((x,y,0),
                                       quat,
                                       rospy.Time.now(),
                                       "map",
                                       "base_link")
    
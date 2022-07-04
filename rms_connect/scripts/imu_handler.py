import rospy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu

class IMUHandler:
    def __init__(self):
        self.imu_publisher = rospy.Publisher("imu", Imu, queue_size=10)
        self.roll, self.pitch, self.yaw = (0, 0, 0) 

    def attitude_info_handler(self, info):
        self.yaw, self.pitch, self.roll = info

    def imu_info_handler(self, info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = info
        msg = Imu()
        
        msg.header.frame_id = "map"
        
        quad = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        
        msg.orientation.x = quad[0]
        msg.orientation.y = quad[1]
        msg.orientation.z = quad[2]
        msg.orientation.w = quad[3]
        
        msg.linear_acceleration.x = acc_x
        msg.linear_acceleration.y = acc_y
        msg.linear_acceleration.z = acc_z
        
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        self.imu_publisher.publish(msg)
        
        # rospy.loginfo("chassis imu: acc_x:{0}, acc_y:{1}, acc_z:{2}, gyro_x:{3}, gyro_y:{4}, gyro_z:{5}".format(
            # acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))
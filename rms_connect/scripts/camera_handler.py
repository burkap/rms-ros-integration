import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraHandler:
    def __init__(self, robo):
        self.image_publisher = rospy.Publisher("/image", Image, queue_size=10)
        self.robo = robo
        self.camera = self.robo.camera
        self.camera.start_video_stream(display=False)
        
        self.cv_bridge = CvBridge()
        self.encoding = 'bgr8'
        
        self.img_msg = Image()
    
    def publish_image(self):
        img = self.camera.read_cv2_image()
        self.img_msg = self.convert_cv2_to_ros_img(img)
        self.img_msg.header.stamp = rospy.Time.now()
        self.img_msg.header.frame_id = 'camera_link'
        self.image_publisher.publish(self.img_msg)

    def convert_cv2_to_ros_img(self, img):
        return self.cv_bridge.cv2_to_imgmsg(img, self.encoding)           
        
        
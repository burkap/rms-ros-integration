import rospy
from std_msgs.msg import Float64MultiArray

class GimbalController:
    def on_command(self, data):
        self.data = data.data
        print(self.data)
         
    def publish_message(self):
        self.robo.gimbal.drive_speed(pitch_speed=self.data[0]*self.vel_coeff, yaw_speed=self.data[1]*self.vel_coeff)
        
    def __init__(self, robo):
        self.robo = robo
        self.vel_coeff = 20
        self.data = [0, 0]
        self.subscriber = rospy.Subscriber("/robomaster/control/gimbal", Float64MultiArray, self.on_command)

    

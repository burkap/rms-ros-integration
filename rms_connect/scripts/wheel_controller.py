import rospy
from std_msgs.msg import Float64MultiArray

class WheelController:
    def on_command(self, data):
        self.data = data.data
        print(self.data)
        
    def publish_message(self):
        self.robo.chassis.drive_wheels(w1=self.data[0], w2=self.data[1], w3=self.data[2], w4=self.data[3])

    def __init__(self, robo):
        self.robo = robo
        self.data = [0, 0, 0, 0]
        self.subscriber = rospy.Subscriber("/robomaster/control/wheel_speed", Float64MultiArray, self.on_command)

    

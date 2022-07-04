import rospy
from std_msgs.msg import Float64MultiArray

class WheelController:
    def on_command(self, data):
        self.robo.chassis.drive_wheels(w1=data.data[0], w2=data.data[1], w3=data.data[2], w4=data.data[3])

    def __init__(self, robo):
        self.robo = robo
        self.subscriber = rospy.Subscriber("/robomaster/control/wheel_speed", Float64MultiArray, self.on_command)


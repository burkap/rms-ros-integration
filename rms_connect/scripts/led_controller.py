import rospy
from rms_connect.srv import RgbControl, RgbControlResponse
from robomaster import led
class LedController:
    def __init__(self, robo):
        self.robo = robo
        self.led = self.robo.led
        self.serve = rospy.Service('/rgb_led_control', RgbControl, self.handle_rgb_led)
        
    def handle_rgb_led(self, req):
        self.led.set_led(comp=led.COMP_ALL, 
                         r=req.r, g=req.g, b=req.b,
                         effect=led.EFFECT_ON)
        rospy.sleep(0.1)
        
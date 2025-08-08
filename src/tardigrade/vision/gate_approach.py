import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Modules included in this package
from controls.controllers import PIDRegulator
import tf_quaternion.transformations as transf
from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time

class GateApproachNode(Node):
    def __init__(self, name, **kwargs):
        print('GateApproachNode: initializing node')
        super().__init__(name, **kwargs)

        #self.config = {}

        #self._declare_and_fill_map("angular_multiplier", 1.0, "speed at which the gate is centered", self.config)
        #self._declare_and_fill_map("linear_multiplier", 1.0, "speed multiplier for msgs to cmd_vel", self.config)

        #self.sub_cmd_vel = self.create_subscription(geometry_msgs.Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        #self.sub_odometry = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.sub_image = self.create_subscription(Image, 'input', self.camera_callback,5)
        self.bridge_object = CvBridge()
        print(f"subscribed to {self.sub_image.topic}")
        self._output_pub = self.create_publisher(Twist, 'output', 1)
        print("created publisher")


    def camera_callback(self,msg):
        # convert ros image to opencv image  http://docs.ros.org/en/lunar/api/cv_bridge/html/python/index.html
        print("starting callback")
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
            print("ros image to CV conversion successful")
        except CvBridgeError as e:
            print(e)

        # image scaling
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 60
        crop_img = cv_image[height/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        # convert image to hsv encoding for better color detection in different lighting conditions (reduce saturation impact)
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # masking to obtain specific colors
        upper_yellow = np.array([70,48,255])
        lower_yellow = np.array([50,28,245])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # control center of blob (destination)
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            print(f"[{cx},{cy}]")
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            print(f"[{cx},{cy}]")

        # simple proportional controller for centering the gate
        error_horizontal = cx - width/2
        error_vertical = cy - height/2
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.linear.z = -error_vertical/100
        cmd.angular.z = -error_horizontal/100 
        print("ready to publish")
        self._output_pub.publish(cmd)

def main():
    print('starting gate_approach.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = GateApproachNode('gate_approach', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')


if __name__ == '__main__':
    main()
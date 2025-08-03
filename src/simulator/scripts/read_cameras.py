#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
import sys
import threading
import cv2
import imageio
import signal
import datetime
import random

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageListener(Node):
    """ Abstract class. Subscribes to ROS Image messages. Used for getting camera image data
    from the simulator and converting it for OpenCV. """

    def __init__(self, topic_name, *args, write=False, **kwargs):
        # ensure each node has a unique
        super().__init__('image_listener_' + str(random.random())[2:])
        self.window = topic_name.split('/')[-2]
        self.sub = self.create_subscription(Image, topic_name, self.callback, qos_profile_sensor_data)
        self.images = []
        self.FPS = 30
        self.index = 0
        self.done = False
        self.writer = None
        if write:
            time_string = datetime.datetime.now().strftime('%m-%d-%y_%H-%M-%S')
            self.writer = imageio.get_writer(self.window + '_' + time_string + '.mp4', mode='I', fps=self.FPS)
        else:
            self.timer = self.create_timer(1 / self.FPS, self.display_callback)

    def callback(self, img_msg):
        bridge = CvBridge()
        # Stores as a BGR array for OpenCV.
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.images.append(cv_image)
        if self.writer:
            self.writer.append_data(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        self.last_img = cv_image

    def get_images(self, time):
        self.images = []
        rate = self.create_rate(time)
        rate.sleep()
        return self.images

    def display_callback(self):
        # while True:
        if self.index >= len(self.images) or self.done:
            return
        cv2.imshow(self.window, self.images[self.index])
        self.index += 1
        # you need to hold a key to see the video progress. Press q to quit.
        if cv2.waitKey(0) & 0xFF == ord('q'):
            self.done = True
            print('done now for', self.window)


def signal_handler(signal_num, frame):
    for node in listeners:
        if node.writer:
            node.writer.close()
    exit(0)

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    signal.signal(signal.SIGINT, signal_handler)
    left_listener = ImageListener('/rexrov/cameraleft/image_raw', write=True)
    right_listener = ImageListener('/rexrov/cameraright/image_raw', write=True)
    listener = ImageListener('/rexrov/camera/image_raw', write=True)
    listeners = [left_listener, right_listener, listener]
    executor = SingleThreadedExecutor()
    for node in listeners:
        executor.add_node(node)
    executor.spin()
    print('got here after spin')
    cv2.destroyAllWindows()
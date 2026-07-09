import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import os
from pathlib import Path

class ZedRecorder(Node):
    def __init__(self):
        super().__init__('zed_camera_recorder')
        
        self.image_topic = '/zed/zed_node/rgb/image_rect_color'
        
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        filename = f"tardigrade_cam_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        
        home_dir = str(Path.home())
        save_dir = os.path.join(home_dir, 'tardigrade_videos')
        os.makedirs(save_dir, exist_ok=True) 

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        filename = f"tardigrade_cam_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        full_path = os.path.join(save_dir, filename)
        
        # !!Assuming 720p at 30fps!!
        self.out = cv2.VideoWriter(full_path, fourcc, 30.0, (1280, 720))
        self.get_logger().info(f"Subscribed to {self.image_topic}")
        self.get_logger().info(f"Recording started: saving to {full_path}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.out.write(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def destroy_node(self):
        self.get_logger().info("Stopping recording and saving video file...")
        self.out.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = ZedRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
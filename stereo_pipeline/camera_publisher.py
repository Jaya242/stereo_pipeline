import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoCameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')
        
        # Publishers for left and right camera topics
        self.left_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)
        
        self.bridge = CvBridge()
        
        # Open default webcam (index 0)
        self.cap = cv2.VideoCapture(0)
        
        # Set timer for ~30 FPS (0.033 seconds)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Stereo Camera Publisher Initialized.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from camera.")
            return

        # Resize frame to a standard resolution (e.g., 640x480)
        frame = cv2.resize(frame, (640, 480))
        
        # Split the single frame into left and right halves 
        # (Or you can duplicate the frame if using a single monocular webcam for testing)
        height, width, _ = frame.shape
        mid = width // 2
        
        left_img = frame[:, :mid]
        right_img = frame[:, mid:]
        
        # If your webcam is standard monocular, you can duplicate it instead for testing:
        # left_img = frame
        # right_img = frame

        # Convert to ROS Image messages and publish
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8")
        
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    publisher = StereoCameraPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

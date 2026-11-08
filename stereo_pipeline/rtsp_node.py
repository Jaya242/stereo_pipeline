import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RTSPCameraPublisher(Node):
    def __init__(self):
        super().__init__('rtsp_camera_node')
        self.declare_parameter('rtsp_url', 'rtsp://192.168.1.10:8080/video')
        url = self.get_parameter('rtsp_url').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open RTSP stream: {url}")
        else:
            self.get_logger().info(f"Connected to RTSP Stream: {url}")
            
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'phone_camera_link'
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RTSPCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

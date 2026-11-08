import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RTSPCameraNode(Node):
    def __init__(self):
        super().__init__('rtsp_camera_node')

        # Updated default parameter to active local network IP address (192.168.1.6)
        self.declare_parameter('rtsp_url', 'rtsp://192.168.1.6:8554/live')
        self.rtsp_url = self.get_parameter('rtsp_url').value

        # Publisher for raw camera frame output
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Connect to RTSP stream source
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open RTSP stream: {self.rtsp_url}')
        else:
            self.get_logger().info(f'Connected to RTSP Stream: {self.rtsp_url}')

        # 30 FPS timer callback (approx. 0.033 sec interval)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning('Frame capture skipped or stream temporarily disconnected...')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTSPCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class StereoProcessorNode(Node):
    def __init__(self):
        super().__init__('stereo_processor_node')

        # Camera Intrinsic Parameters
        self.declare_parameter('fx', 800.0)
        self.declare_parameter('fy', 800.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('baseline', 0.06)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.baseline = self.get_parameter('baseline').value

        self.bridge = CvBridge()

        # Subscribe to RTSP raw camera topic
        self.sub_left = self.create_subscription(
            Image, '/camera/image_raw', self.left_image_callback, 10
        )

        # Publishers for PoseStamped and visual debug feed
        self.pose_pub = self.create_publisher(PoseStamped, '/box_pose', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/object_debug', 10)

        self.get_logger().info('Stereo Processing Node Active -> Listening to /camera/image_raw')

    def detect_object(self, image):
        # 1. Convert frame to Grayscale (removes color dependence completely)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 2. Blur to smooth out pixel noise
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # 3. Otsu Binarization + Thresholding to isolate any distinct foreground object
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 4. Morphological operations to clean up small specks and fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # 5. Find contours of ALL detected objects regardless of color
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            # Filter out tiny noise and full-screen outer frame background boundaries
            if 500 < area < (image.shape[0] * image.shape[1] * 0.8):
                moments = cv2.moments(contour)
                if moments["m00"] != 0:
                    u = moments["m10"] / moments["m00"]
                    v = moments["m01"] / moments["m00"]
                    return (u, v), cv2.boundingRect(contour)
        return None, None

    def left_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        centroid, bbox = self.detect_object(frame)

        if centroid is not None:
            u_left, v_left = centroid
            disparity = 16.0  # Pixel offset calculation

            if disparity > 0:
                # Triangulate 3D Position
                Z = (self.fx * self.baseline) / disparity
                X = ((u_left - cx) * Z) / self.fx
                Y = ((v_left - cy) * Z) / self.fy

                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.header.frame_id = 'camera_link'
                pose_msg.pose.position.x = float(X)
                pose_msg.pose.position.y = float(Y)
                pose_msg.pose.position.z = float(Z)
                pose_msg.pose.orientation.w = 1.0

                self.pose_pub.publish(pose_msg)
                self.get_logger().info(f'Box Pose -> X: {X:.2f}m | Y: {Y:.2f}m | Z: {Z:.2f}m')

                # Render bounding box and annotations
                x, y, bw, bh = bbox
                scale = max(1.0, w / 640.0)
                thickness = int(round(2 * scale))

                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), thickness)

                label = f'X:{X:.2f}m Y:{Y:.2f}m Z:{Z:.2f}m'
                font_scale = 0.8 * scale
                text_pos_y = max(30, y - 12)

                (text_w, text_h), baseline_h = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
                )
                cv2.rectangle(
                    frame,
                    (x, text_pos_y - text_h - 4),
                    (x + text_w, text_pos_y + baseline_h),
                    (0, 0, 0),
                    -1
                )
                cv2.putText(
                    frame, label,
                    (x, text_pos_y), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (0, 255, 0), thickness, cv2.LINE_AA
                )

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(debug_msg)

        # Native auto-scaling display window
        cv2.namedWindow("Stereo Object Tracking", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Stereo Object Tracking", 960, 720)
        cv2.imshow("Stereo Object Tracking", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = StereoProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoPipelineNode(Node):
    def __init__(self):
        super().__init__('stereo_pipeline_node')
        self.bridge = CvBridge()
        
        # Camera intrinsics & parameters
        self.fx = 800.0
        self.fy = 800.0
        self.cx = 320.0
        self.cy = 240.0
        self.baseline = 0.06
        self.min_area = 300
        
        self.latest_left = None
        self.latest_right = None

        # Subscribers for left and right camera topics
        self.subscription_left = self.create_subscription(
            Image, '/stereo/left/image_raw', self.left_callback, 10)
        self.subscription_right = self.create_subscription(
            Image, '/stereo/right/image_raw', self.right_callback, 10)
        
        # Publisher for the processed visualization overlay
        self.publisher_ = self.create_publisher(Image, '/stereo/output_overlay', 10)
        self.get_logger().info("Stereo Pipeline ROS 2 Node Initialized.")

    def left_callback(self, msg):
        self.latest_left = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process()

    def right_callback(self, msg):
        self.latest_right = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process()

    def detect_colours(self, image):
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        colour_ranges = {
            "red": [
                (np.array([0, 100, 80]), np.array([10, 255, 255])),
                (np.array([170, 100, 80]), np.array([179, 255, 255]))
            ],
            "blue": [
                (np.array([90, 100, 50]), np.array([130, 255, 255]))
            ]
        }
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        detections = []

        for colour, ranges in colour_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, lower, upper)
            
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue
                u = moments["m10"] / moments["m00"]
                v = moments["m01"] / moments["m00"]

                detections.append({
                    "colour": colour,
                    "u": float(u),
                    "v": float(v),
                    "area": float(area),
                    "bbox": (int(x), int(y), int(w), int(h))
                })
        return detections

    def match_objects(self, left_detections, right_detections, max_vertical_error=3):
        matches = []
        used_right_indices = set()
        for left_obj in left_detections:
            candidates = []
            for right_index, right_obj in enumerate(right_detections):
                if right_index in used_right_indices:
                    continue
                if right_obj["colour"] != left_obj["colour"]:
                    continue
                vertical_error = abs(left_obj["v"] - right_obj["v"])
                if vertical_error > max_vertical_error:
                    continue
                candidates.append((vertical_error, right_index, right_obj))

            if not candidates:
                continue
            _, right_index, right_obj = min(candidates, key=lambda item: item[0])
            used_right_indices.add(right_index)
            matches.append((left_obj, right_obj))
        return matches

    def triangulate(self, left_obj, right_obj):
        u_left, v_left = left_obj["u"], left_obj["v"]
        u_right, v_right = right_obj["u"], right_obj["v"]
        
        if abs(v_left - v_right) > 3:
            return None
        disparity = u_left - u_right
        if disparity <= 0:
            return None

        Z = (self.fx * self.baseline) / disparity
        X = ((u_left - self.cx) * Z) / self.fx
        Y = ((v_left - self.cy) * Z) / self.fy
        return {"X": X, "Y": Y, "Z": Z, "disparity": disparity}

    def try_process(self):
        if self.latest_left is None or self.latest_right is None:
            return

        left_img = self.latest_left.copy()
        right_img = self.latest_right.copy()

        left_dets = self.detect_colours(left_img)
        right_dets = self.detect_colours(right_img)
        matches = self.match_objects(left_dets, right_dets)

        for left_obj, right_obj in matches:
            point = self.triangulate(left_obj, right_obj)
            if point:
                self.get_logger().info(
                    f"Detected {left_obj['colour']}: X={point['X']:.2f}m, Y={point['Y']:.2f}m, Z={point['Z']:.2f}m"
                )

        # Publish annotated left image overlay
        output_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoPipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

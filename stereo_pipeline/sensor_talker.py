import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SensorTalkerNode(Node):
    def __init__(self):
        super().__init__('sensor_talker_node')
        
        # Create a publisher on topic '/sensor_distance' using Float32 message type
        self.publisher_ = self.create_publisher(Float32, '/sensor_distance', 10)
        
        # Timer triggers every 0.1 seconds (10 Hz real-time publish rate)
        self.timer = self.create_timer(0.1, self.publish_distance)
        self.get_logger().info('Sensor Talker Node Started -> Publishing on /sensor_distance')

    def publish_distance(self):
        msg = Float32()
        # Simulate a fluctuating distance reading between 0.2m and 2.0m
        msg.data = round(random.uniform(0.2, 2.0), 2)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent Distance: {msg.data} m')


def main(args=None):
    rclpy.init(args=args)
    node = SensorTalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

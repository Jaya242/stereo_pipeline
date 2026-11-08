import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CarListenerNode(Node):
    def __init__(self):
        super().__init__('car_listener_node')
        
        # Subscribe to '/sensor_distance' topic with a callback function
        self.subscription = self.create_subscription(
            Float32,
            '/sensor_distance',
            self.distance_callback,
            10
        )
        self.get_logger().info('Car Listener Node Active -> Waiting for data...')

    def distance_callback(self, msg):
        distance = msg.data
        
        # Real-time decision logic based on live incoming messages
        if distance < 0.5:
            self.get_logger().warn(f'ALERT! Obstacle at {distance}m -> APPLY BRAKES!')
        else:
            self.get_logger().info(f'Clear path ({distance}m) -> Keep Driving')


def main(args=None):
    rclpy.init(args=args)
    node = CarListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map')
        self.publisher_ = self.create_publisher(String, '/map', 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.publish_map)

    def publish_map(self):
        msg = String()
        msg.data = 'Map data here'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

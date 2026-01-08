import rclpy
from traffic.traffic_logic import TrafficSignalPublisher

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

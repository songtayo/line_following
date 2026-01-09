import rclpy
from control.lane_follower_logic import LaneFollowerNode

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 로봇 정지 명령
        stop_twist = Twist()
        for _ in range(5):
            node.publisher.publish(stop_twist)
            time.sleep(0.01)
        node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

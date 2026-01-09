import rclpy
import time
from geometry_msgs.msg import Twist  # Twist 임포트 확인
from control.lane_follower_logic import LaneFollowerNode

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 1. KeyboardInterrupt 발생 시 로그 출력 (선택 사항)
        node.get_logger().info('Stopping the robot...')
    finally:
        # 2. Twist 메시지 생성을 위한 명시적 선언
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        
        # 3. 짧게 반복하여 메시지 전송 (메시지 유실 방지)
        for _ in range(5):
            node.publisher.publish(stop_twist)
            # rclpy.spin_once 대신 잠시 대기
            time.sleep(0.1) 
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

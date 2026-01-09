import rclpy
import time
from control.lane_follower_logic import LaneFollowerNode

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        # 1. 로봇 정지 함수 호출
        node.stop_robot()
        
        # 2. [핵심] 메시지가 네트워크로 완전히 나갈 때까지 대기
        # 이 과정이 없으면 터틀봇은 정지 신호를 받기 전에 연결이 끊깁니다.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
            
        # 3. 안전하게 종료
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("Robot control terminated safely.")

if __name__ == '__main__':
    main()

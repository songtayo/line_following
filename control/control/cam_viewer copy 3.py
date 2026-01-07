import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import py_trees
import time

# --- [BT 행동 노드 정의] ---

class CheckObstacle(py_trees.behaviour.Behaviour):
    """장애물 여부를 체크하는 조건 노드"""
    def __init__(self, node):
        super().__init__("Check Obstacle")
        self.node = node

    def update(self):
        # 장애물이 감지되면 SUCCESS를 반환하여 다음 AvoidAction이 실행되게 함
        if self.node.obstacle != 0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class AvoidAction(py_trees.behaviour.Behaviour):
    """장애물 감지 시의 동작 (정지 및 회피 준비)"""
    def __init__(self, node):
        super().__init__("Avoid Action")
        self.node = node

    def update(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.node.publisher.publish(twist)
        self.node.get_logger().warn("Obstacle Detected! Stopping...")
        # 이 시점에서 robot_state를 1로 바꿔서 나중에 회피 로직으로 넘길 수도 있음
        self.node.robot_state = 1 
        return py_trees.common.Status.RUNNING

class LaneFollowAction(py_trees.behaviour.Behaviour):
    """차선 주행 및 신호등 제어 로직"""
    def __init__(self, node):
        super().__init__("Lane Follow Action")
        self.node = node

    def update(self):
        twist = Twist()
        
        # 1. 차선 인식 여부 확인
        if not self.node.is_detected:
            self.node.publisher.publish(twist)
            return py_trees.common.Status.RUNNING

        # 2. 조향 계산
        if self.node.lane_center is not None:
            error = self.node.lane_center - self.node.img_center
            twist.angular.z = -self.node.p_gain_dual * error
            twist.linear.x = self.node.linear_speed
        elif self.node.single_lane_info is not None:
            cx = self.node.single_lane_info['cx']
            side = self.node.single_lane_info['side']
            target_offset = 140 if side == 'left' else -140
            twist.angular.z = -self.node.p_gain_single * (cx - self.node.img_center + target_offset)
            twist.linear.x = self.node.linear_speed * 0.6

        # 3. 신호등 판단 (Red=0, Green=1, Passed=2)
        if self.node.traffic_light == 0:
            self.node.count = 0
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif self.node.traffic_light == 1:
            self.node.count += 1
            if self.node.count < 50:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                self.node.traffic_light = 2 # 통과 상태로 변경

        self.node.publisher.publish(twist)
        return py_trees.common.Status.RUNNING

# --- [메인 노드 클래스] ---

class LaneFollowerBTNode(Node):
    def __init__(self):
        super().__init__('lane_follower_bt_node')
        
        # ROS 2 구독/발행 설정
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 1)
        self.subscription = self.create_subscription(Int32, 'traffic_color', self.traffic_callback, 10)
        self.subscription = self.create_subscription(Int32, 'obstacle_status', self.lidar_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.init_variables()
        
        # 행동 트리 구성
        self.bt_root = self.setup_behavior_tree()
        
        # BT 실행 타이머 (20Hz)
        self.timer = self.create_timer(0.04, self.control_timer_callback)

    def init_variables(self):
        self.img_center = 160.0
        self.lane_center = None
        self.is_detected = False
        self.single_lane_info = None
        self.min_lane_distance = 100
        self.p_gain_dual = 0.01
        self.p_gain_single = 0.01
        self.linear_speed = 0.1
        self.count = 0
        self.traffic_light = 1
        self.obstacle = 0
        self.robot_state = 0

    def setup_behavior_tree(self):
        """Selector를 사용해 장애물 회피를 주행보다 우선순위에 둠"""
        root = py_trees.composites.Selector(name="Main Logic", memory=False)

        # 줄기 1: 장애물 대응 (장애물 체크 -> 멈춤/회피)
        avoid_sequence = py_trees.composites.Sequence(name="Avoidance Branch", memory=True)
        avoid_sequence.add_children([CheckObstacle(self), AvoidAction(self)])

        # 줄기 2: 일반 주행
        lane_follow = LaneFollowAction(self)

        root.add_children([avoid_sequence, lane_follow])
        return root

    def control_timer_callback(self):
        """매 틱마다 트리를 업데이트함"""
        self.bt_root.tick_once()

    def listener_callback(self, data):
        """영상 인식 부분 (기존 로직 유지)"""
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            height, width = frame.shape[:2]
            self.img_center = width / 2.0

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            roi_vertices = np.array([[(0, height), (width, height), 
                                      (width, int(height*0.5)), (0, int(height*0.5))]], np.int32)
            roi_mask = np.zeros_like(mask)
            cv2.fillPoly(roi_mask, roi_vertices, 255)
            cropped_mask = cv2.bitwise_and(mask, roi_mask)

            lines = cv2.HoughLinesP(cropped_mask, 1, np.pi/180, 20, 20, 100)
            self.update_lane_data(lines)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def update_lane_data(self, lines):
        """검출된 선 정보를 변수에 저장"""
        if lines is not None:
            line_list = [{'len': np.sqrt((l[0][2]-l[0][0])**2 + (l[0][3]-l[0][1])**2), 
                          'cx': (l[0][0]+l[0][2])/2.0} for l in lines]
            line_list.sort(key=lambda x: x['len'], reverse=True)

            selected = []
            if line_list:
                selected.append(line_list[0])
                for i in range(1, len(line_list)):
                    if abs(selected[0]['cx'] - line_list[i]['cx']) > self.min_lane_distance:
                        selected.append(line_list[i])
                        break

            if len(selected) == 2:
                self.lane_center = (selected[0]['cx'] + selected[1]['cx']) / 2.0
                self.single_lane_info, self.is_detected = None, True
            elif len(selected) == 1:
                side = 'left' if selected[0]['cx'] < self.img_center else 'right'
                self.single_lane_info = {'cx': selected[0]['cx'], 'side': side}
                self.lane_center, self.is_detected = None, True
            else:
                self.is_detected = False
        else:
            self.is_detected = False

    def traffic_callback(self, msg): self.traffic_light = msg.data
    def lidar_callback(self, msg): self.obstacle = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
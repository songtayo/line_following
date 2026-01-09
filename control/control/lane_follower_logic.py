import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower_node')
        
        # 1. ROS 2 구독 및 발행 설정
        # 큐 사이즈를 1로 설정하여 최신 프레임 위주로 처리하도록 유도 (병목 방지)
        self.subscription_1 = self.create_subscription(
           Image, '/image_raw', self.listener_callback, 1)
        self.subscription_2 = self.create_subscription(
            Int32, 'traffic_color', self.traffic_callback, 10)
        self.subscription_3 = self.create_subscription(
            Int32, 'obstacle_status', self.lidar_callback, 10)
        # Publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. 제어 타이머 설정 (0.05초 = 20Hz 주기)
        self.timer = self.create_timer(0.04, self.control_timer_callback)
        
        self.bridge = CvBridge()

        # 3. 데이터 공유를 위한 상태 변수
        self.img_center = 160.0    # 320x240 해상도 기준 중앙값 초기화
        self.lane_center = None    # 두 선의 중앙 x좌표
        self.is_detected = False   # 선 검출 여부
        self.single_lane_info = None # 한 선 검출 시 정보 저장

        # 4. 설정값 및 제어 파라미터
        self.min_lane_distance = 100 # 두 차선 사이의 최소 x 거리
        self.p_gain_dual = 0.01     # 두 줄 보일 때 조향 민감도
        self.p_gain_single = 0.01   # 한 줄 보일 때 조향 민감도 (강하게)
        self.linear_speed = 0.1      # 직진 기본 속도

        # 5. traffic light parameter
        self.count = 0
        self.traffic_light = 1

        # 6. obstacle parameter
        self.obstacle = 0

        # 7. robot state
        self.robot_state = 0 # 0 is noraml, 1 is avoid obstacle

        # ---------------------------------------------------------
        # 8. obstacle avoid parameter
        # 2. 상태 변수
        self.state = 'FORWARD'
        self.timer_count = 0
        
        # 3. [튜닝 구역] 수정된 파라미터
        self.speed_fwd = 0.1      # 직진 속도 (m/s)
        self.speed_turn = 0.5     # 회전 속도 (rad/s)
        
        # [수정 1] 회전 각도 부족 해결 -> 시간을 늘림 (기존 31 -> 45)
        # (만약 너무 많이 돌면 40정도로 줄이세요)
        self.TIME_TURN_90 = 45    
        
        # [수정 2] 옆으로 이동 거리 2배 증가 (기존 20 -> 40)
        # 장애물 폭이 넓어도 안 부딪히게 함
        self.TIME_MOVE_SIDE = 40  
        
        # [수정 3] 앞으로 지르는 거리 2배 이상 증가 (기존 40 -> 90)
        # 멀리서 인식해도 장애물을 완전히 지나칠 때까지 직진하도록 아주 길게 잡음
        self.TIME_PASS_LONG = 90  

        self.get_logger().info('Control has started')


    def listener_callback(self, data):
        """
        [함수 1: 영상 인식]
        영상을 받아 선을 검출하고 조향에 필요한 좌표만 계산하여 변수에 저장합니다.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            height, width = frame.shape[:2]
            self.img_center = width / 2.0

            # 1. 전처리 (그레이스케일 및 이진화)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 황토색 바닥 환경을 고려하여 threshold 값 100 설정
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

            # 2. 모폴로지 연산 (노이즈 제거)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 3. ROI 설정 (하단 50% 영역)
            roi_vertices = np.array([[(0, height), (width, height), 
                                      (width, int(height*0.5)), (0, int(height*0.5))]], np.int32)
            roi_mask = np.zeros_like(mask)
            cv2.fillPoly(roi_mask, roi_vertices, 255)
            cropped_mask = cv2.bitwise_and(mask, roi_mask)

            # 4. 허프 변환을 이용한 선 검출
            lines = cv2.HoughLinesP(cropped_mask, 1, np.pi/180, threshold=20, 
                                    minLineLength=20, maxLineGap=100)

            line_list = []
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    line_list.append({'len': length, 'cx': (x1 + x2) / 2.0, 'pts': (x1, y1, x2, y2)})

                # 길이를 기준으로 내림차순 정렬하여 가장 유효한 선 2개 선택
                line_list.sort(key=lambda x: x['len'], reverse=True)

                selected = []
                if len(line_list) > 0:
                    selected.append(line_list[0])
                    for i in range(1, len(line_list)):
                        if abs(selected[0]['cx'] - line_list[i]['cx']) > self.min_lane_distance:
                            selected.append(line_list[i])
                            break

                # 인식 결과 업데이트
                if len(selected) == 2:
                    self.lane_center = (selected[0]['cx'] + selected[1]['cx']) / 2.0
                    self.single_lane_info = None
                    self.is_detected = True
                    # 시각화 (중앙점)
                    cv2.circle(frame, (int(self.lane_center), int(height*0.75)), 10, (0, 255, 255), -1)
                elif len(selected) == 1:
                    self.lane_center = None
                    side = 'left' if selected[0]['cx'] < self.img_center else 'right'
                    self.single_lane_info = {'cx': selected[0]['cx'], 'side': side}
                    self.is_detected = True
                else:
                    self.is_detected = False
            else:
                self.is_detected = False

            # 화면 중앙선 시각화
            # cv2.line(frame, (int(self.img_center), 0), (int(self.img_center), height), (0, 255, 0), 1)
            # cv2.imshow("Lane Detection Result", frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def traffic_callback(self, msg):
        self.traffic_light = msg.data

    def lidar_callback(self, msg):
        self.obstacle = msg.data

    def avoid_obstacle(self):
        twist = Twist()
        self.get_logger().info(f"avoid~~~~~!!!!!!!")
        # 로직은 기존과 동일 (timer_count가 증가하는 속도가 빨라졌으므로 상단 변수로 상쇄됨)
        if self.state == 'FORWARD':
            self.get_logger().warn("🚧 장애물 감지! 0.04s 주기로 회피 시작")
            self.state = 'STEP1_TURN_L'
            self.timer_count = 0
        
        elif self.state == 'STEP1_TURN_L':
            twist.angular.z = self.speed_turn
            self.timer_count += 1
            if self.timer_count > self.TIME_TURN_90:
                self.state = 'STEP2_MOVE_OUT'; self.timer_count = 0
                
        elif self.state == 'STEP2_MOVE_OUT':
            twist.linear.x = self.speed_fwd
            self.timer_count += 1
            if self.timer_count > self.TIME_MOVE_SIDE:
                self.state = 'STEP3_TURN_R'; self.timer_count = 0
                
        elif self.state == 'STEP3_TURN_R':
            twist.angular.z = -self.speed_turn 
            self.timer_count += 1
            if self.timer_count > self.TIME_TURN_90:
                self.state = 'STEP4_PASS_OBSTACLE'; self.timer_count = 0
                
        elif self.state == 'STEP4_PASS_OBSTACLE':
            twist.linear.x = self.speed_fwd
            self.timer_count += 1
            if self.timer_count > self.TIME_PASS_LONG:
                self.state = 'STEP5_TURN_R'; self.timer_count = 0
                
        elif self.state == 'STEP5_TURN_R':
            twist.angular.z = -self.speed_turn
            self.timer_count += 1
            if self.timer_count > self.TIME_TURN_90:
                self.state = 'STEP6_RETURN_IN'; self.timer_count = 0
        
        elif self.state == 'STEP6_RETURN_IN':
            twist.linear.x = self.speed_fwd
            self.timer_count += 1
            if self.timer_count > self.TIME_MOVE_SIDE: 
                self.state = 'STEP7_REALIGN'; self.timer_count = 0
                
        elif self.state == 'STEP7_REALIGN':
            twist.angular.z = self.speed_turn
            self.timer_count += 1
            if self.timer_count > self.TIME_TURN_90:
                self.get_logger().info("✅ 회피 완료!")
                self.robot_state = 0
                self.state = 'FORWARD'; self.timer_count = 0

        return twist

    def control_timer_callback(self):
        """
        [함수 2: 로봇 제어]
        타이머 주기에 따라 독립적으로 실행되며, 저장된 최신 좌표 정보를 바탕으로 속도 명령을 발행합니다.
        """
        twist = Twist()

        if self.robot_state == 0:
            if not self.is_detected:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                if self.lane_center is not None:
                    # [상황 1] 두 줄 인식: 두 줄의 중앙을 따라 주행
                    error = self.lane_center - self.img_center
                    twist.angular.z = -self.p_gain_dual * error
                    twist.linear.x = self.linear_speed
                
                elif self.single_lane_info is not None:
                    # [상황 2] 한 줄 인식: 차선으로부터 100px 간격 유지하며 주행
                    cx = self.single_lane_info['cx']
                    side = self.single_lane_info['side']
                    
                    # 왼쪽 차선이면 우측으로 100px, 오른쪽 차선이면 좌측으로 100px 오프셋
                    target_offset = 140 if side == 'left' else -140
                    error = cx - self.img_center
                    
                    # 한 줄 주행 시에는 속도를 낮추고 더 민감하게 회전
                    twist.angular.z = -self.p_gain_single * (error + target_offset)
                    twist.linear.x = self.linear_speed * 0.6

            if self.obstacle == 0:
                if self.traffic_light == 0:
                    self.count = 0
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif self.traffic_light == 1:
                    self.count += 1
                    if self.count < 100:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        pass
                else:
                    pass
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.robot_state = 1
        else:
            twist = self.avoid_obstacle()
        
        self.publisher.publish(twist)



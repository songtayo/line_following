import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image # 이미지 토픽을 받기 위해 필요
from cv_bridge import CvBridge    # ROS 이미지를 OpenCV 이미지로 변환
import cv2
import numpy as np

class TrafficSignalPublisher(Node):
    def __init__(self):
        super().__init__('traffic_signal_publisher')
        # 1. 발행자 설정 (판단 결과 전송)
        self.publisher_ = self.create_publisher(Int32, 'traffic_color', 10)
        # 2. 구독자 설정 (터틀봇 카메라 이미지 수신)
        # 터틀봇3의 경우 이미지 토픽명이 보통 '/image_raw'입니다.
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        # ROS-OpenCV 변환 브릿지 초기화
        self.bridge = CvBridge()
        self.get_logger().info('Traffic Signal Publisher Node with TurtleBot Camera has started.')

    def image_callback(self, msg):
        try:
            # 3. ROS Image 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # :빨간색_원: 빨간색 범위
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red = (
            cv2.inRange(hsv, lower_red1, upper_red1) +
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        # :큰_파란색_원: 파란색 범위
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        red_pixels = cv2.countNonZero(mask_red)
        blue_pixels = cv2.countNonZero(mask_blue)
        # print(f"red: {red_pixels}, blue: {blue_pixels}")
        # self.get_logger().info(f"red: {red_pixels}, blue: {blue_pixels}")
        result_msg = Int32()
        # 조건 판단
        if red_pixels > 120:
            result_msg.data = 0
            status = "STOP (RED)"
            color = (0, 0, 255)
            pixel_info = f"Red pixels: {red_pixels}"
        elif blue_pixels > 1000:
            result_msg.data = 1
            status = "GO (BLUE)"
            color = (255, 0, 0)
            pixel_info = f"Blue pixels: {blue_pixels}"
        else:
            # 인식되지 않아도 화면은 유지
            # cv2.imshow("TurtleBot Camera", frame)
            # cv2.waitKey(1)
            return
        # 결과 발행
        self.publisher_.publish(result_msg)
        # self.get_logger().info(f'Published: {result_msg.data} ({pixel_info})')
        # 화면에 정보 표시
        # cv2.putText(frame, status, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        # cv2.putText(frame, pixel_info, (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        # cv2.imshow("TurtleBot Camera", frame)
        # cv2.waitKey(1)


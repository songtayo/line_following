import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy # QoS 설정을 위해 추가

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        
        # 1. QoS 프로파일 설정 (터틀봇 라이다와 호환되도록 Best Effort 설정)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # 이 부분이 핵심입니다
            depth=10
        )
        
        # 2. 구독자(Subscriber): 설정한 QoS 프로파일을 적용하여 생성
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile) # 여기에 qos_profile을 넣습니다
        
        # 3. 발행자(Publisher)
        self.publisher_ = self.create_publisher(Int32, 'obstacle_status', 10)
        
        self.detection_range = 0.3
        self.get_logger().info('QoS 설정이 완료된 장애물 감지 노드가 시작되었습니다.')

    def scan_callback(self, msg):
        # (나머지 로직은 이전과 동일)
        front_ranges = msg.ranges[0:15] + msg.ranges[345:360]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        
        status_msg = Int32()
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < self.detection_range:
                status_msg.data = 1
                # self.get_logger().warn(f'장애물 감지 (1) | 거리: {min_dist:.2f}m')
            else:
                status_msg.data = 0
        else:
            status_msg.data = 0

        self.publisher_.publish(status_msg)



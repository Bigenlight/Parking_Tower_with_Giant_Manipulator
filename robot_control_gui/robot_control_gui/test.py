#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # -----------------------
        #      Publishers
        # -----------------------
        self.turtlebot_pub = self.create_publisher(String, '/turtlebot_state', 10)
        self.arm_pub = self.create_publisher(String, '/arm_state', 10)

        # -----------------------
        #      Subscriber
        # -----------------------
        self.destination_sub = self.create_subscription(
            Int32,
            '/destination',
            self.destination_callback,
            10
        )

        # -----------------------
        #   Internal Variables
        # -----------------------
        self.move_in_progress = False    # 로봇팔이 현재 이동 중인지 여부
        self.current_move_type = None    # 'port_to_tower' 또는 'tower_to_port'
        self.moving_timer = None         # 이동 시뮬레이션용 타이머
        self.move_count = 0             # 이동 시뮬레이션(초 단위) 누적

        self.get_logger().info('TestNode Initialized.')

        # 시작 시퀀스: 1초 후 tb1 arrived → 2초 후 tb2 arrived → 3초 후 port arrived
        self.create_one_shot_timer(1.0, self.publish_tb1_arrived)
        self.create_one_shot_timer(2.0, self.publish_tb2_arrived)
        self.create_one_shot_timer(3.0, self.publish_port_arrived)

    # ------------------------------------------------
    #   Initial Sequence: tb1, tb2 도착, port 도착
    # ------------------------------------------------
    def publish_tb1_arrived(self):
        """Publish 'tb1 arrived'"""
        msg = String(data='tb1 arrived')
        self.turtlebot_pub.publish(msg)
        self.get_logger().info('[TestNode] turtlebot1 선착장 도착')

    def publish_tb2_arrived(self):
        """Publish 'tb2 arrived'"""
        msg = String(data='tb2 arrived')
        self.turtlebot_pub.publish(msg)
        self.get_logger().info('[TestNode] turtlebot2 선착장 도착')

    def publish_port_arrived(self):
        """Publish 'port arrived' (로봇팔 선착장 도착)"""
        msg = String(data='port arrived')
        self.arm_pub.publish(msg)
        self.get_logger().info('[TestNode] robot arm 선착장 도착')

    # ------------------------------------------------
    #   Subscriber Callback (/destination)
    # ------------------------------------------------
    def destination_callback(self, msg: Int32):
        """
        스카이넷에서 퍼블리시되는 /destination 을 수신.
        - 1~9: 선착장 → 주차장 이동(3초 뒤 'tower arrived')
        - 0:   주차장 → 선착장 이동(3초 뒤 'port arrived')
        """
        destination_value = msg.data
        self.get_logger().info(f'[TestNode] Received /destination: {destination_value}')

        # 이미 이동 중이면 새로운 /destination 명령은 무시(또는 처리 로직 변경 가능)
        if self.move_in_progress:
            self.get_logger().warn('로봇팔이 이미 이동 중입니다. 새 /destination 명령은 무시합니다.')
            return

        # 목적지에 따라 이동 시뮬레이션
        if 1 <= destination_value <= 9:
            # port -> tower
            self.start_moving_simulation('port_to_tower')
        elif destination_value == 0:
            # tower -> port
            self.start_moving_simulation('tower_to_port')
        else:
            self.get_logger().warn(f'[TestNode] 무효한 목적지: {destination_value}. (0 ~ 9) 범위만 유효')

    # ------------------------------------------------
    #   Moving Simulation (3초 이동 후 도착 pub)
    # ------------------------------------------------
    def start_moving_simulation(self, move_type: str):
        """
        move_type: 'port_to_tower' or 'tower_to_port'
        3초 뒤 각각 'tower arrived' 또는 'port arrived' 퍼블리시.
        """
        self.move_in_progress = True
        self.current_move_type = move_type
        self.move_count = 0

        self.get_logger().info(f'[TestNode] 로봇팔 이동 시작: {move_type}')
        # 1초 단위로 move_in_progress를 표시
        self.moving_timer = self.create_one_shot_timer(1.0, self.moving_timer_callback)

    def moving_timer_callback(self):
        """1초마다 호출되어 총 3초 동안 이동 시뮬레이션"""
        self.move_count += 1
        self.get_logger().info(f'[TestNode] 로봇팔 이동 중... {self.move_count}초 경과')

        if self.move_count < 3:
            # 3초가 될 때까지 타이머를 재설정
            self.moving_timer = self.create_one_shot_timer(1.0, self.moving_timer_callback)
        else:
            # 3초가 지나면 도착 메시지(pub) 후 종료
            if self.current_move_type == 'port_to_tower':
                self.publish_tower_arrived()
            elif self.current_move_type == 'tower_to_port':
                self.publish_port_arrived()
            else:
                self.get_logger().warn(f'[TestNode] 알 수 없는 move_type: {self.current_move_type}')

            # 이동 상태 리셋
            self.move_in_progress = False
            self.current_move_type = None
            self.moving_timer = None
            self.move_count = 0

    def publish_tower_arrived(self):
        """Publish 'tower arrived'."""
        msg = String(data='tower arrived')
        self.arm_pub.publish(msg)
        self.get_logger().info('[TestNode] robot arm 주차장 도착 (tower arrived)')

    def publish_port_arrived(self):
        """Publish 'port arrived'."""
        msg = String(data='port arrived')
        self.arm_pub.publish(msg)
        self.get_logger().info('[TestNode] robot arm 선착장 도착 (port arrived)')

    # ------------------------------------------------
    #   One-shot Timer Helper
    # ------------------------------------------------
    def create_one_shot_timer(self, timer_period, callback):
        """timer_period 초 후에 1회성 callback을 수행하는 타이머 생성"""
        def timer_cb():
            callback()
            self.destroy_timer(timer_handle)

        timer_handle = self.create_timer(timer_period, timer_cb)
        return timer_handle


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('[TestNode] Stopped cleanly')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

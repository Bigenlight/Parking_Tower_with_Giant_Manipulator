import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from collections import deque

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # 구독자 설정
        self.turtlebot_sub = self.create_subscription(
            String,
            '/turtlebot_state',
            self.turtlebot_state_callback,
            10
        )
        self.arm_sub = self.create_subscription(
            String,
            '/arm_state',
            self.arm_state_callback,
            10
        )

        # 발행자 설정
        self.tb1_cmd_vel_pub = self.create_publisher(Twist, '/tb1/cmd_vel', 10)
        self.tb2_cmd_vel_pub = self.create_publisher(Twist, '/tb2/cmd_vel', 10)
        self.arm_command_pub = self.create_publisher(String, '/arm_commands', 10)

        # 상태 변수 초기화
        self.tb1_arrived = False
        self.tb2_arrived = False
        self.arm_at_port = False
        self.arm_busy = False  # Arm이 작업 중인지 여부

        # TurtleBot 큐 (우선순위: tb1 먼저)
        self.tb_queue = deque()

        # 현재 작업 중인 TurtleBot
        self.current_tb = None

        # 목적지 번호
        self.destination = 1
        self.max_destination = 9

        # 상태
        self.state = 'IDLE'  # 초기 상태

        self.get_logger().info('Control Node Initialized.')

    def turtlebot_state_callback(self, msg):
        self.get_logger().info(f'Received turtlebot_state: "{msg.data}"')
        if msg.data == 'tb1 arrived':
            if not self.tb1_arrived:
                self.tb1_arrived = True
                self.add_to_queue('tb1')
        elif msg.data == 'tb2 arrived':
            if not self.tb2_arrived:
                self.tb2_arrived = True
                self.add_to_queue('tb2')
        self.process_queue()

    def arm_state_callback(self, msg):
        self.get_logger().info(f'Received arm_state: "{msg.data}"')
        if msg.data == 'port arrived':
            self.arm_at_port = True
            self.arm_busy = False
            self.get_logger().info('Robot arm has arrived at the port.')
            self.process_queue()
        elif msg.data == 'tower arrived':
            self.get_logger().info('Robot arm has arrived at the tower.')
            self.handle_arm_at_tower()

    def add_to_queue(self, tb):
        # 큐에 TurtleBot 추가 (tb1 우선)
        if tb not in self.tb_queue:
            if tb == 'tb1':
                # tb1이면 큐 앞에 추가
                self.tb_queue.appendleft(tb)
            else:
                # tb2는 뒤에 추가
                self.tb_queue.append(tb)
            self.get_logger().info(f'Added {tb} to the queue.')

    def process_queue(self):
        if self.state == 'IDLE' and self.arm_at_port and not self.arm_busy and self.tb_queue:
            self.current_tb = self.tb_queue.popleft()
            self.get_logger().info(f'Starting processing of {self.current_tb}.')
            self.state = 'MOVE_TB_FORWARD'
            self.move_turtlebot(self.current_tb, linear_x=0.3, duration=5)

    def move_turtlebot(self, tb, linear_x, duration):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = 0.0

        publisher = self.tb1_cmd_vel_pub if tb == 'tb1' else self.tb2_cmd_vel_pub
        direction = "forward" if linear_x > 0 else "backward"
        self.get_logger().info(f'Moving {tb} {direction} with linear.x = {linear_x} for {duration} seconds.')

        # 발행 횟수 계산 (10Hz 기준)
        iterations = int(duration * 10)
        for _ in range(iterations):
            publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 정지 명령 발행
        twist.linear.x = 0.0
        publisher.publish(twist)
        self.get_logger().info(f'{tb} stopped moving.')

        if linear_x > 0:
            # 앞으로 이동한 경우 Robot Arm에 'on board' 명령 발행
            self.state = 'ARM_ON_BOARD'
            self.arm_busy = True
            self.publish_arm_command(f'on board {self.destination}')
        elif linear_x < 0:
            # 후진한 경우 Robot Arm에 'empty' 명령 발행
            self.state = 'ARM_EMPTY'
            self.arm_busy = True
            self.publish_arm_command('empty')

    def publish_arm_command(self, command):
        msg = String()
        msg.data = command
        self.arm_command_pub.publish(msg)
        self.get_logger().info(f'Published to /arm_commands: "{command}"')

    def handle_arm_at_tower(self):
        if self.state == 'ARM_ON_BOARD':
            # Robot Arm이 목적지에 도착한 경우 TurtleBot을 후진시킴
            self.get_logger().info(f'Handling arm at tower for {self.current_tb}.')
            self.state = 'MOVE_TB_BACKWARD'
            self.move_turtlebot(self.current_tb, linear_x=-0.3, duration=5)
        elif self.state == 'ARM_EMPTY':
            # Robot Arm이 'empty' 상태에서 선착장으로 이동
            self.get_logger().info('Handling arm empty state.')
            self.state = 'ARM_MOVE_TO_PORT'
            self.publish_arm_command('move_to_port')

    def run(self):
        # 메인 루프
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # 상태에 따른 추가 처리
            if self.state == 'ARM_ON_BOARD':
                # Robot Arm이 목적지로 이동 중이므로 대기
                pass
            elif self.state == 'ARM_MOVE_TO_PORT':
                # Robot Arm이 선착장으로 이동 중이므로 대기
                pass
            elif self.state == 'MOVE_TB_BACKWARD':
                # TurtleBot을 후진시킨 후 Robot Arm을 비움
                # 'empty' 명령 발행 후 arm_state_callback에서 처리
                pass
            elif self.state == 'ARM_EMPTY':
                # 이미 handle_arm_at_tower에서 처리됨
                pass
            elif self.state == 'IDLE':
                # 모든 작업 완료, 대기 상태
                pass

    def handle_arm_arrival_at_port(self):
        self.arm_at_port = True
        self.arm_busy = False
        self.get_logger().info('Robot arm has arrived back at the port.')
        self.state = 'IDLE'
        self.process_queue()

    def handle_arm_move_to_port_complete(self):
        # Robot Arm이 선착장으로 이동을 완료한 경우
        self.handle_arm_arrival_at_port()

    # arm_state_callback에서 'port arrived' 메시지를 받으면 handle_arm_arrival_at_port를 호출
    def arm_state_callback(self, msg):
        self.get_logger().info(f'Received arm_state: "{msg.data}"')
        if msg.data == 'port arrived':
            self.handle_arm_arrival_at_port()
        elif msg.data == 'tower arrived':
            self.handle_arm_at_tower()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Control node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from collections import deque

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Subscribers
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

        # Publishers
        self.tb1_cmd_vel_pub = self.create_publisher(Twist, '/tb1/cmd_vel', 10)
        self.tb2_cmd_vel_pub = self.create_publisher(Twist, '/tb2/cmd_vel', 10)
        self.arm_command_pub = self.create_publisher(String, '/arm_commands', 10)
        self.destination_pub = self.create_publisher(Int32, '/destination', 10)  # 목적지 토픽 퍼블리셔

        # State variables
        self.tb1_arrived = False
        self.tb2_arrived = False
        
        # 로봇팔 상태
        self.arm_at_port = False   # 로봇팔이 선착장(port)에 도착했는가?
        self.arm_busy = False      # 로봇팔이 현재 동작중인가?

        # TurtleBot queue (priority: tb1 first)
        self.tb_queue = deque()

        # 현재 처리중인 TurtleBot
        self.current_tb = None

        # Destination number
        self.destination = 1  # 다음 배달(주차) 위치
        self.max_destination = 9

        # FSM(상태 기계)용 상태
        self.state = 'IDLE'
        # 가능한 state:
        # - IDLE               : 대기
        # - MOVE_TB_FORWARD    : 터틀봇 전진중
        # - ARM_ON_BOARD       : 로봇팔에 on board 중(터틀봇 적재)
        # - MOVE_TB_BACKWARD   : 터틀봇 후진중
        # - ARM_EMPTY          : 로봇팔 empty 상태(터틀봇 하역)
        # - ARM_MOVE_TO_PORT   : 로봇팔이 port로 이동중(필요하다면 사용)

        self.get_logger().info('Control Node Initialized.')

        # Timer to publish initial message after 1 second
        self.initial_timer = self.create_timer(1.0, self.initial_publish_callback)
        self.initial_timer_active = True  # 한 번만 실행할 플래그

    def initial_publish_callback(self):
        """처음 노드 시작 시, 로봇팔에 'start_control_sequence' 를 보내는 예시."""
        if self.initial_timer_active:
            msg = String()
            msg.data = "start_control_sequence"
            self.arm_command_pub.publish(msg)
            self.get_logger().info('Published initial message to /arm_commands: "start_control_sequence"')
            self.initial_timer_active = False
            self.initial_timer.cancel()  # 타이머 정지

    def turtlebot_state_callback(self, msg: String):
        """TurtleBot 상태 수신 콜백"""
        self.get_logger().info(f'Received turtlebot_state: "{msg.data}"')

        if msg.data == 'tb1 arrived':
            if not self.tb1_arrived:
                self.tb1_arrived = True
                self.add_to_queue('tb1')
        elif msg.data == 'tb2 arrived':
            if not self.tb2_arrived:
                self.tb2_arrived = True
                self.add_to_queue('tb2')

        # 큐 처리 시도
        self.process_queue()

    def arm_state_callback(self, msg: String):
        """로봇팔 상태 수신 콜백"""
        self.get_logger().info(f'Received arm_state: "{msg.data}"')
        if msg.data == 'port arrived':
            # 로봇팔이 선착장에 도착
            self.arm_at_port = True
            self.arm_busy = False
            self.get_logger().info('Robot arm has arrived at the port.')
            self.state = 'IDLE'
            self.process_queue()

        elif msg.data == 'tower arrived':
            # 로봇팔이 tower(주차장)에 도착
            self.get_logger().info('Robot arm has arrived at the tower.')
            self.handle_arm_at_tower()

    def add_to_queue(self, tb):
        """tb1이 먼저 오면 맨 앞, 그 외(tb2)는 뒤로 넣는 식으로 순서 관리"""
        if tb not in self.tb_queue:
            if tb == 'tb1':
                self.tb_queue.appendleft(tb)
            else:
                self.tb_queue.append(tb)
            self.get_logger().info(f'Added {tb} to the queue.')

    def process_queue(self):
        """
        - state가 'IDLE' 이고
        - arm이 port에 있고 비어있으며
        - queue에 무언가 있으면 → 큐 맨 앞의 TurtleBot 처리 시작
        """
        if (
            self.state == 'IDLE'
            and self.arm_at_port
            and not self.arm_busy
            and self.tb_queue
        ):
            self.current_tb = self.tb_queue.popleft()
            self.get_logger().info(f'Starting processing of {self.current_tb}.')
            self.state = 'MOVE_TB_FORWARD'
            # 전진 명령(5초)
            self.move_turtlebot(self.current_tb, linear_x=0.3, duration=5)

    def move_turtlebot(self, tb, linear_x, duration):
        """
        TurtleBot 이동 명령 함수. linear_x=0.3 → 전진, -0.3 → 후진
        duration초 동안 10Hz로 명령을 퍼블리시 후 정지
        이동 끝나면 on_board 또는 empty를 로봇팔에 지시
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = 0.0

        if tb == 'tb1':
            publisher = self.tb1_cmd_vel_pub
        else:
            publisher = self.tb2_cmd_vel_pub

        direction = "forward" if linear_x > 0 else "backward"
        self.get_logger().info(f'Moving {tb} {direction} with linear.x = {linear_x} for {duration} seconds.')

        # duration초 동안 10Hz(0.1초 간격)로 이동 명령 퍼블리시
        iterations = int(duration * 10)
        for _ in range(iterations):
            publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 정지
        twist.linear.x = 0.0
        publisher.publish(twist)
        self.get_logger().info(f'{tb} stopped moving.')

        # 전진 후 → on_board
        # 후진 후 → empty
        if linear_x > 0:
            self.state = 'ARM_ON_BOARD'
            self.arm_busy = True
            # 로봇팔에 on board 명령
            self.publish_arm_command(f'on board {self.destination}')
            # 목적지 publish (예: 8)
            self.publish_destination(self.destination)

        elif linear_x < 0:
            self.state = 'ARM_EMPTY'
            self.arm_busy = True
            # 로봇팔에 empty 명령
            self.publish_arm_command('empty')
            # 주차 완료 후 port로 되돌리기 위해 destination=0
            self.publish_destination(0)

    def publish_arm_command(self, command: str):
        """로봇팔 명령 퍼블리시"""
        msg = String()
        msg.data = command
        self.arm_command_pub.publish(msg)
        self.get_logger().info(f'Published to /arm_commands: "{command}"')

    def publish_destination(self, value: int):
        """로봇팔 목적지(선착장=0, 그 외=1~9 등)를 /destination 토픽으로 퍼블리시"""
        msg = Int32()
        msg.data = value
        self.destination_pub.publish(msg)
        self.get_logger().info(f'Published to /destination: {value}')

    def handle_arm_at_tower(self):
        """
        로봇팔이 tower에 도착했다는 것은
        → on board 완료 상태(ARM_ON_BOARD)에서 터틀봇 주차 완료 시점을 의미.
        → 그 다음에는 TurtleBot을 뒤로 5초 이동(하역) 명령 진행.
        """
        if self.state == 'ARM_ON_BOARD':
            self.get_logger().info(f'Handling arm at tower for {self.current_tb}.')
            self.state = 'MOVE_TB_BACKWARD'
            self.move_turtlebot(self.current_tb, linear_x=-0.3, duration=5)

        # 혹은 다른 케이스를 원하면 여기에 elif self.state == 'ARM_MOVE_TO_PORT': 등 추가

    def run(self):
        """메인 스핀 루프"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # state에 따라 추가로 처리할 것이 있다면 여기에 적어준다.
            # 현재는 모든 처리는 콜백 함수 내에서 처리.

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

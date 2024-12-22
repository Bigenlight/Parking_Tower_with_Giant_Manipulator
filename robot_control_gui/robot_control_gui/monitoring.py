import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from collections import deque

class SkynetNode(Node):
    def __init__(self):
        super().__init__('skynet_node')
        
        # ----------------------
        #      Subscribers
        # ----------------------
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

        # ----------------------
        #      Publishers
        # ----------------------
        self.tb1_cmd_vel_pub = self.create_publisher(Twist, '/tb1/cmd_vel', 10)
        self.tb2_cmd_vel_pub = self.create_publisher(Twist, '/tb2/cmd_vel', 10)
        self.destination_pub = self.create_publisher(Int32, '/destination', 10)

        # ----------------------
        #      State Vars
        # ----------------------
        self.tb_queue = deque()      # tb1, tb2 대기열 (tb1 우선)
        self.current_tb = None       # 현재 옮길 터틀봇
        self.destination = 1         # 기본 목적지 (1~9)
        self.state = 'IDLE'          # FSM (IDLE, MOVE_TB_FORWARD, ARM_ON_BOARD, MOVE_TB_BACKWARD, ARM_EMPTY 등)

        # 로봇팔 상태
        self.arm_at_port = False     # 로봇팔이 선착장에 있는지
        self.arm_busy = False        # 로봇팔이 바쁜 상태인지(옮기는 중)

        # 단순 표시용: tb1, tb2가 '도착'했다고 알려왔는지
        self.tb1_arrived = False
        self.tb2_arrived = False

        self.get_logger().info('Skynet Node Initialized.')

    # -----------------------------------------------------------
    #   Subscriber Callbacks
    # -----------------------------------------------------------
    def turtlebot_state_callback(self, msg: String):
        """
        /turtlebot_state 로부터
        'tb1 arrived' or 'tb2 arrived' 등의 데이터를 수신
        """
        self.get_logger().info(f'[Skynet] Received turtlebot_state: "{msg.data}"')

        if msg.data == 'tb1 arrived':
            if not self.tb1_arrived:
                self.tb1_arrived = True
                self.add_to_queue('tb1')

        elif msg.data == 'tb2 arrived':
            if not self.tb2_arrived:
                self.tb2_arrived = True
                self.add_to_queue('tb2')

        # 새로 도착한 터틀봇이 있으면 큐를 처리해본다
        self.process_queue()

    def arm_state_callback(self, msg: String):
        """
        /arm_state 로부터
        'port arrived' or 'tower arrived' 를 수신
        """
        self.get_logger().info(f'[Skynet] Received arm_state: "{msg.data}"')

        if msg.data == 'port arrived':
            # 로봇팔이 선착장에 도착
            self.arm_at_port = True
            self.arm_busy = False
            self.state = 'IDLE'
            self.get_logger().info('[Skynet] Robot arm has arrived at the port. (state=IDLE)')

            # 이제 새 터틀봇을 처리할 수 있으면 처리
            self.process_queue()

        elif msg.data == 'tower arrived':
            # 로봇팔이 주차장(tower)에 도착
            self.get_logger().info('[Skynet] Robot arm has arrived at the tower.')
            self.handle_arm_at_tower()

    # -----------------------------------------------------------
    #   Internal Logic
    # -----------------------------------------------------------
    def add_to_queue(self, tb: str):
        """
        tb1이 먼저 도착하면 맨 앞에,
        그 외(tb2)는 뒤에 추가. (tb1 우선)
        """
        if tb not in self.tb_queue:
            if tb == 'tb1':
                self.tb_queue.appendleft(tb)
            else:
                self.tb_queue.append(tb)
            self.get_logger().info(f'[Skynet] Added {tb} to the queue. Current queue: {list(self.tb_queue)}')

    def process_queue(self):
        """
        - state가 'IDLE' 이고
        - arm이 port에 있고, 현재 바쁘지 않으면
        - 대기열에서 하나를 꺼내 전진 → 목적지 → 후진 → port 복귀
        """
        if (
            self.state == 'IDLE'
            and self.arm_at_port
            and not self.arm_busy
            and self.tb_queue
        ):
            # 큐에서 맨 앞의 터틀봇 꺼내서 처리 시작
            self.current_tb = self.tb_queue.popleft()
            self.get_logger().info(f'[Skynet] Start moving {self.current_tb}. (state=MOVE_TB_FORWARD)')

            self.state = 'MOVE_TB_FORWARD'
            self.arm_busy = True  # 로봇팔에서 새 작업을 시작
            self.move_turtlebot(self.current_tb, linear_x=2.0, duration=10)

    def move_turtlebot(self, tb: str, linear_x: float, duration: float):
        """
        TurtleBot 이동 명령.
        - linear_x > 0 : 전진
        - linear_x < 0 : 후진
        - duration 초 동안 10Hz로 이동 명령 퍼블리시 후 정지
        이동 완료 후 상태에 따라 다음 단계 진행
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = 0.0

        if tb == 'tb1':
            publisher = self.tb1_cmd_vel_pub
        else:
            publisher = self.tb2_cmd_vel_pub

        direction = "forward" if linear_x > 0 else "backward"
        self.get_logger().info(
            f'[Skynet] Moving {tb} {direction} (x={linear_x}) for {duration} seconds.'
        )

        # duration초 동안 10Hz(0.1s 간격)로 이동 명령 퍼블리시
        iterations = int(duration * 10)
        for _ in range(iterations):
            publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 이동 종료 -> 정지
        twist.linear.x = 0.0
        publisher.publish(twist)
        self.get_logger().info(f'[Skynet] {tb} stopped moving.')

        # ---------------------------
        #   이후 동작 결정
        # ---------------------------
        if linear_x > 0:
            # 전진 → 'ARM_ON_BOARD' 단계
            self.state = 'ARM_ON_BOARD'
            # 로봇팔이 주차장으로 이동할 목적지 퍼블리시 (ex. 8)
            self.publish_destination(self.destination)
            self.get_logger().info(
                f'[Skynet] TurtleBot on board. Next destination = {self.destination} (ARM_ON_BOARD)'
            )

        else:
            # 후진 → 'ARM_EMPTY' 단계
            self.state = 'ARM_EMPTY'
            # 주차 완료 후 port로 되돌리기 위해 destination=0
            self.publish_destination(0)
            self.get_logger().info(
                '[Skynet] TurtleBot empty. Returning arm to port. (ARM_EMPTY -> /destination=0)'
            )

    def publish_destination(self, value: int):
        """
        로봇팔 목적지(0=port, 1~9=tower) 를 /destination 토픽으로 퍼블리시
        """
        msg = Int32()
        msg.data = value
        self.destination_pub.publish(msg)
        self.get_logger().info(f'[Skynet] Published /destination: {value}')

    def handle_arm_at_tower(self):
        """
        /arm_state = 'tower arrived' 를 수신했을 때,
        ARM_ON_BOARD 상태였다면 → TurtleBot을 후진시켜서 내려놓는다.
        """
        if self.state == 'ARM_ON_BOARD':
            self.get_logger().info(f'[Skynet] (ARM_ON_BOARD) -> Move {self.current_tb} backward to unload.')
            self.state = 'MOVE_TB_BACKWARD'
            self.move_turtlebot(self.current_tb, linear_x=-2.0, duration=10)

    # -----------------------------------------------------------
    #   Main Loop
    # -----------------------------------------------------------
    def run(self):
        """
        메인 스핀 루프
        콜백 위주로 동작, 상태 전환은 콜백들 안에서 진행
        """
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # 필요 시 추가 로직 가능

def main(args=None):
    rclpy.init(args=args)
    node = SkynetNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('[Skynet] Stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

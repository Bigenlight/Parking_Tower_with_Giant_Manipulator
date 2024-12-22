#!/usr/bin/env python3

import os
# Qt 플랫폼 플러그인 경로를 PyQt5 및 OpenCV 임포트 전에 설정
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'

import sys
import threading
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from collections import deque

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QLabel, QGraphicsView, QGraphicsScene, QMessageBox,
    QSizePolicy, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QObject, pyqtSignal
from PyQt5.QtGui import QPixmap, QPen, QBrush, QColor, QPainter, QImage

##############################################################################
# 1) Monitoring 코드 (ControlNode)
##############################################################################
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
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)

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

    def run(self):
        """메인 스핀 루프"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # state에 따라 추가로 처리할 것이 있다면 여기에 적어준다.
            # 현재는 모든 처리는 콜백 함수 내에서 처리.

##############################################################################
# 2) RobotControlNode + PyQt GUI
##############################################################################
class ImageSignal(QObject):
    image_signal = pyqtSignal(str, QImage)

class RobotControlNode(Node):
    """
    - tb1, tb2 각각 odom, camera/image_raw를 구독
    - 카메라 이미지를 QImage로 변환하여 MapWindow에 송신
    """
    def __init__(self, robots, image_signal):
        super().__init__('robot_control_node')
        self.robots = robots
        self.image_signal = image_signal
        self.bridge = CvBridge()
        self.current_pose = {r['name']: None for r in robots}

        for robot in self.robots:
            odom_topic = f'/{robot["name"]}/odom'
            self.create_subscription(
                Odometry,
                odom_topic,
                self.create_odom_callback(robot['name']),
                10
            )
            image_topic = f'/{robot["name"]}/camera/image_raw'
            self.create_subscription(
                Image,
                image_topic,
                self.create_image_callback(robot['name']),
                10
            )

    def create_odom_callback(self, name):
        def cb(msg: Odometry):
            self.current_pose[name] = msg.pose.pose
        return cb

    def create_image_callback(self, name):
        def cb(msg: Image):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, c = cv_img.shape
                bytes_line = c * w
                q_img = QImage(cv_img.data, w, h, bytes_line, QImage.Format_RGB888)
                self.image_signal.image_signal.emit(name, q_img)
            except Exception as e:
                self.get_logger().error(f"Image conversion error[{name}]: {e}")
        return cb

##############################################################################
# 3) PyQt GUI (MapWindow)
##############################################################################
class MapWindow(QMainWindow):
    def __init__(self, robot_control_node, monitoring_node, robots):
        super().__init__()
        self.setWindowTitle("로봇 제어 GUI")
        self.setGeometry(100, 100, 1600, 800)

        self.robot_control_node = robot_control_node
        self.monitoring_node = monitoring_node
        self.robots = robots

        # 메인 레이아웃
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)

        # 카메라 레이아웃
        self.camera_layout = QVBoxLayout()
        self.camera_labels = {}
        for robot in self.robots:
            label = QLabel(robot['name'])
            label.setAlignment(Qt.AlignCenter)
            self.camera_layout.addWidget(label)

            cam_label = QLabel()
            cam_label.setFixedSize(400, 300)
            cam_label.setStyleSheet("background-color: black;")
            cam_label.setAlignment(Qt.AlignCenter)
            self.camera_layout.addWidget(cam_label)
            self.camera_labels[robot['name']] = cam_label

        # 맵
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView(self.map_scene)
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # 지도 로드
        self.map_loaded = False
        map_yaml_path = "/home/rokey/7_ws/src/robot_control_gui/maps/map.yaml"  # 실제 yaml 경로
        self.load_map(map_yaml_path)

        self.robot_items = {}
        for robot in robots:
            color = QColor(robot.get('color', 'green'))
            item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(color), QBrush(color))
            item.setZValue(1)
            self.robot_items[robot['name']] = item

        # 레이아웃 배치
        self.main_layout.addLayout(self.camera_layout)
        self.main_layout.addWidget(self.map_view, stretch=3)

        # 주차 위치 버튼(1~9)
        self.button_widget = QWidget()
        self.button_grid_layout = QGridLayout()
        self.button_widget.setLayout(self.button_grid_layout)
        self.buttons = []
        for i in range(9):
            btn_num = i + 1
            button = QPushButton(str(btn_num))
            button.setCheckable(True)
            button.setFixedSize(60, 60)
            if btn_num == 1:
                button.setStyleSheet("background-color: yellow")
                button.setChecked(True)
                self.monitoring_node.destination = 1
            else:
                button.setChecked(False)
            button.clicked.connect(self.handle_button_click)

            row, col = divmod(i, 3)
            self.button_grid_layout.addWidget(button, row, col)
            self.buttons.append(button)

        self.main_layout.addWidget(self.button_widget)

        # 타이머: 로봇 위치 업데이트
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_positions)
        self.update_timer.start(100)

        # RobotControlNode -> 시그널 연결 (카메라 이미지를 수신하여 표시)
        self.robot_control_node.image_signal.image_signal.connect(self.update_image)

    def load_map(self, yaml_path):
        if not os.path.exists(yaml_path):
            QMessageBox.critical(self, "Error", f"Map YAML not found: {yaml_path}")
            return
        with open(yaml_path, 'r') as f:
            try:
                map_data = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                QMessageBox.critical(self, "Error", f"Failed to read yaml: {exc}")
                return

        img_path = os.path.join(os.path.dirname(yaml_path), map_data.get('image', 'map.pgm'))
        resolution = map_data.get('resolution', 0.05)
        origin = map_data.get('origin', [0.0, 0.0, 0.0])

        self.map_scale = 1.0 / resolution
        self.map_origin = (origin[0], origin[1])

        if not os.path.exists(img_path):
            QMessageBox.critical(self, "Error", f"Map image not found: {img_path}")
            return

        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            QMessageBox.critical(self, "Error", f"Failed to load map image: {img_path}")
            return

        scale_factor = 0.5
        scaled_pixmap = pixmap.scaled(
            int(pixmap.width() * scale_factor),
            int(pixmap.height() * scale_factor),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.map_scene.clear()
        self.map_scene.addPixmap(scaled_pixmap)
        self.map_scene.setSceneRect(QRectF(scaled_pixmap.rect()))
        self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

        self.map_loaded = True

    def handle_button_click(self):
        clicked = self.sender()
        if clicked.isChecked():
            for btn in self.buttons:
                if btn == clicked:
                    btn.setStyleSheet("background-color: yellow")
                    dest_value = int(btn.text())
                    self.monitoring_node.destination = dest_value
                    self.monitoring_node.get_logger().info(f"Destination set to: {dest_value}")
                else:
                    btn.setStyleSheet("")
                    btn.setChecked(False)
        else:
            # 체크 해제 방지
            clicked.setChecked(True)

    def update_positions(self):
        if not self.map_loaded:
            return
        for robot in self.robots:
            name = robot['name']
            pose = self.robot_control_node.current_pose.get(name)
            if pose:
                x = pose.position.x
                y = pose.position.y
                self.update_robot_on_map(name, x, y)

    def update_robot_on_map(self, name, x, y):
        # (x, y) → 픽셀 좌표 변환
        pixel_x = (x - self.map_origin[0]) * self.map_scale * 0.5 + self.map_view.width() / 2
        pixel_y = (-y + self.map_origin[1]) * self.map_scale * 0.5 + self.map_view.height() / 2
        if name in self.robot_items:
            self.robot_items[name].setPos(pixel_x, pixel_y)

    def update_image(self, robot_name, q_image):
        if robot_name in self.camera_labels:
            pixmap = QPixmap.fromImage(q_image)
            scaled = pixmap.scaled(
                self.camera_labels[robot_name].width(),
                self.camera_labels[robot_name].height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.camera_labels[robot_name].setPixmap(scaled)

##############################################################################
# 4) 메인 함수: GUI(메인 스레드) + ROS 노드(spin 실행)
##############################################################################
def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # 1) 노드 생성
    monitoring_node = ControlNode()

    # 로봇 목록
    robots = [
        {'name': 'tb1', 'color': 'red'},
        {'name': 'tb2', 'color': 'blue'},
    ]
    image_signal = ImageSignal()
    robot_control_node = RobotControlNode(robots, image_signal)

    # 2) 멀티스레드 실행기
    executor = MultiThreadedExecutor()
    executor.add_node(monitoring_node)
    executor.add_node(robot_control_node)

    # 3) 실행기를 별도 스레드에서 spin
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 4) 메인 스레드에서 PyQt GUI 실행
    app = QApplication(sys.argv)
    window = MapWindow(robot_control_node, monitoring_node, robots)
    window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        monitoring_node.destroy_node()
        robot_control_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import sys
import os
import threading
import queue
from collections import deque

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import cv2

# ROS 메시지
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# PyQt5
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGraphicsView, QGraphicsScene, QMessageBox, QMainWindow, QSizePolicy,
    QGridLayout
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer, QRectF
from PyQt5.QtGui import QPixmap, QPen, QBrush, QColor, QPainter, QImage, QFont


# -------------------------------------------------
#   ImageSignal: PyQt 신호 정의 (카메라 영상용)
# -------------------------------------------------
class ImageSignal(QObject):
    # 로봇 이름과 QImage를 함께 전달
    image_signal = pyqtSignal(str, QImage)


# -------------------------------------------------
#   SkynetNode: 터틀봇/로봇팔 제어 및 FSM
# -------------------------------------------------
class SkynetNode(Node):
    def __init__(self, map_window=None):
        super().__init__('skynet_node')

        # GUI(MapWindow) 객체 참조 (버튼 활성/비활성 제어를 위해)
        self.map_window = map_window  

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

        # GUI 버튼으로부터 설정되는 목적지 (1~9), 기본값 1
        self.destination = 1

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

            # 로봇팔 이동이 끝났으므로, 버튼들 활성화
            if self.map_window:
                self.map_window.enable_destination_buttons()

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

            # 로봇팔 이동이 시작되므로, 버튼들 비활성화
            if self.map_window:
                self.map_window.disable_destination_buttons()

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
            # 로봇팔이 주차장으로 이동할 목적지 퍼블리시 (현재 self.destination)
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
    #   외부(예: GUI)에서 destination 업데이트
    # -----------------------------------------------------------
    def set_destination(self, new_dest: int):
        """
        GUI에서 누른 버튼 번호를 통해 destination 설정
        로봇팔이 이동 중(arm_busy=True)이면 변경 불가하도록 처리할 수도 있음
        """
        if self.arm_busy:
            # 이미 이동 중이면 로그만 찍고 무시 (또는 다른 정책 적용 가능)
            self.get_logger().warn("[Skynet] Arm is busy. Cannot change destination now.")
            return

        self.destination = new_dest
        self.get_logger().info(f"[Skynet] Destination updated to: {self.destination}")


# -------------------------------------------------
#   RobotControlNode: Odom/Camera 수신
# -------------------------------------------------
class RobotControlNode(Node):
    def __init__(self, robots, image_signal):
        super().__init__('robot_control_node')
        self.image_signal = image_signal
        self.robots = robots  # 로봇 딕셔너리 리스트
        self.bridge = CvBridge()  # CvBridge 인스턴스 생성

        # 로봇의 현재 위치 저장 변수
        self.current_pose = {robot['name']: None for robot in self.robots}

        # Subscribers
        for robot in self.robots:
            # Odom subscriber
            odom_topic = f'/{robot["name"]}/odom'
            self.create_subscription(
                Odometry,
                odom_topic,
                self.create_odom_callback(robot['name']),
                10
            )
            # Image subscriber
            image_topic = f'/{robot["name"]}/camera/image_raw'
            self.create_subscription(
                Image,
                image_topic,
                self.create_image_callback(robot['name']),
                10
            )

    def create_odom_callback(self, robot_name):
        def odom_callback(msg: Odometry):
            self.current_pose[robot_name] = msg.pose.pose
            self.get_logger().debug(
                f"현재 로봇 {robot_name} 위치: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}"
            )
        return odom_callback

    def create_image_callback(self, robot_name):
        def image_callback(msg: Image):
            try:
                # ROS Image 메시지를 OpenCV 이미지로 변환
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                # OpenCV 이미지를 RGB로 변환
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                # QImage로 변환
                height, width, channel = cv_image.shape
                bytes_per_line = 3 * width
                q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                # QImage 신호 전송
                self.image_signal.image_signal.emit(robot_name, q_img)
            except Exception as e:
                self.get_logger().error(f"이미지 변환 오류 for {robot_name}: {e}")
        return image_callback

    def close(self):
        pass  # 필요한 종료 작업이 있다면 추가


# -------------------------------------------------
#   MapWindow: 지도, 카메라, 목적지 버튼 GUI
# -------------------------------------------------
class MapWindow(QMainWindow):
    def __init__(self, skynet_node, robot_node, robots):
        super().__init__()
        # ROS 노드 참조
        self.skynet_node = skynet_node
        self.robot_node = robot_node

        # 로봇 리스트
        self.robots = robots

        self.setWindowTitle("로봇 제어 GUI")
        self.setGeometry(100, 100, 1600, 800)

        # 메인 위젯 & 레이아웃
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)

        # ------------------------------------------------
        #   카메라 레이아웃 (왼쪽)
        # ------------------------------------------------
        self.camera_layout = QVBoxLayout()
        self.camera_layout.setSpacing(10)
        self.camera_layout.setContentsMargins(10, 10, 10, 10)

        self.camera_labels = {}
        for robot in self.robots:
            # 로봇 이름 레이블
            label = QLabel(f"{robot['name']}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-weight: bold; font-size: 12px;")
            self.camera_layout.addWidget(label)

            # 카메라 이미지 레이블
            camera_label = QLabel()
            camera_label.setFixedSize(400, 300)
            camera_label.setStyleSheet("background-color: black;")
            camera_label.setAlignment(Qt.AlignCenter)
            self.camera_layout.addWidget(camera_label)

            self.camera_labels[robot['name']] = camera_label

        # ------------------------------------------------
        #   지도 표시 (QGraphicsView)
        # ------------------------------------------------
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView(self.map_scene)
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 실제 map.yaml 경로로 수정하세요
        self.load_map("/home/rokey/7_ws/src/robot_control_gui/maps/map.yaml")

        # 로봇 아이콘 추가
        self.robot_items = {}
        for robot in self.robots:
            color = QColor(robot.get('color', 'green'))  # 기본 green
            robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(color), QBrush(color))
            robot_item.setZValue(1)
            self.robot_items[robot['name']] = robot_item

        # ------------------------------------------------
        #   목적지 버튼들 (오른쪽)
        # ------------------------------------------------
        self.button_widget = QWidget()
        self.button_grid_layout = QGridLayout()
        self.button_widget.setLayout(self.button_grid_layout)
        self.buttons = []

        # 초기에 선택된 목적지를 SkynetNode에서 가져온다
        self.destination = self.skynet_node.destination  

        for i in range(9):
            button_number = i + 1
            button = QPushButton(str(button_number))
            button.setCheckable(True)
            button.setFixedSize(60, 60)

            # SkynetNode의 기본 destination과 동일하면 체크
            if button_number == self.destination:
                button.setStyleSheet("background-color: yellow")
                button.setChecked(True)
            else:
                button.setStyleSheet("")
                button.setChecked(False)

            button.clicked.connect(self.handle_button_click)
            row = i // 3
            col = i % 3
            self.button_grid_layout.addWidget(button, row, col)
            self.buttons.append(button)

        # ------------------------------------------------
        #   메인 레이아웃에 위젯들 배치
        # ------------------------------------------------
        self.main_layout.addLayout(self.camera_layout)
        self.main_layout.addWidget(self.map_view, stretch=3)
        self.main_layout.addWidget(self.button_widget)

        # 맵 로드 완료 여부 플래그
        self.map_loaded = True

        # 로봇 위치 업데이트 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_position)
        self.timer.start(100)  # 100ms

        # RobotControlNode에서 카메라 이미지를 받아오는 신호 연결
        self.robot_node.image_signal.image_signal.connect(self.update_camera_image)

    # ------------------------------------------------
    #   지도 로드
    # ------------------------------------------------
    def load_map(self, map_yaml_path):
        if not os.path.exists(map_yaml_path):
            self.skynet_node.get_logger().error(f"지도 yaml 파일을 찾을 수 없습니다: {map_yaml_path}")
            QMessageBox.critical(self, "오류", f"지도 yaml 파일을 찾을 수 없습니다: {map_yaml_path}")
            return

        with open(map_yaml_path, 'r') as file:
            try:
                map_data = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                self.skynet_node.get_logger().error(f"지도 yaml 파일 읽기 오류: {exc}")
                QMessageBox.critical(self, "오류", f"지도 yaml 파일 읽기 오류: {exc}")
                return

        map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_data.get('image', 'map.pgm'))
        resolution = map_data.get('resolution', 0.05)
        origin = map_data.get('origin', [0.0, 0.0, 0.0])

        # 해상도(픽셀/미터)
        self.map_scale = 1.0 / resolution
        # 지도 원점
        self.map_origin = (origin[0], origin[1])

        if not os.path.exists(map_image_path):
            self.skynet_node.get_logger().error(f"지도 이미지 파일을 찾을 수 없습니다: {map_image_path}")
            QMessageBox.critical(self, "오류", f"지도 이미지 파일을 찾을 수 없습니다: {map_image_path}")
            return

        pixmap = QPixmap(map_image_path)
        if pixmap.isNull():
            self.skynet_node.get_logger().error(f"지도 이미지 로드 실패: {map_image_path}")
            QMessageBox.critical(self, "오류", f"지도 이미지 로드 실패: {map_image_path}")
            return

        # 맵 이미지 축소 비율
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

        self.map_loaded = True
        self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

        self.skynet_node.get_logger().info(f"맵 로드 완료: {map_image_path}")
        self.skynet_node.get_logger().info(f"해상도: {resolution} m/pixel, 원점: {origin}")
        self.skynet_node.get_logger().info(f"맵 스케일: {self.map_scale} 픽셀/meter, 맵 원점: {self.map_origin}")

    # ------------------------------------------------
    #   버튼 클릭 처리
    # ------------------------------------------------
    def handle_button_click(self):
        clicked_button = self.sender()
        if clicked_button.isChecked():
            # 다른 버튼들 해제
            for button in self.buttons:
                if button == clicked_button:
                    button.setStyleSheet("background-color: yellow")
                    self.destination = int(button.text())
                else:
                    button.setStyleSheet("")
                    button.setChecked(False)

            # SkynetNode에 destination 전달
            self.skynet_node.set_destination(self.destination)
        else:
            # 체크 해제 방지 (최소 1개 버튼은 always-checked 상태)
            clicked_button.setChecked(True)

    # ------------------------------------------------
    #   로봇 위치 업데이트 (100ms)
    # ------------------------------------------------
    def update_robot_position(self):
        for robot in self.robots:
            robot_name = robot['name']
            pose = self.robot_node.current_pose.get(robot_name)
            if pose:
                x = pose.position.x
                y = pose.position.y
                self.update_robot_position_on_map(x, y, robot_name)

    def update_robot_position_on_map(self, x, y, robot_name):
        if not self.map_loaded:
            return
        # 실제 좌표를 픽셀 좌표로 변환
        pixel_x = (x - self.map_origin[0]) * self.map_scale * 0.5 + self.map_view.width() / 2
        pixel_y = (-y + self.map_origin[1]) * self.map_scale * 0.5 + self.map_view.height() / 2
        self.robot_items[robot_name].setPos(pixel_x, pixel_y)

    # ------------------------------------------------
    #   카메라 영상 갱신
    # ------------------------------------------------
    def update_camera_image(self, robot_name, q_img):
        if robot_name in self.camera_labels:
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaled(
                self.camera_labels[robot_name].width(),
                self.camera_labels[robot_name].height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.camera_labels[robot_name].setPixmap(scaled_pixmap)

    # ------------------------------------------------
    #   버튼 비활성화/활성화
    # ------------------------------------------------
    def disable_destination_buttons(self):
        for button in self.buttons:
            button.setEnabled(False)

    def enable_destination_buttons(self):
        for button in self.buttons:
            button.setEnabled(True)
        # 마지막으로 눌린 destination 버튼 찾아서 다시 표시
        for button in self.buttons:
            if int(button.text()) == self.destination:
                button.setChecked(True)
                button.setStyleSheet("background-color: yellow")
            else:
                button.setChecked(False)
                button.setStyleSheet("")


# -------------------------------------------------
#   메인 함수
# -------------------------------------------------
def ros_spin(node_list):
    """
    여러 노드를 spin_once로 순회하며 콜백 처리
    """
    while rclpy.ok():
        for n in node_list:
            rclpy.spin_once(n, timeout_sec=0.1)


def main(args=None):
    # Qt 플랫폼 플러그인 경로 설정 (환경에 맞게 수정)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'
    # 필요시 SHM Transport 비활성화
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    rclpy.init(args=args)

    # 카메라 표시용 PyQt 시그널
    image_signal = ImageSignal()

    # 로봇 정의
    robots = [
        {'name': 'tb1', 'color': 'red'},
        {'name': 'tb2', 'color': 'blue'},
    ]

    # RobotControlNode 생성
    robot_node = RobotControlNode(robots, image_signal)

    # SkynetNode 생성 (초기엔 GUI를 모름, 일단 None)
    skynet_node = SkynetNode(map_window=None)

    # 두 노드를 함께 spin하기 위한 리스트
    node_list = [robot_node, skynet_node]

    # ROS 스레드 실행
    ros_thread = threading.Thread(target=ros_spin, args=(node_list,), daemon=True)
    ros_thread.start()

    # PyQt Application
    app = QApplication(sys.argv)

    # MapWindow 생성 (skynet_node, robot_node 참조)
    map_window = MapWindow(skynet_node, robot_node, robots)
    map_window.show()

    # SkynetNode도 MapWindow 참조 (양방향 연결)
    skynet_node.map_window = map_window

    # Qt 메인루프 실행
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        skynet_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()


if __name__ == "__main__":
    main()

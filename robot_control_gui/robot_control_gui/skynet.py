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

# QoS 관련 임포트
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


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
    QGridLayout, QFrame
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
        #   QoS 설정 (중복 메시지 방지)
        # ----------------------
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # ----------------------
        #      Subscribers
        # ----------------------
        self.turtlebot_sub = self.create_subscription(
            String,
            '/turtlebot_state',
            self.turtlebot_state_callback,
            qos_profile  # ← QoS 적용
        )
        self.arm_sub = self.create_subscription(
            String,
            '/arm_state',
            self.arm_state_callback,
            qos_profile  # ← QoS 적용
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

        # 기본 destination (기존 코드를 유지)
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

            # 실제 이동 시작
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

        publisher = self.tb1_cmd_vel_pub if tb == 'tb1' else self.tb2_cmd_vel_pub

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

        # 이후 동작 결정
        if linear_x > 0:
            # 전진 → 'ARM_ON_BOARD' 단계
            self.state = 'ARM_ON_BOARD'
            # 수정: 이동시키는 터틀봇(tb1/tb2)의 버튼 번호로 destination을 퍼블리시
            self.publish_destination(going_port=False)

            self.get_logger().info(
                f'[Skynet] TurtleBot on board. Next destination = (based on {tb} button)'
            )
        else:
            # 후진 → 'ARM_EMPTY' 단계
            self.state = 'ARM_EMPTY'
            # 주차 완료 후 port로 되돌리기 위해 destination=0
            self.publish_destination(going_port=True)
            self.get_logger().info(
                '[Skynet] TurtleBot empty. Returning arm to port. (ARM_EMPTY -> /destination=0)'
            )

    def publish_destination(self, going_port: bool):
        """
        going_port=True 이면 port(=0)로 이동,
        going_port=False 이면 현재 이동 중인 터틀봇(tb1/tb2)의 버튼 번호로 이동

        즉, tb1이면 MapWindow.selected_tb1,
            tb2이면 MapWindow.selected_tb2 를 퍼블리시
        """
        if going_port:
            # port
            msg = Int32()
            msg.data = 0
            self.destination_pub.publish(msg)
            self.get_logger().info('[Skynet] Published /destination: 0')
        else:
            # tower
            if self.current_tb == 'tb1':
                # tb1 선택 번호
                value = self.map_window.selected_tb1 if self.map_window else self.destination
            else:
                # tb2 선택 번호
                value = self.map_window.selected_tb2 if self.map_window else self.destination

            msg = Int32()
            msg.data = value
            self.destination_pub.publish(msg)
            self.get_logger().info(f'[Skynet] Published /destination: {value}')

    def handle_arm_at_tower(self):
        """
        /arm_state = 'tower arrived' -> ARM_ON_BOARD인 경우 후진해서 내려놓기
        """
        if self.state == 'ARM_ON_BOARD':
            self.get_logger().info(f'[Skynet] (ARM_ON_BOARD) -> Move {self.current_tb} backward to unload.')
            self.state = 'MOVE_TB_BACKWARD'
            self.move_turtlebot(self.current_tb, linear_x=-2.0, duration=10)

    def set_destination(self, new_dest: int):
        """
        기존 코드: GUI에서 누른 버튼 번호를 통해 destination 설정.
        (실제 publish는 publish_destination()에서 current_tb 체크)
        """
        if self.arm_busy:
            self.get_logger().warn("[Skynet] Arm is busy. Cannot change destination now.")
            return
        self.destination = new_dest
        self.get_logger().info(f"[Skynet] Destination updated to: {self.destination}")


# -------------------------------------------------
#   RobotControlNode: Odom/Camera 수신
#   (이미지 토픽을 image_raw/compressed 로 변경)
# -------------------------------------------------
class RobotControlNode(Node):
    def __init__(self, robots, image_signal):
        super().__init__('robot_control_node')
        self.image_signal = image_signal
        self.robots = robots
        self.bridge = CvBridge()

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
            # *** 여기서 image_raw → image_raw/compressed 로 변경
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
                # 압축된 ROS Image 메시지도 CvBridge로 변환 가능(encoding 유의)
                # 여기서는 bgr8로 디코딩 시도
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                height, width, channel = cv_image.shape
                bytes_per_line = 3 * width
                q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                self.image_signal.image_signal.emit(robot_name, q_img)
            except Exception as e:
                self.get_logger().error(f"이미지 변환 오류 for {robot_name}: {e}")
        return image_callback

    def close(self):
        pass


# -------------------------------------------------
#   MapWindow: 지도, 카메라, 목적지 버튼 GUI
# -------------------------------------------------
class MapWindow(QMainWindow):
    def __init__(self, skynet_node, robot_node, robots):
        super().__init__()
        # ROS 노드 참조
        self.skynet_node = skynet_node
        self.robot_node = robot_node

        self.robots = robots

        self.setWindowTitle("로봇 제어 GUI")
        self.setGeometry(100, 100, 1481, 741)

        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)

        # -------------------------
        #   카메라 레이아웃 (왼쪽)
        # -------------------------
        self.camera_layout = QVBoxLayout()
        self.camera_layout.setSpacing(10)
        self.camera_layout.setContentsMargins(10, 10, 10, 10)

        self.camera_labels = {}
        for robot in self.robots:
            label = QLabel(f"{robot['name']}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-weight: bold; font-size: 12px;")
            self.camera_layout.addWidget(label)

            camera_label = QLabel()
            camera_label.setFixedSize(400, 300)
            camera_label.setStyleSheet("background-color: black;")
            camera_label.setAlignment(Qt.AlignCenter)
            self.camera_layout.addWidget(camera_label)
            self.camera_labels[robot['name']] = camera_label

        # -------------------------
        #   지도 표시
        # -------------------------
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView(self.map_scene)
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 실제 map.yaml 경로로 수정 필요
        self.load_map("/home/rokey/7_ws/src/robot_control_gui/maps/map.yaml")

        self.robot_items = {}
        for robot in self.robots:
            color = QColor(robot.get('color', 'green'))
            robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(color), QBrush(color))
            robot_item.setZValue(1)
            self.robot_items[robot['name']] = robot_item

        # -------------------------
        #   오른쪽 칼럼 (tb1/tb2 버튼)
        # -------------------------
        self.right_col_widget = QWidget()
        self.right_col_layout = QVBoxLayout()
        self.right_col_widget.setLayout(self.right_col_layout)

        # tb1
        self.tb1_label = QLabel("tb1")
        self.tb1_label.setAlignment(Qt.AlignCenter)
        self.tb1_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.right_col_layout.addWidget(self.tb1_label)

        self.tb1_buttons_widget = QWidget()
        self.tb1_buttons_layout = QGridLayout(self.tb1_buttons_widget)
        self.tb1_buttons_widget.setLayout(self.tb1_buttons_layout)
        self.buttons_tb1 = []

        # 기본 선택
        self.selected_tb1 = 1

        for i in range(9):
            btn_num = i + 1
            btn = QPushButton(str(btn_num))
            btn.setCheckable(True)
            btn.setFixedSize(60, 60)
            if btn_num == self.selected_tb1:
                btn.setStyleSheet("background-color: yellow")
                btn.setChecked(True)
            else:
                btn.setStyleSheet("")
                btn.setChecked(False)

            btn.clicked.connect(lambda checked, n=btn_num: self.handle_tb1_button_click(n))
            row = i // 3
            col = i % 3
            self.tb1_buttons_layout.addWidget(btn, row, col)
            self.buttons_tb1.append(btn)

        self.right_col_layout.addWidget(self.tb1_buttons_widget)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        self.right_col_layout.addWidget(line)

        # tb2
        self.tb2_label = QLabel("tb2")
        self.tb2_label.setAlignment(Qt.AlignCenter)
        self.tb2_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.right_col_layout.addWidget(self.tb2_label)

        self.tb2_buttons_widget = QWidget()
        self.tb2_buttons_layout = QGridLayout(self.tb2_buttons_widget)
        self.tb2_buttons_widget.setLayout(self.tb2_buttons_layout)
        self.buttons_tb2 = []

        self.selected_tb2 = 2

        for i in range(9):
            btn_num = i + 1
            btn = QPushButton(str(btn_num))
            btn.setCheckable(True)
            btn.setFixedSize(60, 60)
            if btn_num == self.selected_tb2:
                btn.setStyleSheet("background-color: yellow")
                btn.setChecked(True)
            else:
                btn.setStyleSheet("")
                btn.setChecked(False)

            btn.clicked.connect(lambda checked, n=btn_num: self.handle_tb2_button_click(n))
            row = i // 3
            col = i % 3
            self.tb2_buttons_layout.addWidget(btn, row, col)
            self.buttons_tb2.append(btn)

        self.right_col_layout.addWidget(self.tb2_buttons_widget)

        self.main_layout.addLayout(self.camera_layout)
        self.main_layout.addWidget(self.map_view, stretch=3)
        self.main_layout.addWidget(self.right_col_widget)

        self.map_loaded = True

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_position)
        self.timer.start(100)

        self.robot_node.image_signal.image_signal.connect(self.update_camera_image)

        # 초기 상태
        self.skynet_node.set_destination(self.selected_tb1)

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

        self.map_scale = 1.0 / resolution
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
    #   tb1 버튼 클릭
    # ------------------------------------------------
    def handle_tb1_button_click(self, num: int):
        if num == self.selected_tb1:
            return
        # 스타일 업데이트
        for btn in self.buttons_tb1:
            n = int(btn.text())
            if n == num:
                btn.setStyleSheet("background-color: yellow")
                btn.setChecked(True)
            else:
                btn.setStyleSheet("")
                btn.setChecked(False)
        self.selected_tb1 = num

        # 비활성화만 유지, 회색 제거
        self.update_tb2_buttons_state()

        # SkynetNode에 destination 전달 (tb1 이동 시 이 번호 사용)
        self.skynet_node.set_destination(self.selected_tb1)

    # ------------------------------------------------
    #   tb2 버튼 클릭
    # ------------------------------------------------
    def handle_tb2_button_click(self, num: int):
        if num == self.selected_tb2:
            return
        for btn in self.buttons_tb2:
            n = int(btn.text())
            if n == num:
                btn.setStyleSheet("background-color: yellow")
                btn.setChecked(True)
            else:
                btn.setStyleSheet("")
                btn.setChecked(False)
        self.selected_tb2 = num

        self.update_tb1_buttons_state()

        # SkynetNode에 destination 전달 (tb2 이동 시 이 번호 사용)
        self.skynet_node.set_destination(self.selected_tb2)

    # ------------------------------------------------
    #   서로 겹치는 번호 비활성화 (회색 제거)
    # ------------------------------------------------
    def update_tb1_buttons_state(self):
        """
        tb2에서 선택된 숫자(self.selected_tb2)는 tb1에서 비활성화
        나머지는 활성화
        """
        for btn in self.buttons_tb1:
            n = int(btn.text())
            if n == self.selected_tb2:
                btn.setEnabled(False)
                # 버튼 비활성만, 배경색 GRAY 제거
            else:
                btn.setEnabled(True)
                if n == self.selected_tb1:
                    btn.setStyleSheet("background-color: yellow")
                else:
                    btn.setStyleSheet("")

    def update_tb2_buttons_state(self):
        """
        tb1에서 선택된 숫자(self.selected_tb1)는 tb2에서 비활성화
        나머지는 활성화
        """
        for btn in self.buttons_tb2:
            n = int(btn.text())
            if n == self.selected_tb1:
                btn.setEnabled(False)
            else:
                btn.setEnabled(True)
                if n == self.selected_tb2:
                    btn.setStyleSheet("background-color: yellow")
                else:
                    btn.setStyleSheet("")

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
        for btn in self.buttons_tb1:
            btn.setEnabled(False)
        for btn in self.buttons_tb2:
            btn.setEnabled(False)

    def enable_destination_buttons(self):
        self.update_tb1_buttons_state()
        self.update_tb2_buttons_state()


# -------------------------------------------------
#   메인 함수
# -------------------------------------------------
def ros_spin(node_list):
    while rclpy.ok():
        for n in node_list:
            rclpy.spin_once(n, timeout_sec=0.1)


def main(args=None):
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    rclpy.init(args=args)

    image_signal = ImageSignal()
    robots = [
        {'name': 'tb1', 'color': 'red'},
        {'name': 'tb2', 'color': 'blue'},
    ]
    robot_node = RobotControlNode(robots, image_signal)
    skynet_node = SkynetNode(map_window=None)
    node_list = [robot_node, skynet_node]

    ros_thread = threading.Thread(target=ros_spin, args=(node_list,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    map_window = MapWindow(skynet_node, robot_node, robots)
    map_window.show()

    skynet_node.map_window = map_window

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

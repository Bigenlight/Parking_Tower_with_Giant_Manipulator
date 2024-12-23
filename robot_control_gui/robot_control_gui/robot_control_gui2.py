#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
#

import sys
import threading
import queue
import os
import yaml  # YAML 파일을 읽기 위한 패키지 추가
import rclpy
from rclpy.node import Node
# NavigateToPose 및 ActionClient 관련 임포트 제거
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGraphicsView, QGraphicsScene, QMessageBox, QMainWindow, QSizePolicy,
    QGridLayout  # QGridLayout 추가
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer, QRectF
from PyQt5.QtGui import QPixmap, QPen, QBrush, QColor, QPainter, QImage, QFont
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry  # Odometry 메시지 임포트
from sensor_msgs.msg import Image  # Image 메시지 임포트
from cv_bridge import CvBridge  # cv_bridge 임포트
import numpy as np  # numpy 임포트
import cv2  # OpenCV 임포트

class ImageSignal(QObject):
    image_signal = pyqtSignal(str, QImage)  # robot_name and QImage

class RobotControlNode(Node):
    def __init__(self, robots, image_signal):
        super().__init__('robot_control_node')
        self.image_signal = image_signal
        self.robots = robots  # 로봇 딕셔너리 리스트
        self.bridge = CvBridge()  # CvBridge 인스턴스 생성

        # 로봇의 현재 위치 저장 변수 (dict)
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
            # Odometry 메시지에서 Pose 추출
            self.current_pose[robot_name] = msg.pose.pose
            self.get_logger().debug(f"현재 로봇 {robot_name} 위치: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")
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

class MapWindow(QMainWindow):
    def __init__(self, node, robots):
        super().__init__()
        self.node = node
        self.robots = robots  # 로봇 딕셔너리 리스트
        self.setWindowTitle("로봇 제어 GUI")
        self.setGeometry(100, 100, 1600, 800)  # 창 크기 조정 (폭 늘림)

        # 메인 위젯 설정
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)  # 수평 레이아웃

        # 카메라 이미지 표시를 위한 세로 레이아웃
        self.camera_layout = QVBoxLayout()
        self.camera_layout.setSpacing(10)  # 레이아웃 간격 조정
        self.camera_layout.setContentsMargins(10, 10, 10, 10)  # 여백 조정

        self.camera_labels = {}
        for robot in self.robots:
            # 로봇 이름 레이블 설정
            label = QLabel(f"{robot['name']}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-weight: bold; font-size: 12px;")  # 폰트 크기 축소
            self.camera_layout.addWidget(label)

            # 카메라 이미지 레이블 설정
            camera_label = QLabel()
            camera_label.setFixedSize(400, 300)  # 카메라 이미지 크기 확대 (320x240 -> 400x300)
            camera_label.setStyleSheet("background-color: black;")
            camera_label.setAlignment(Qt.AlignCenter)
            self.camera_layout.addWidget(camera_label)

            self.camera_labels[robot['name']] = camera_label

        # 지도 표시를 위한 QGraphicsView와 QGraphicsScene 설정
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView(self.map_scene)
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)  # 가로 스크롤바 제거
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)    # 세로 스크롤바 제거
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # 동적 크기 조정

        # 지도 로드 (map.yaml과 map.pgm 사용)
        self.load_map("/home/rokey/7_ws/src/robot_control_gui/maps/map.yaml")  # 실제 지도 yaml 경로로 변경

        # 로봇 아이콘 추가 (색상별)
        self.robot_items = {}
        for robot in self.robots:
            color = QColor(robot.get('color', 'green'))  # 기본 색상 green
            robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(color), QBrush(color))
            robot_item.setZValue(1)  # 로봇 아이콘을 맵 위에 표시
            self.robot_items[robot['name']] = robot_item

        # 메인 레이아웃에 카메라 이미지와 맵 추가
        self.main_layout.addLayout(self.camera_layout)
        self.main_layout.addWidget(self.map_view, stretch=3)  # 맵에 더 많은 공간 할당

        # 추가: 주차 위치 설정을 위한 버튼 그리드 추가
        self.button_widget = QWidget()
        self.button_grid_layout = QGridLayout()
        self.button_widget.setLayout(self.button_grid_layout)
        self.buttons = []

        for i in range(9):
            button_number = i + 1
            button = QPushButton(str(button_number))
            button.setCheckable(True)
            button.setFixedSize(60, 60)  # 버튼 크기 설정

            if button_number == 1:
                button.setStyleSheet("background-color: yellow")
                button.setChecked(True)
                self.destination = button_number
                print(f"Destination set to: {self.destination}")
            else:
                button.setStyleSheet("")
                button.setChecked(False)

            button.clicked.connect(self.handle_button_click)
            row = i // 3
            col = i % 3
            self.button_grid_layout.addWidget(button, row, col)
            self.buttons.append(button)

        # 버튼 그리드를 메인 레이아웃에 추가
        self.main_layout.addWidget(self.button_widget)

        # 지도 로드가 완료되었는지 확인하는 플래그
        self.map_loaded = True

        # 로봇 위치 업데이트를 위한 타이머 설정 (100ms마다)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_position)
        self.timer.start(100)  # 100ms

        # ImageSignal 연결
        self.node.image_signal.image_signal.connect(self.update_camera_image)

    def load_map(self, map_yaml_path):
        if not os.path.exists(map_yaml_path):
            self.node.get_logger().error(f"지도 yaml 파일을 찾을 수 없습니다: {map_yaml_path}")
            QMessageBox.critical(self, "오류", f"지도 yaml 파일을 찾을 수 없습니다: {map_yaml_path}")
            return

        # map.yaml 읽기
        with open(map_yaml_path, 'r') as file:
            try:
                map_data = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                self.node.get_logger().error(f"지도 yaml 파일 읽기 오류: {exc}")
                QMessageBox.critical(self, "오류", f"지도 yaml 파일 읽기 오류: {exc}")
                return

        # 필요한 정보 추출
        map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_data.get('image', 'map.pgm'))
        resolution = map_data.get('resolution', 0.05)  # 미터/픽셀
        origin = map_data.get('origin', [0.0, 0.0, 0.0])  # [x, y, theta]

        # 해상도를 기반으로 스케일 설정 (픽셀/미터)
        self.map_scale = 1.0 / resolution  # 픽셀/미터

        # 지도 원점 설정 (odom (0,0)이 지도 중앙에 위치하도록 설정)
        # map_origin is based on map.yaml's origin
        self.map_origin = (origin[0], origin[1])

        if not os.path.exists(map_image_path):
            self.node.get_logger().error(f"지도 이미지 파일을 찾을 수 없습니다: {map_image_path}")
            QMessageBox.critical(self, "오류", f"지도 이미지 파일을 찾을 수 없습니다: {map_image_path}")
            return

        pixmap = QPixmap(map_image_path)
        if pixmap.isNull():
            self.node.get_logger().error(f"지도 이미지 로드 실패: {map_image_path}")
            QMessageBox.critical(self, "오류", f"지도 이미지 로드 실패: {map_image_path}")
            return

        # 맵 이미지 축소 비율 설정 (예: 50% 축소)
        scale_factor = 0.5  # 원하는 축소 비율로 조정
        scaled_pixmap = pixmap.scaled(
            int(pixmap.width() * scale_factor),
            int(pixmap.height() * scale_factor),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        # 맵 이미지 추가
        self.map_scene.clear()
        self.map_scene.addPixmap(scaled_pixmap)

        # QRect을 QRectF로 변환하여 전달
        self.map_scene.setSceneRect(QRectF(scaled_pixmap.rect()))

        # 로봇 아이콘 다시 추가 (이미 추가됨)

        self.map_loaded = True

        # 맵 이미지 크기 축소하여 GUI 창에 맞게 표시
        self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

        # QGraphicsView의 크기 고정 해제하여 동적 크기 조정 가능
        # self.map_view.setFixedSize(scaled_pixmap.width(), scaled_pixmap.height())  # 제거

        self.node.get_logger().info(f"맵 로드 완료: {map_image_path}")
        self.node.get_logger().info(f"해상도: {resolution} m/pixel, 원점: {origin}")
        self.node.get_logger().info(f"맵 스케일: {self.map_scale} 픽셀/meter, 맵 원점: {self.map_origin}")

    def handle_button_click(self):
        clicked_button = self.sender()
        if clicked_button.isChecked():
            for button in self.buttons:
                if button == clicked_button:
                    button.setStyleSheet("background-color: yellow")
                    self.destination = int(button.text())
                    print(f"Destination set to: {self.destination}")
                else:
                    button.setStyleSheet("")
                    button.setChecked(False)
        else:
            # 모든 버튼이 체크 해제되지 않도록 방지
            clicked_button.setChecked(True)

    def update_robot_position(self):
        for robot in self.robots:
            robot_name = robot['name']
            if self.node.current_pose[robot_name]:
                x = self.node.current_pose[robot_name].position.x
                y = self.node.current_pose[robot_name].position.y
                self.update_robot_position_on_map(x, y, robot_name)

    def update_robot_position_on_map(self, x, y, robot_name):
        if not hasattr(self, 'map_loaded') or not self.map_loaded:
            return
        # 실제 좌표를 픽셀 좌표로 변환 (map_origin을 기준으로 설정)
        pixel_x = (x - self.map_origin[0]) * self.map_scale * 0.5 + self.map_view.width() / 2
        pixel_y = (-y + self.map_origin[1]) * self.map_scale * 0.5 + self.map_view.height() / 2
        self.node.get_logger().debug(f"로봇 {robot_name} 픽셀 좌표: x={pixel_x}, y={pixel_y}")  # 디버깅용 로그 추가

        self.robot_items[robot_name].setPos(pixel_x, pixel_y)  # y축 반전

    def update_camera_image(self, robot_name, q_img):
        if robot_name in self.camera_labels:
            # QImage를 QPixmap으로 변환하여 QLabel에 표시
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaled(
                self.camera_labels[robot_name].width(),
                self.camera_labels[robot_name].height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.camera_labels[robot_name].setPixmap(scaled_pixmap)

class RobotControlGUI(QWidget):
    def __init__(self, robot_queue, node, image_signal):
        super().__init__()
        # 이 예제에서는 더 이상 주문 큐나 이미지 신호가 필요 없으므로 생략
        pass

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    import sys  # 이미 상단에 있음
    # Qt 플랫폼 플러그인 경로 설정 (필요에 따라 수정)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'

    # SHM Transport 비활성화 (필요시)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    rclpy.init(args=args)

    robot_queue = queue.Queue()
    image_signal = ImageSignal()

    # Define robots with names and colors
    robots = [
        {'name': 'tb1', 'color': 'red'},
        {'name': 'tb2', 'color': 'blue'},
    ]

    node = RobotControlNode(robots, image_signal)

    # ROS2 spinning을 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    map_window = MapWindow(node, robots)
    map_window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == "__main__":
    main()

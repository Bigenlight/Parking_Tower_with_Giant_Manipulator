#!/usr/bin/env python3
#
# Copyright ...
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

import sys
import threading
import queue
import os
import yaml  # YAML 파일을 읽기 위한 패키지 추가
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGraphicsView, QGraphicsScene, QMessageBox, QMainWindow
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer, QRectF
from PyQt5.QtGui import QPixmap, QPen, QBrush, QColor, QPainter, QImage
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry  # Odometry 메시지 임포트
from sensor_msgs.msg import Image  # Image 메시지 임포트
from cv_bridge import CvBridge  # cv_bridge 임포트
import numpy as np  # numpy 임포트

class ImageSignal(QObject):
    image_signal = pyqtSignal(QImage)  # QImage 신호 추가

class RobotControlNode(Node):
    def __init__(self, robot_queue, image_signal):
        super().__init__('robot_control_node')
        self.image_signal = image_signal
        self.robot_queue = robot_queue

        # NavigateToPose 액션 클라이언트 초기화
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # /odom 토픽 구독 초기화
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # /camera/image_raw 토픽 구독 초기화
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()  # CvBridge 인스턴스 생성

        # 로봇의 현재 위치 저장 변수 (Pose 타입)
        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        # Odometry 메시지에서 Pose 추출
        self.current_pose = msg.pose.pose
        self.get_logger().debug(f"현재 로봇 위치: x={self.current_pose.position.x}, y={self.current_pose.position.y}")

    def image_callback(self, msg: Image):
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
            self.image_signal.image_signal.emit(q_img)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

    def send_navigate_goal_to_position(self, position):
        x, y, z = position
        # 액션 서버가 준비될 때까지 대기 (최대 3초)
        if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("NavigateToPose 액션 서버가 준비되지 않았습니다.")
            return

        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        # 기본 방향 설정 (필요에 따라 수정)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # 목표 전송
        send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        send_goal_future.add_done_callback(lambda future: self.navigate_to_pose_action_goal(future))

    def navigate_to_pose_action_feedback(self, feedback_msg):
        # 피드백 처리 (선택 사항)
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Current pose: {feedback.current_pose.pose}")

    def navigate_to_pose_action_goal(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"목표 전송 실패: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().info("목표가 거부되었습니다.")
            return

        self.get_logger().info("목표가 수락되었습니다.")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigate_to_pose_result)

    def navigate_to_pose_result(self, future):
        try:
            result = future.result().result
            status = future.result().status
            if status == 4:
                self.get_logger().info("목표가 중단되었습니다.")
            elif status == 5:
                self.get_logger().warn("목표가 취소되었습니다.")
            else:
                self.get_logger().info(f"목표 달성: {result}")
        except Exception as e:
            self.get_logger().error(f"목표 결과 처리 실패: {e}")

    def close(self):
        pass  # 필요한 종료 작업이 있다면 추가

import cv2  # OpenCV 임포트

class MapWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("로봇 제어 GUI")
        self.setGeometry(100, 100, 1600, 800)  # 초기 창 크기 설정 (넓이를 늘림)

        # 메인 위젯 설정
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)  # 수평 레이아웃

        # 카메라 이미지 표시를 위한 QLabel 설정
        self.camera_label = QLabel()
        self.camera_label.setFixedSize(640, 480)  # 카메라 이미지 크기 설정
        self.camera_label.setStyleSheet("background-color: black;")  # 기본 배경색 설정
        self.camera_label.setAlignment(Qt.AlignCenter)  # 중앙 정렬

        # 지도 표시를 위한 QGraphicsView와 QGraphicsScene 설정
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView(self.map_scene)
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)  # 가로 스크롤바 제거
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)    # 세로 스크롤바 제거

        # 지도 로드 (map.yaml과 map.pgm 사용)
        self.load_map("/home/rokey/7_ws/src/robot_control_gui/maps/map.yaml")  # 실제 지도 yaml 경로로 변경

        # 로봇 아이콘 추가 (원 형태)
        self.robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(Qt.red), QBrush(Qt.red))
        self.robot_item.setZValue(1)  # 로봇 아이콘을 맵 위에 표시

        # 마우스 클릭 이벤트 필터 설정
        self.map_view.viewport().installEventFilter(self)

        # 메인 레이아웃에 카메라 이미지와 맵 추가
        self.main_layout.addWidget(self.camera_label)
        self.main_layout.addWidget(self.map_view)

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
        # 로봇의 초기 위치가 (20.00, -14.50)이므로, odom (0,0)이 이 위치에 매핑되도록 map_origin을 설정
        self.map_origin = (-2.00, 0.50)  # odom (0,0) 좌표가 지도 중앙에 위치하도록 설정

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

        # 로봇 아이콘 다시 추가 (축소된 맵에 맞게 위치 조정)
        self.robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(Qt.red), QBrush(Qt.red))
        self.robot_item.setZValue(1)  # 로봇 아이콘을 맵 위에 표시

        self.map_loaded = True

        # 맵 이미지 크기 축소하여 GUI 창에 맞게 표시
        self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

        # QGraphicsView의 크기를 축소된 맵 이미지 크기에 맞게 고정하여 스크롤바가 생기지 않도록 설정
        self.map_view.setFixedSize(scaled_pixmap.width(), scaled_pixmap.height())

        self.node.get_logger().info(f"맵 로드 완료: {map_image_path}")
        self.node.get_logger().info(f"해상도: {resolution} m/pixel, 원점: {origin}")
        self.node.get_logger().info(f"맵 스케일: {self.map_scale} 픽셀/meter, 맵 원점: {self.map_origin}")

    def eventFilter(self, source, event):
        if source == self.map_view.viewport() and event.type() == event.MouseButtonPress:
            if event.button() == Qt.LeftButton:
                pos = event.pos()
                scene_pos = self.map_view.mapToScene(pos)
                self.handle_map_click(scene_pos)
                return True
        return super().eventFilter(source, event)

    def handle_map_click(self, scene_pos):
        # 클릭한 픽셀 좌표를 실제 좌표로 변환
        pixel_x = scene_pos.x()
        pixel_y = scene_pos.y()
        # 지도의 중앙을 기준으로 변환 (맵 축소 비율 고려)
        # scale_factor = 0.5이므로, map_scale * scale_factor = 20.0 * 0.5 = 10.0
        real_x = (pixel_x - self.map_view.width() / 2) / (self.map_scale * 0.5) + self.map_origin[0]
        real_y = (-pixel_y + self.map_view.height() / 2) / (self.map_scale * 0.5) + self.map_origin[1]  # y축 반전

        self.node.get_logger().info(f"Map clicked at: real_x={real_x:.2f}, real_y={real_y:.2f}")
        QMessageBox.information(self, "목표 설정", f"목표 위치가 설정되었습니다: ({real_x:.2f}, {real_y:.2f})")

        # 목표 위치로 네비게이션 목표 전송
        goal_position = (real_x, real_y, 0.0)  # z는 평면상의 위치로 설정
        self.node.send_navigate_goal_to_position(goal_position)

    def update_robot_position(self):
        if self.node.current_pose:
            x = self.node.current_pose.position.x
            y = self.node.current_pose.position.y
            self.update_robot_position_on_map(x, y)

    def update_robot_position_on_map(self, x, y):
        if not hasattr(self, 'map_loaded') or not self.map_loaded:
            return
        # 실제 좌표를 픽셀 좌표로 변환 (odom (0,0)이 지도 중앙에 위치하도록 설정)
        # scale_factor = 0.5
        pixel_x = (x - self.map_origin[0]) * self.map_scale * 0.5 + self.map_view.width() / 2
        pixel_y = (-y + self.map_origin[1]) * self.map_scale * 0.5 + self.map_view.height() / 2
        self.node.get_logger().debug(f"로봇 픽셀 좌표: x={pixel_x}, y={pixel_y}")  # 디버깅용 로그 추가

        self.robot_item.setPos(pixel_x, pixel_y)  # y축 반전

    def update_camera_image(self, q_img):
        # QImage를 QPixmap으로 변환하여 QLabel에 표시
        pixmap = QPixmap.fromImage(q_img)
        scaled_pixmap = pixmap.scaled(
            self.camera_label.width(),
            self.camera_label.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.camera_label.setPixmap(scaled_pixmap)

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
    node = RobotControlNode(robot_queue, image_signal)

    # ROS2 spinning을 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    map_window = MapWindow(node)
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

#!/usr/bin/env python3
import sys
import threading
import queue
import os
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGraphicsView, QGraphicsScene, QMessageBox, QMainWindow
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QPen, QBrush, QColor, QPainter
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ImageSignal(QObject):
    image_signal = pyqtSignal(object)

class RobotControlNode(Node):
    def __init__(self, robot_queue, image_signal):
        super().__init__('robot_control_node')
        self.image_signal = image_signal
        self.robot_queue = robot_queue

        # NavigateToPose 액션 클라이언트 초기화
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF 버퍼와 리스너 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 로봇의 현재 위치 저장 변수
        self.current_pose = None

        # 주기적으로 TF를 조회하여 로봇의 위치 업데이트
        self.create_timer(0.1, self.update_robot_pose)

    def update_robot_pose(self):
        try:
            # 'base_link'는 로봇의 기본 프레임, 'map'은 전역 프레임
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_pose = trans.transform
            self.get_logger().debug(f"현재 로봇 위치: x={self.current_pose.translation.x}, y={self.current_pose.translation.y}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("로봇의 현재 위치를 조회할 수 없습니다.")

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

class MapWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("로봇 제어 GUI")
        self.setGeometry(100, 100, 1200, 800)  # 창 크기 조정

        # 메인 위젯 설정
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)

        # 지도 표시를 위한 QGraphicsView와 QGraphicsScene 설정
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView(self.map_scene)
        self.map_view.setFixedSize(800, 800)  # 원하는 크기로 설정
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_view.setRenderHint(QPainter.Antialiasing)

        # 로봇 아이콘 추가 (원 형태)
        self.robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(Qt.red), QBrush(Qt.red))
        self.robot_item.setZValue(1)  # 로봇 아이콘을 맵 위에 표시

        # 지도 로드 (예: slam_toolbox로 생성된 지도 사용)
        self.load_map("/home/rokey/7_ws/src/robot_control_gui/maps/map.png")  # 실제 지도 이미지 경로로 변경

        # 마우스 클릭 이벤트 필터 설정
        self.map_view.viewport().installEventFilter(self)

        self.main_layout.addWidget(self.map_view)

        # 로봇 위치 업데이트를 위한 타이머 설정 (100ms마다)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_position)
        self.timer.start(100)  # 100ms

    def load_map(self, map_image_path):
        if not os.path.exists(map_image_path):
            self.node.get_logger().error(f"지도 이미지 파일을 찾을 수 없습니다: {map_image_path}")
            QMessageBox.critical(self, "오류", f"지도 이미지 파일을 찾을 수 없습니다: {map_image_path}")
            return

        pixmap = QPixmap(map_image_path)
        self.map_scene.clear()
        self.map_scene.addPixmap(pixmap)
        self.map_scene.setSceneRect(pixmap.rect())

        # 로봇 아이콘 다시 추가
        self.robot_item = self.map_scene.addEllipse(-5, -5, 10, 10, QPen(Qt.red), QBrush(Qt.red))
        self.robot_item.setZValue(1)  # 로봇 아이콘을 맵 위에 표시

        self.map_loaded = True

        # 지도 스케일 및 원점 설정 (실제 환경에 맞게 조정)
        self.map_scale = 100.0  # 예: 1미터당 100픽셀
        self.map_origin = (-5.0, -5.0)  # 지도 원점의 실제 좌표 (x, y)

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
        real_x = pixel_x / self.map_scale + self.map_origin[0]
        real_y = -pixel_y / self.map_scale + self.map_origin[1]  # y축 반전

        self.node.get_logger().info(f"Map clicked at: real_x={real_x:.2f}, real_y={real_y:.2f}")
        QMessageBox.information(self, "목표 설정", f"목표 위치가 설정되었습니다: ({real_x:.2f}, {real_y:.2f})")

        # 목표 위치로 네비게이션 목표 전송
        goal_position = (real_x, real_y, 0.0)  # z는 평면상의 위치로 설정
        self.node.send_navigate_goal_to_position(goal_position)

    def update_robot_position(self):
        if self.node.current_pose:
            x = self.node.current_pose.translation.x
            y = self.node.current_pose.translation.y
            self.update_robot_position_on_map(x, y)

    def update_robot_position_on_map(self, x, y):
        if not hasattr(self, 'map_loaded') or not self.map_loaded:
            return
        # 실제 좌표를 픽셀 좌표로 변환
        pixel_x = (x - self.map_origin[0]) * self.map_scale
        pixel_y = (y - self.map_origin[1]) * self.map_scale
        self.robot_item.setPos(pixel_x, -pixel_y)  # y축 반전 (픽셀 좌표계와 ROS 좌표계 차이)

class RobotControlGUI(QWidget):
    def __init__(self, robot_queue, node, image_signal):
        super().__init__()
        # 이 예제에서는 더 이상 주문 큐나 이미지 신호가 필요 없으므로 생략
        pass

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    # Qt 플랫폼 플러그인 경로 설정 (필요에 따라 수정)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'
    
    
    rclpy.init(args=args)

    robot_queue = queue.Queue()
    image_signal = ImageSignal()
    node = RobotControlplatformsNode(robot_queue, image_signal)

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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class MultiTurtlebotNavigator(Node):
    def __init__(self):
        super().__init__('multi_turtlebot_navigator')

        # === 콜백 그룹 설정 (tb1/tb2 액션 콜백 분리) ===
        self.cb_group_tb1 = MutuallyExclusiveCallbackGroup()
        self.cb_group_tb2 = MutuallyExclusiveCallbackGroup()

        # === tb1, tb2 각각에 대한 액션 클라이언트 생성 ===
        self.tb1_action_client = ActionClient(
            self,
            NavigateToPose,
            '/tb1/navigate_to_pose',
            callback_group=self.cb_group_tb1
        )
        self.tb2_action_client = ActionClient(
            self,
            NavigateToPose,
            '/tb2/navigate_to_pose',
            callback_group=self.cb_group_tb2
        )

        # === /turtlebot_state 토픽 퍼블리셔 생성 ===
        self.turtlebot_state_pub = self.create_publisher(String, '/turtlebot_state', 10)

        # === 로그 출력 간격(초) 조절을 위해 최근 로그 시각 저장 ===
        self.last_tb1_feedback_time = 0.0
        self.last_tb2_feedback_time = 0.0
        self.feedback_interval = 5.0  # 5초마다만 피드백 로그 출력

        # === tb1, tb2 도착 여부 상태 변수 ===
        self.tb1_done = False
        self.tb2_done = False

        # ----------------------------------------------------------------------
        #       tb1: "중간 웨이포인트" (goal1), "최종 목표" (goal2) 따로 설정
        # ----------------------------------------------------------------------

        # 1) tb1 중간 웨이포인트 설정
        #    아래 값은 질문에서 주어진 /tb1/amcl_pose 예시 값입니다.
        self.tb1_waypoint = NavigateToPose.Goal()
        self.tb1_waypoint.pose.header.frame_id = 'map'
        self.tb1_waypoint.pose.pose.position.x = 2.4
        self.tb1_waypoint.pose.pose.position.y = 1.28306016716040083
        self.tb1_waypoint.pose.pose.orientation.z = 0.0
        self.tb1_waypoint.pose.pose.orientation.w = 1.0

        # 2) tb1 최종 목표지점 설정
        self.tb1_goal = NavigateToPose.Goal()
        self.tb1_goal.pose.header.frame_id = 'map'
        self.tb1_goal.pose.pose.position.x = 34.82508850097656
        self.tb1_goal.pose.pose.position.y = 0.28306016716040083
        self.tb1_goal.pose.pose.orientation.z = 0.0
        self.tb1_goal.pose.pose.orientation.w = 1.0

        # === tb1가 중간 웨이포인트를 찍었는지 여부 체크 ===
        self.tb1_waypoint_reached = False

        # === tb2 목표지점 설정 ===
        self.tb2_goal = NavigateToPose.Goal()
        self.tb2_goal.pose.header.frame_id = 'map'
        self.tb2_goal.pose.pose.position.x = 34.83428955078125
        self.tb2_goal.pose.pose.position.y = -1.000349067687988
        self.tb2_goal.pose.pose.orientation.z = 0.0436194  # Updated for +5 degrees rotation
        self.tb2_goal.pose.pose.orientation.w = 0.9990482  # Updated for +5 degrees rotation

        # === 액션 서버 연결 대기 ===
        self.get_logger().info('Waiting for tb1 navigate_to_pose action server...')
        self.tb1_action_client.wait_for_server()
        self.get_logger().info('tb1 navigate_to_pose action server connected.')

        self.get_logger().info('Waiting for tb2 navigate_to_pose action server...')
        self.tb2_action_client.wait_for_server()
        self.get_logger().info('tb2 navigate_to_pose action server connected.')

        # ----------------------------------------------------------------------
        #   tb1 먼저 "웨이포인트 목표"를 전송 -> 도착 콜백에서 최종 목표 전송
        # ----------------------------------------------------------------------
        self.send_tb1_waypoint()

        # === tb2 Goal 전송(기존 로직 그대로) ===
        self.send_tb2_goal()

    # ----------------------------------------------------------------------
    #                           액션 전송 함수
    # ----------------------------------------------------------------------
    def send_tb1_waypoint(self):
        """tb1에게 중간 웨이포인트를 먼저 전송"""
        self.get_logger().info('Sending tb1 waypoint goal...')
        self._tb1_send_goal_future = self.tb1_action_client.send_goal_async(
            self.tb1_waypoint,
            feedback_callback=self.tb1_feedback_callback
        )
        self._tb1_send_goal_future.add_done_callback(self.tb1_goal_response_callback)

    def send_tb1_goal(self):
        """tb1에게 최종 목표를 전송"""
        self.get_logger().info('Sending tb1 final goal...')
        self._tb1_send_goal_future = self.tb1_action_client.send_goal_async(
            self.tb1_goal,
            feedback_callback=self.tb1_feedback_callback
        )
        self._tb1_send_goal_future.add_done_callback(self.tb1_goal_response_callback)

    def send_tb2_goal(self):
        self.get_logger().info('Sending tb2 goal...')
        self._tb2_send_goal_future = self.tb2_action_client.send_goal_async(
            self.tb2_goal,
            feedback_callback=self.tb2_feedback_callback
        )
        self._tb2_send_goal_future.add_done_callback(self.tb2_goal_response_callback)

    # ----------------------------------------------------------------------
    #                         피드백 콜백 함수
    # ----------------------------------------------------------------------
    def tb1_feedback_callback(self, feedback_msg):
        """tb1 경로 진행 상황 피드백 (5초에 한 번씩만 로그)"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_tb1_feedback_time > self.feedback_interval:
            distance_remaining = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f"[TB1] distance remaining = {distance_remaining:.3f}")
            self.last_tb1_feedback_time = now_sec

    def tb2_feedback_callback(self, feedback_msg):
        """tb2 경로 진행 상황 피드백 (5초에 한 번씩만 로그)"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_tb2_feedback_time > self.feedback_interval:
            distance_remaining = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f"[TB2] distance remaining = {distance_remaining:.3f}")
            self.last_tb2_feedback_time = now_sec

    # ----------------------------------------------------------------------
    #                     Goal 수락 여부 콜백 함수
    # ----------------------------------------------------------------------
    def tb1_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('tb1 goal rejected.')
            return

        self.get_logger().info('tb1 goal accepted.')
        self._tb1_get_result_future = goal_handle.get_result_async()
        self._tb1_get_result_future.add_done_callback(self.tb1_get_result_callback)

    def tb2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('tb2 goal rejected.')
            return

        self.get_logger().info('tb2 goal accepted.')
        self._tb2_get_result_future = goal_handle.get_result_async()
        self._tb2_get_result_future.add_done_callback(self.tb2_get_result_callback)

    # ----------------------------------------------------------------------
    #                     최종 결과 콜백 함수 (arrived)
    # ----------------------------------------------------------------------
    def tb1_get_result_callback(self, future):
        _ = future.result().result

        # 아직 웨이포인트를 안 찍은 상태였다면(웨이포인트 도착 시)
        if not self.tb1_waypoint_reached:
            self.get_logger().info('tb1 arrived at waypoint!')

            # tb1이 웨이포인트에 도착했다는 메시지 퍼블리시
            msg = String()
            msg.data = 'tb1 arrived at waypoint'
            self.turtlebot_state_pub.publish(msg)

            # 웨이포인트 도착 상태 설정
            self.tb1_waypoint_reached = True

            # --- 이제 tb1 최종 목표를 전송 ---
            self.send_tb1_goal()

        else:
            # 이미 웨이포인트를 찍었고, 최종 목표에 도착한 상태
            self.get_logger().info('tb1 arrived at final goal!')

            msg = String()
            msg.data = 'tb1 arrived'
            self.turtlebot_state_pub.publish(msg)

            self.tb1_done = True

    def tb2_get_result_callback(self, future):
        _ = future.result().result
        self.get_logger().info('tb2 arrived at goal!')

        msg = String()
        msg.data = 'tb2 arrived'
        self.turtlebot_state_pub.publish(msg)

        self.tb2_done = True


def main(args=None):
    rclpy.init(args=args)

    # 멀티 스레드 실행자 설정
    executor = MultiThreadedExecutor(num_threads=4)
    node = MultiTurtlebotNavigator()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

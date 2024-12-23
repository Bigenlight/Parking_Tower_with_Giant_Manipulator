#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class TurtlebotStateMonitor(Node):
    def __init__(self):
        super().__init__('turtlebot_state_monitor')
        self.tb1_arrived = False
        self.tb2_arrived = False

        # /turtlebot_state 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/turtlebot_state',
            self.state_callback,
            10
        )

        self.get_logger().info('TurtlebotStateMonitor node has started.')

    def state_callback(self, msg):
        if msg.data == 'tb1 arrived':
            self.tb1_arrived = True
            self.get_logger().info('Detected: tb1 arrived')
        elif msg.data == 'tb2 arrived':
            self.tb2_arrived = True
            self.get_logger().info('Detected: tb2 arrived')

        if self.tb1_arrived and self.tb2_arrived:
            self.get_logger().info('Both tb1 and tb2 have arrived. Initiating shutdown of Nav2 and Rviz...')
            self.shutdown_nav2_and_rviz()

    def shutdown_nav2_and_rviz(self):
        try:
            # # 각 로봇의 네임스페이스를 기반으로 Nav2 관련 노드 종료
            # robots = ['tb1', 'tb2']
            # for robot in robots:
            #     namespace = f'/{robot}'

            #     # Nav2 관련 노드 이름 (네임스페이스 포함)
            #     nav2_nodes = [
            #         f'{namespace}/bt_navigator',
            #         f'{namespace}/controller_server',
            #         f'{namespace}/planner_server',
            #         f'{namespace}/amcl',
            #         f'{namespace}/lifecycle_manager_map_server',
            #         f'{namespace}/map_server'
            #     ]

                # for node_name in nav2_nodes:
                #     self.get_logger().info(f'Killing Nav2 node: {node_name}')
                #     # subprocess.run(['ros2', 'node', 'kill', node_name], check=True)
                #     # 대안: pkill을 사용하여 노드 이름으로 프로세스 종료
                #     subprocess.run(['pkill', '-f', node_name], check=True)

                # 해당 로봇의 Rviz 노드 종료
                # rviz_node_name = f'rviz_{robot}'
            rviz_node_name = f'rviz_tb1'
            self.get_logger().info(f'Killing Rviz node: {rviz_node_name}')
            subprocess.run(['pkill', '-f', rviz_node_name], check=True)

            self.get_logger().info('Nav2 and Rviz have been successfully terminated.')

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        finally:
            # 모니터 노드 종료
            self.get_logger().info('Shutting down TurtlebotStateMonitor node.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

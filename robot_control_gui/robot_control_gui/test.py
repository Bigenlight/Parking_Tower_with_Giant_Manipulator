#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Publishers
        self.turtlebot_pub = self.create_publisher(String, '/turtlebot_state', 10)
        self.arm_pub = self.create_publisher(String, '/arm_state', 10)

        # Subscriber
        self.arm_commands_sub = self.create_subscription(
            String,
            '/arm_commands',
            self.arm_commands_callback,
            10
        )

        # Start initial sequence
        self.publish_tb1_arrived()

        # Variables for handling moving sequences
        self.moving_timer = None
        self.move_count = 0
        self.moving_type = None  # 'on_board' or 'empty'

    def publish_tb1_arrived(self):
        """Publish 'tb1 arrived' and schedule next step."""
        msg = String()
        msg.data = 'tb1 arrived'
        self.turtlebot_pub.publish(msg)
        self.get_logger().info('turtlebot1 선착장 도착')

        # Schedule tb2 arrived after 1 second
        self.create_one_shot_timer(1.0, self.publish_tb2_arrived)

    def publish_tb2_arrived(self):
        """Publish 'tb2 arrived' and schedule next step."""
        msg = String()
        msg.data = 'tb2 arrived'
        self.turtlebot_pub.publish(msg)
        self.get_logger().info('turtlebot2 선착장 도착')

        # Schedule port arrived after 1 second
        self.create_one_shot_timer(1.0, self.publish_port_arrived)

    def publish_port_arrived(self):
        """Publish 'port arrived'."""
        msg = String()
        msg.data = 'port arrived'
        self.arm_pub.publish(msg)
        self.get_logger().info('robot arm 선착장 도착')

    def publish_tower_arrived(self):
        """Publish 'tower arrived'."""
        msg = String()
        msg.data = 'tower arrived'
        self.arm_pub.publish(msg)
        self.get_logger().info('robot arm 주차장 도착')

    def arm_commands_callback(self, msg):
        """Handle 'on board {n}' and 'empty' commands."""
        command = msg.data
        self.get_logger().info(f'Received arm_command: "{command}"')

        if command.startswith('on board'):
            parts = command.split()
            if len(parts) == 3:
                try:
                    n = int(parts[2])
                    if 1 <= n <= 9:
                        self.start_moving_sequence(command_type='on_board')
                    else:
                        self.get_logger().warn(f'Invalid on board number: {n}. Must be between 1 and 9.')
                except ValueError:
                    self.get_logger().warn(f'Invalid on board number: {parts[2]} is not an integer.')
            else:
                self.get_logger().warn(f'Invalid on board command format: "{command}". Expected "on board <n>".')

        elif command == 'empty':
            self.start_moving_sequence(command_type='empty')
        else:
            self.get_logger().warn(f'Unknown arm command: "{command}".')

    def start_moving_sequence(self, command_type):
        """Start the 3-second moving sequence."""
        if self.moving_timer is not None:
            self.get_logger().warn('Already in a moving sequence. Ignoring new command.')
            return

        self.move_count = 0
        self.moving_type = command_type
        self.moving_timer = self.create_one_shot_timer(1.0, self.moving_timer_callback)
        self.get_logger().info('Starting moving sequence.')

    def moving_timer_callback(self):
        """Callback for moving sequence."""
        self.move_count += 1
        self.get_logger().info(f'로봇팔 이동중({self.move_count} sec)')

        if self.move_count < 3:
            # Reschedule for next second
            self.moving_timer = self.create_one_shot_timer(1.0, self.moving_timer_callback)
        else:
            # After 3 seconds, publish 'tower arrived' or 'port arrived'
            if self.moving_type == 'on_board':
                msg = String()
                msg.data = 'tower arrived'
                self.arm_pub.publish(msg)
                self.get_logger().info('tower arrived')
            elif self.moving_type == 'empty':
                msg = String()
                msg.data = 'port arrived'
                self.arm_pub.publish(msg)
                self.get_logger().info('port arrived')

            # Reset moving_timer and moving_type
            self.moving_timer = None
            self.moving_type = None

    def create_one_shot_timer(self, timer_period, callback):
        """Create a one-shot timer that calls the callback once after timer_period seconds."""
        # Using lambda to capture the timer variable in the callback
        timer = self.create_timer(timer_period, lambda: self.on_timer_callback(callback, timer))
        return timer

    def on_timer_callback(self, callback, timer):
        """Callback wrapper that calls the actual callback and destroys the timer."""
        callback()
        self.destroy_timer(timer)


def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        test_publisher.get_logger().info('Test publisher stopped cleanly')
    finally:
        test_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

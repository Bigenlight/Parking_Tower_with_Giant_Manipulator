#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Int32
from arduinobot_msgs.action import ArduinobotTask

from linkattacher_msgs.srv import AttachLink, DetachLink


class ManipulatorManager(Node):
    def __init__(self):
        super().__init__("manipulator_manager")
        self.get_logger().info("매니저 시작...")

        # -- Internal State --
        # We'll only do one "port move" if the first turtlebot_state is "tb2 arrived".
        self.port_action_done_once = False  # Once we've done it, we never do it again.

        # We'll track how many times we've done a *non-zero* destination
        self.non_zero_destination_count = 0

        # If the last destination was 0, we can re-publish "port arrived" once
        self.last_destination = None
        self.has_published_port_again = False

        # -- Publishers --
        self.arm_state_pub = self.create_publisher(String, "/arm_state", 10)

        # -- Subscriptions --
        self.tb_state_sub = self.create_subscription(
            String, "/turtlebot_state", self.tb_state_callback, 10
        )
        self.destination_sub = self.create_subscription(
            Int32, "/destination", self.destination_callback, 10
        )

        # -- Action Client --
        self._action_client = ActionClient(self, ArduinobotTask, "/task_server")

        # -- Service Clients (async) --
        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")

        # Poll service availability
        self._attach_available = False
        self._detach_available = False

        def check_services():
            if not self._attach_available:
                self._attach_available = self.attach_client.service_is_ready()
            if not self._detach_available:
                self._detach_available = self.detach_client.service_is_ready()
            if self._attach_available and self._detach_available:
                self.get_logger().info("Attach/Detach services are ready.")
                self.destroy_timer(self._service_check_timer)

        self._service_check_timer = self.create_timer(1.0, check_services)

        self.get_logger().info("ManipulatorManager node has been started")


    # ----------------------------------------------------------------------------
    # Callback: /turtlebot_state
    # ----------------------------------------------------------------------------
    def tb_state_callback(self, msg: String):
        """
        If the first /turtlebot_state we see is "tb2 arrived",
        do a one-time port move (task=0) with no attach/detach.
        """
        robot_str = msg.data.strip()

        # If we haven't done the port move yet, and the message is exactly "tb2 arrived"
        if (not self.port_action_done_once) and (robot_str == "tb2 arrived"):
            self.port_action_done_once = True
            self.get_logger().info("First time seeing 'tb2 arrived' => doing one-time port move (task=0).")
            self.send_arm_goal(
                task_number=0,
                done_callback=self._on_first_port_move_complete
            )

        # If our last destination was 0, we can re-publish "port arrived" once
        if self.last_destination == 0 and not self.has_published_port_again:
            self.publish_port_arrived()
            self.has_published_port_again = True


    def _on_first_port_move_complete(self):
        """
        Called after finishing the special port move (task=0) triggered by 'tb2 arrived'.
        """
        self.get_logger().info("[_on_first_port_move_complete] Arrived at port => publishing 'port arrived'.")
        self.publish_port_arrived()


    # ----------------------------------------------------------------------------
    # Callback: /destination
    # ----------------------------------------------------------------------------
    def destination_callback(self, msg: Int32):
        destination_num = msg.data
        self.get_logger().info(f"[destination_callback] Received destination: {destination_num}")

        # If the user sets destination=0 => do the standard zero logic
        if destination_num == 0:
            self._handle_zero_destination()
            return

        # Otherwise, handle non-zero
        self._handle_nonzero_destination(destination_num)


    def _handle_zero_destination(self):
        """
        Move to 13 -> Move to 0 -> Publish "port arrived"
        No attach/detach.
        """
        self.has_published_port_again = False  # Reset in case we do multiple zeros in a row

        def after_13():
            # Then go to 0
            self.send_arm_goal(
                task_number=0,
                done_callback=self._on_zero_destination_port_done
            )

        self.get_logger().info("[_handle_zero_destination] => Move to 13, then 0, no attach/detach.")
        self.send_arm_goal(task_number=13, done_callback=after_13)


    def _on_zero_destination_port_done(self):
        """
        Called after finishing move to 0 for a /destination=0 scenario.
        """
        self.get_logger().info("[_on_zero_destination_port_done] Reached 0 => publishing 'port arrived'.")
        self.last_destination = 0
        self.publish_port_arrived()


    def _handle_nonzero_destination(self, destination_num: int):
        """
        If /destination > 0, we do the following based on how many non-zero destinations
        we have already processed:
          1) For the 1st non-zero => attach tb2, move 13, move final, detach tb2.
          2) For the 2nd non-zero => attach tb1, move 13, move final, detach tb1.
          3) For the 3rd+ => no attach/detach, just move 13, move final.
        Finally, publish "tower arrived".
        """
        self.has_published_port_again = False

        if self.non_zero_destination_count == 0:
            # First non-zero => attach tb2
            robot_name = "tb2"
        elif self.non_zero_destination_count == 1:
            # Second non-zero => attach tb1
            robot_name = "tb1"
        else:
            # Third or beyond => no attach
            robot_name = None

        self.non_zero_destination_count += 1
        self.get_logger().info(
            f"[handle_nonzero_destination] non_zero_destination_count={self.non_zero_destination_count}. "
            f"robot_name={robot_name}"
        )

        if robot_name is not None:
            time.sleep(5)  # Slow down before attach
            self._attach_then_move(destination_num, robot_name)
        else:
            self._move_13_then_final(destination_num, None)


    def _attach_then_move(self, destination_num: int, robot_name: str):
        self.async_attach_object(
            robot_name,
            on_done=lambda success: self._after_attach(success, robot_name, destination_num)
        )

    def _after_attach(self, success: bool, robot_name: str, destination_num: int):
        if success:
            self.get_logger().info(f"[attach] Successfully attached {robot_name}")
        else:
            self.get_logger().error(f"[attach] Failed to attach {robot_name}, continuing anyway...")

        self._move_13_then_final(destination_num, robot_name)


    def _move_13_then_final(self, destination_num: int, robot_name: str):
        """
        Move to 13, then move to 'destination_num'.
        If 'robot_name' is not None, we detach afterwards.
        """
        def after_13():
            # Then go to final destination
            self.send_arm_goal(
                task_number=destination_num,
                done_callback=lambda: self._on_final_destination(destination_num, robot_name)
            )

        self.get_logger().info(f"[move_13_then_final] ->13-> then ->{destination_num} (robot={robot_name})")
        self.send_arm_goal(task_number=13, done_callback=after_13)


    def _on_final_destination(self, destination_num: int, robot_name: str):
        """
        Called after finishing the final non-zero destination move.
        If robot_name != None, detach. Then publish "tower arrived".
        """
        self.get_logger().info(f"[_on_final_destination] Reached {destination_num}.")
        self.last_destination = destination_num

        if robot_name:
            self.async_detach_object(
                robot_name,
                on_done=lambda _: self._publish_tower_arrived()
            )
        else:
            self._publish_tower_arrived()


    def _publish_tower_arrived(self):
        msg = String()
        msg.data = "tower arrived"
        self.arm_state_pub.publish(msg)
        self.get_logger().info("[_publish_tower_arrived] Published 'tower arrived'.")


    # --------------------------------------------------------------------------
    # Async Attach/Detach
    # --------------------------------------------------------------------------
    def async_attach_object(self, robot_name: str, on_done):
        """
        Attach 'robot_name' asynchronously.
        on_done(success: bool)
        """
        if not self._attach_available:
            self.get_logger().warn("[async_attach_object] Attach service not ready yet; cannot attach.")
            on_done(False)
            return

        req = AttachLink.Request()
        req.model1_name = "arduinobot"
        req.link1_name = "link_j6e"
        req.model2_name = robot_name
        req.link2_name = f"{robot_name}::base_link"

        self.get_logger().info(f"[async_attach_object] Sending AttachLink => {req}")
        future = self.attach_client.call_async(req)

        def _resp_cb(fut):
            resp = fut.result()
            if resp is None:
                self.get_logger().error("[async_attach_object] No response from attach service.")
                on_done(False)
                return
            if resp.success:
                self.get_logger().info(f"[async_attach_object] Attach success: {resp.message}")
                on_done(True)
            else:
                self.get_logger().error(f"[async_attach_object] Attach failed: {resp.message}")
                on_done(False)

        future.add_done_callback(_resp_cb)


    def async_detach_object(self, robot_name: str, on_done):
        """
        Detach 'robot_name' asynchronously.
        on_done(success: bool)
        """
        if not self._detach_available:
            self.get_logger().warn("[async_detach_object] Detach service not ready yet; cannot detach.")
            on_done(False)
            return

        req = DetachLink.Request()
        req.model1_name = "arduinobot"
        req.link1_name = "link_j6e"
        req.model2_name = robot_name
        req.link2_name = f"{robot_name}::base_link"

        self.get_logger().info(f"[async_detach_object] Sending DetachLink => {req}")
        future = self.detach_client.call_async(req)

        def _resp_cb(fut):
            resp = fut.result()
            if resp is None:
                self.get_logger().error("[async_detach_object] No response from detach service.")
                on_done(False)
                return
            if resp.success:
                self.get_logger().info(f"[async_detach_object] Detach success: {resp.message}")
                on_done(True)
            else:
                self.get_logger().error(f"[async_detach_object] Detach failed: {resp.message}")
                on_done(False)

        future.add_done_callback(_resp_cb)


    # --------------------------------------------------------------------------
    # Utility: Publish "port arrived"
    # --------------------------------------------------------------------------
    def publish_port_arrived(self):
        msg = String()
        msg.data = "port arrived"
        self.arm_state_pub.publish(msg)
        self.get_logger().info("[publish_port_arrived] Published 'port arrived'.")


    # --------------------------------------------------------------------------
    # Action goal helper
    # --------------------------------------------------------------------------
    def send_arm_goal(self, task_number: int, done_callback=None):
        if not self._action_client.server_is_ready():
            self.get_logger().info("[send_arm_goal] Action server not ready. Waiting...")
            self._action_client.wait_for_server()

        goal_msg = ArduinobotTask.Goal()
        goal_msg.task_number = task_number

        self.get_logger().info(f"[send_arm_goal] Sending ArduinobotTask goal: {task_number}")
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"[send_arm_goal] Goal {task_number} was rejected.")
                return

            self.get_logger().info(f"[send_arm_goal] Goal {task_number} accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def result_callback(future):
            res = future.result().result
            if res.success:
                self.get_logger().info(f"[send_arm_goal] Goal {task_number} succeeded!")
                if done_callback:
                    done_callback()
            else:
                self.get_logger().error(f"[send_arm_goal] Goal {task_number} failed.")

        send_goal_future.add_done_callback(goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorManager()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

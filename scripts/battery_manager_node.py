#!/usr/bin/env python3
"""
battery_manager_node.py
------------------------
State-machine node that adds a virtual battery + charging-station workflow
on top of the existing AMR exploration stack.

Integration point: subscribes to `exploration_complete` (std_msgs/Empty),
published by the frontier_exploration_ros2 node when
completion_event_enabled: true is set in frontier_explorer_params.yaml.
This topic is transient_local + reliable + depth 1, so a late-joining
subscriber (this node, if started after explore.launch.py) still receives
the latest completion event.

States:
    IDLE                    -> waiting for exploration_complete
    EXPLORING                -> exploration in progress (battery draining)
    EXPLORATION_COMPLETE      -> event received, about to ask for charger pose
    WAITING_FOR_CHARGER_INPUT -> blocking on terminal input() for x, y
    NAVIGATING_TO_CHARGER     -> NavigateToPose action goal in flight
    CHARGING                  -> battery climbing back to 100%
    DONE                      -> charging finished

Battery model:
    - Starts at 100.0
    - Drains while linear/angular velocity (from /odometry/filtered) is
      above a small noise threshold, at battery_drain_rate_per_sec.
    - Charges at battery_charge_rate_per_sec once NAVIGATING_TO_CHARGER
      succeeds and the CHARGING state is entered.
    - This is a SIMULATED battery for demonstration purposes — it is not
      tied to any real power model, just elapsed time x velocity.

Run after:
    Terminal 1: ros2 launch amr_description spawn.launch.py
    Terminal 2: ros2 launch amr_description slam.launch.py
    Terminal 3: ros2 launch amr_description nav2.launch.py use_slam:=true
    Terminal 4: ros2 launch amr_description explore.launch.py
    Terminal 5: ros2 run amr_description battery_manager_node.py
                (or: ros2 launch amr_description battery.launch.py)
"""

import math
import threading
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty, String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


class RobotState(Enum):
    IDLE = auto()
    EXPLORING = auto()
    EXPLORATION_COMPLETE = auto()
    WAITING_FOR_CHARGER_INPUT = auto()
    NAVIGATING_TO_CHARGER = auto()
    CHARGING = auto()
    DONE = auto()


class BatteryManagerNode(Node):

    def __init__(self):
        super().__init__('battery_manager_node')

        # ------------------------------------------------------------ #
        # Parameters — all overridable via config/battery_config.yaml
        # ------------------------------------------------------------ #
        self.declare_parameter('completion_event_topic', 'exploration_complete')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('battery_drain_rate_per_sec', 0.05)
        self.declare_parameter('battery_charge_rate_per_sec', 1.0)
        self.declare_parameter('velocity_noise_threshold', 0.02)
        self.declare_parameter('full_battery_value', 100.0)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('use_sim_time_for_battery', True)

        self.completion_topic = self.get_parameter('completion_event_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.drain_rate = self.get_parameter('battery_drain_rate_per_sec').value
        self.charge_rate = self.get_parameter('battery_charge_rate_per_sec').value
        self.vel_threshold = self.get_parameter('velocity_noise_threshold').value
        self.full_battery = self.get_parameter('full_battery_value').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value

        # ------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------ #
        self.state = RobotState.EXPLORING
        self.battery_level = self.full_battery
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0
        self._goal_handle = None
        self._input_thread = None

        # ------------------------------------------------------------ #
        # QoS matching frontier_exploration_ros2's completion event
        # (transient_local, reliable, depth 1) — REQUIRED to receive a
        # message that may have been published before this node started.
        # ------------------------------------------------------------ #
        completion_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.completion_sub = self.create_subscription(
            Empty,
            self.completion_topic,
            self.on_exploration_complete,
            completion_qos,
        )

        # Odometry — default QoS is fine, /odometry/filtered uses reliable/volatile
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.on_odom,
            10,
        )

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        # Nav2 action client — same action the frontier explorer already uses
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timers
        self.battery_timer = self.create_timer(1.0, self.battery_timer_cb)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info(
            f'battery_manager_node started. State=EXPLORING, battery=100.0%. '
            f'Listening on "{self.completion_topic}" for exploration completion.'
        )

    # -------------------------------------------------------------- #
    # Odometry callback — tracks current speed for the drain model
    # -------------------------------------------------------------- #
    def on_odom(self, msg: Odometry):
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        self.last_linear_speed = math.hypot(lin.x, lin.y)
        self.last_angular_speed = abs(ang.z)

    # -------------------------------------------------------------- #
    # Battery drain / charge timer (1 Hz)
    # -------------------------------------------------------------- #
    def battery_timer_cb(self):
        is_moving = (
            self.last_linear_speed > self.vel_threshold
            or self.last_angular_speed > self.vel_threshold
        )

        if self.state in (RobotState.EXPLORING, RobotState.NAVIGATING_TO_CHARGER):
            if is_moving:
                self.battery_level = max(0.0, self.battery_level - self.drain_rate)
        elif self.state == RobotState.CHARGING:
            self.battery_level = min(self.full_battery, self.battery_level + self.charge_rate)
            if self.battery_level >= self.full_battery:
                self.get_logger().info('Battery fully charged (100%). Charging complete.')
                self.state = RobotState.DONE

        self.publish_battery_state()

        # Low-battery warning log (does not yet trigger an interrupt —
        # that is Option C / Phase 3 behavior, intentionally left as a
        # future extension point. See README note in battery_config.yaml).
        if self.state == RobotState.EXPLORING and self.battery_level <= self.low_battery_threshold:
            self.get_logger().warn(
                f'Battery at {self.battery_level:.1f}% — below low_battery_threshold '
                f'({self.low_battery_threshold}%). Auto-return-on-low-battery is not yet '
                f'enabled; this is logged for visibility only.'
            )

    def publish_battery_state(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.battery_level / 100.0)
        msg.voltage = float('nan')
        msg.present = True
        if self.state == RobotState.CHARGING:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.battery_level >= self.full_battery:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(msg)

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    # -------------------------------------------------------------- #
    # Exploration complete callback
    # -------------------------------------------------------------- #
    def on_exploration_complete(self, _msg: Empty):
        if self.state != RobotState.EXPLORING:
            # Already handled (or never started) — ignore duplicate/late
            # transient_local replays.
            self.get_logger().debug(
                f'Received exploration_complete while in state {self.state.name}; ignoring.'
            )
            return

        self.get_logger().info('Exploration complete event received.')
        self.state = RobotState.EXPLORATION_COMPLETE
        self.prompt_for_charger_pose()

    # -------------------------------------------------------------- #
    # Terminal input for charger pose (Option A) — runs in a separate
    # thread so it does not block the ROS executor / callbacks.
    # -------------------------------------------------------------- #
    def prompt_for_charger_pose(self):
        self.state = RobotState.WAITING_FOR_CHARGER_INPUT
        self._input_thread = threading.Thread(target=self._read_charger_input, daemon=True)
        self._input_thread.start()

    def _read_charger_input(self):
        print('\n' + '=' * 60)
        print('  EXPLORATION COMPLETE — charging station setup')
        print('=' * 60)
        print('  Enter the charging station coordinates in the MAP frame.')
        print('  (Tip: use RViz "Publish Point" or hover the mouse over')
        print('   the map to read coordinates from the bottom-left corner.)')
        print('=' * 60)

        x_val = None
        y_val = None
        while x_val is None:
            try:
                raw = input('  Charger X (meters): ').strip()
                x_val = float(raw)
            except ValueError:
                print('  Invalid number, try again.')

        while y_val is None:
            try:
                raw = input('  Charger Y (meters): ').strip()
                y_val = float(raw)
            except ValueError:
                print('  Invalid number, try again.')

        print(f'  Charger set to ({x_val:.2f}, {y_val:.2f}). Sending navigation goal...\n')
        self.send_nav_goal(x_val, y_val)

    # -------------------------------------------------------------- #
    # Nav2 action client — send goal, handle result
    # -------------------------------------------------------------- #
    def send_nav_goal(self, x: float, y: float, yaw: float = 0.0):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'navigate_to_pose action server not available after 10s. '
                'Is nav2.launch.py running? Returning to EXPLORATION_COMPLETE.'
            )
            self.state = RobotState.EXPLORATION_COMPLETE
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        # Identity orientation (yaw=0) is sufficient for docking approach in
        # most cases; extend here with a quaternion-from-yaw helper if a
        # specific heading is required for the physical charger contacts.
        goal_msg.pose.pose.orientation.w = 1.0

        self.state = RobotState.NAVIGATING_TO_CHARGER
        self.get_logger().info(f'Sending NavigateToPose goal: x={x:.2f}, y={y:.2f}')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_cb
        )
        send_goal_future.add_done_callback(self.nav_goal_response_cb)

    def nav_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining to charger: {remaining:.2f} m')

    def nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Charger goal was rejected by Nav2. Retrying in 5s...')
            self.state = RobotState.EXPLORATION_COMPLETE
            self.create_timer(5.0, self._retry_prompt_once)
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def _retry_prompt_once(self):
        # one-shot retry helper; create_timer fires repeatedly so we guard
        # re-entry by checking state
        if self.state == RobotState.EXPLORATION_COMPLETE:
            self.prompt_for_charger_pose()

    def nav_result_cb(self, future):
        status = future.result().status
        # status 4 == STATUS_SUCCEEDED in action_msgs/msg/GoalStatus
        if status == 4:
            self.get_logger().info('Reached charging station. Beginning charge.')
            self.state = RobotState.CHARGING
        else:
            self.get_logger().warn(
                f'NavigateToPose did not succeed (status={status}). '
                f'Returning to EXPLORATION_COMPLETE so you can re-enter coordinates.'
            )
            self.state = RobotState.EXPLORATION_COMPLETE
            self.prompt_for_charger_pose()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
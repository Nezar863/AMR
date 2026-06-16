#!/usr/bin/env python3
"""
return_to_start_node.py
-----------------------
Listens for the `exploration_complete` event published by frontier_exploration_ros2,
then autonomously navigates the robot back to the pose it had when this node started.

State machine:
    WAITING_FOR_TF      → TF map->base_link not yet available
    CAPTURING_START     → Recording the initial pose (first valid TF lookup)
    EXPLORING           → Exploration running; node is idle and listening
    RETURNING           → NavigateToPose action goal in flight
    DONE                → Robot reached start pose successfully
    FAILED              → Nav2 rejected/aborted the goal (logged, node stays alive)

Design decisions:
  - Start pose is captured from TF (map → base_link) the moment a valid
    transform is available, NOT from odometry.  Odometry drifts; the map
    frame is the authoritative frame Nav2 plans in, so it is the only
    correct frame to store a "return" waypoint in.
  - The subscription to `exploration_complete` uses QoS transient_local
    (depth=1, reliable) — identical to what frontier_exploration_ros2
    publishes.  This guarantees the message is received even if this node
    starts *after* exploration has already finished (late-joiner scenario).
  - A guard flag (_completion_received) prevents duplicate triggers from
    the transient_local replay mechanism.

Run AFTER:
    Terminal 1: ros2 launch amr_description spawn.launch.py
    Terminal 2: ros2 launch amr_description slam.launch.py
    Terminal 3: ros2 launch amr_description nav2.launch.py use_sim_time:=true
    Terminal 4: ros2 launch amr_description explore.launch.py
    Terminal 5: ros2 run amr_description return_to_start_node.py
             (or via return_to_start.launch.py — see launch/ directory)
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String
from nav2_msgs.action import NavigateToPose

# tf2 Python bindings — ships with ros-jazzy-tf2-ros
import tf2_ros
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Tiny helper: build a quaternion from a yaw angle (radians)
# ---------------------------------------------------------------------------
def _yaw_to_quaternion(yaw: float):
    """Returns (x, y, z, w) quaternion for a pure yaw rotation."""
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class RobotState(Enum):
    WAITING_FOR_TF  = auto()   # TF not yet ready
    CAPTURING_START = auto()   # About to record initial pose
    EXPLORING       = auto()   # Idle; waiting for exploration_complete
    RETURNING       = auto()   # NavigateToPose goal sent
    DONE            = auto()   # Reached start
    FAILED          = auto()   # Nav2 rejected / aborted


class ReturnToStartNode(Node):
    """
    Captures the robot's starting pose in the map frame at launch time,
    then navigates back to it when `exploration_complete` is received.
    """

    def __init__(self):
        super().__init__('return_to_start_node')

        # ------------------------------------------------------------------ #
        # Declare parameters (all overridable from a YAML config or CLI)
        # ------------------------------------------------------------------ #
        self.declare_parameter('completion_event_topic', 'exploration_complete')
        self.declare_parameter('nav_action_name',        'navigate_to_pose')
        self.declare_parameter('map_frame',              'map')
        self.declare_parameter('robot_frame',            'base_link')
        self.declare_parameter('tf_lookup_timeout_sec',  5.0)
        self.declare_parameter('nav_server_timeout_sec', 15.0)
        #self.declare_parameter('use_sim_time',           True)

        completion_topic     = self.get_parameter('completion_event_topic').value
        nav_action           = self.get_parameter('nav_action_name').value
        self._map_frame      = self.get_parameter('map_frame').value
        self._robot_frame    = self.get_parameter('robot_frame').value
        self._tf_timeout     = self.get_parameter('tf_lookup_timeout_sec').value
        self._nav_timeout    = self.get_parameter('nav_server_timeout_sec').value

        # ------------------------------------------------------------------ #
        # Internal state
        # ------------------------------------------------------------------ #
        self._state               = RobotState.WAITING_FOR_TF
        self._start_pose          = None   # geometry_msgs/PoseStamped in map frame
        self._completion_received = False  # guard against duplicate callbacks
        self._goal_handle         = None

        # ------------------------------------------------------------------ #
        # TF2 buffer + listener — used to capture starting pose
        # ------------------------------------------------------------------ #
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------ #
        # Subscriber: exploration_complete
        # Must mirror the publisher's QoS exactly:
        #   - transient_local + reliable + depth 1
        # so that a late-starting node still receives the event.
        # ------------------------------------------------------------------ #
        completion_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._completion_sub = self.create_subscription(
            Empty,
            completion_topic,
            self._on_exploration_complete,
            completion_qos,
        )

        # ------------------------------------------------------------------ #
        # Publisher: /return_to_start/status  (String — human-readable state)
        # ------------------------------------------------------------------ #
        self._status_pub = self.create_publisher(String, '/return_to_start/status', 10)

        # ------------------------------------------------------------------ #
        # Nav2 action client
        # ------------------------------------------------------------------ #
        self._nav_client = ActionClient(self, NavigateToPose, nav_action)

        # ------------------------------------------------------------------ #
        # Timer: attempt to capture start pose every 0.5 s until we have it
        # ------------------------------------------------------------------ #
        self._tf_poll_timer = self.create_timer(0.5, self._poll_for_start_pose)

        # ------------------------------------------------------------------ #
        # Timer: publish status every 2 s
        # ------------------------------------------------------------------ #
        self._status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f'return_to_start_node started.\n'
            f'  map_frame  : {self._map_frame}\n'
            f'  robot_frame: {self._robot_frame}\n'
            f'  completion topic: "{completion_topic}" (transient_local)\n'
            f'  nav action : "{nav_action}"\n'
            f'Waiting for TF map → {self._robot_frame} to become available...'
        )

    # ======================================================================
    # TF polling — runs every 0.5 s until start pose is captured
    # ======================================================================

    def _poll_for_start_pose(self):
        """
        Tries to get a transform from map → base_link.
        Once successful, records it as the start pose and cancels this timer.
        """
        if self._state not in (RobotState.WAITING_FOR_TF, RobotState.CAPTURING_START):
            # Already captured or in a later state — stop polling
            self._tf_poll_timer.cancel()
            return

        self._state = RobotState.CAPTURING_START

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),                    # latest available
                timeout=Duration(seconds=self._tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().debug(
                f'TF map→{self._robot_frame} not yet available: {e}'
            )
            return

        # Build a PoseStamped from the transform
        t = transform.transform.translation
        r = transform.transform.rotation

        start = PoseStamped()
        start.header.frame_id = self._map_frame
        start.header.stamp    = self.get_clock().now().to_msg()
        start.pose.position.x = t.x
        start.pose.position.y = t.y
        start.pose.position.z = 0.0
        start.pose.orientation.x = r.x
        start.pose.orientation.y = r.y
        start.pose.orientation.z = r.z
        start.pose.orientation.w = r.w

        self._start_pose = start
        self._state      = RobotState.EXPLORING

        self.get_logger().info(
            f'✅  Start pose captured in "{self._map_frame}" frame:\n'
            f'   x={t.x:.3f}  y={t.y:.3f}  '
            f'yaw≈{math.atan2(2*(r.w*r.z + r.x*r.y), 1 - 2*(r.y**2 + r.z**2)):.3f} rad\n'
            f'Now monitoring "exploration_complete" topic...'
        )

        # No longer need the TF poll timer
        self._tf_poll_timer.cancel()

    # ======================================================================
    # Exploration-complete callback
    # ======================================================================

    def _on_exploration_complete(self, _msg: Empty):
        """
        Called when frontier_exploration_ros2 publishes on `exploration_complete`.
        Guards against duplicate/transient_local replay triggers.
        """
        if self._completion_received:
            self.get_logger().debug(
                'exploration_complete received again — already handled, ignoring.'
            )
            return

        if self._state == RobotState.WAITING_FOR_TF:
            self.get_logger().warn(
                'exploration_complete received but start pose has not been captured yet '
                '(TF map→base_link not available). Cannot return to start. '
                'Is SLAM running? Is the robot spawned?'
            )
            return

        if self._start_pose is None:
            self.get_logger().error(
                'exploration_complete received but _start_pose is None. '
                'This is a bug — please report it.'
            )
            return

        self._completion_received = True
        self._state = RobotState.RETURNING

        sp = self._start_pose.pose.position
        self.get_logger().info(
            f'🗺️  Exploration complete!  Returning to start pose: '
            f'x={sp.x:.3f}  y={sp.y:.3f}'
        )

        self._send_nav_goal()

    # ======================================================================
    # Nav2 goal sending
    # ======================================================================

    def _send_nav_goal(self):
        """Waits for the Nav2 action server and sends a NavigateToPose goal."""
        self.get_logger().info(
            f'Waiting for navigate_to_pose action server '
            f'(timeout {self._nav_timeout:.0f} s)...'
        )

        if not self._nav_client.wait_for_server(timeout_sec=self._nav_timeout):
            self.get_logger().error(
                f'navigate_to_pose server not available after {self._nav_timeout:.0f} s. '
                f'Is nav2.launch.py running? State → FAILED.'
            )
            self._state = RobotState.FAILED
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._start_pose
        # Refresh the timestamp so Nav2 does not complain about stale goals
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        sp = self._start_pose.pose.position
        self.get_logger().info(
            f'📍  Sending NavigateToPose goal → '
            f'x={sp.x:.3f}  y={sp.y:.3f}  frame="{self._map_frame}"'
        )

        future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb,
        )
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'↩️  Returning to start — distance remaining: {remaining:.2f} m',
            throttle_duration_sec=5.0,
        )

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'NavigateToPose goal was REJECTED by Nav2. '
                'Check that Nav2 is fully active and the start pose is reachable. '
                'State → FAILED.'
            )
            self._state = RobotState.FAILED
            return

        self.get_logger().info('Nav2 accepted the return-to-start goal.')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        # action_msgs/msg/GoalStatus: SUCCEEDED = 4
        status = future.result().status

        if status == 4:
            sp = self._start_pose.pose.position
            self.get_logger().info(
                f'🏠  Robot reached start pose (x={sp.x:.3f}, y={sp.y:.3f}). '
                f'Mission complete!'
            )
            self._state = RobotState.DONE
        else:
            self.get_logger().warn(
                f'NavigateToPose ended with status={status} (not SUCCEEDED). '
                f'Possible causes: path blocked, costmap issue, Nav2 timeout. '
                f'State → FAILED. Check Nav2 logs for details.'
            )
            self._state = RobotState.FAILED

    # ======================================================================
    # Status publisher
    # ======================================================================

    def _publish_status(self):
        msg = String()
        msg.data = self._state.name
        self._status_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ReturnToStartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
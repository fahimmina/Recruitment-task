#!/usr/bin/env python3
import asyncio
import json
import math
import queue
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import websockets

# Waypoint is considered reached once within this distance (meters).
DISTANCE_TOLERANCE = 0.15

# Below this heading error (radians), the controller starts driving forward
# instead of turning in place.
HEADING_ALIGN_THRESHOLD = 0.15

ANGULAR_KP = 1.5
LINEAR_KP = 0.5
MAX_LINEAR_SPEED = 0.3
MAX_ANGULAR_SPEED = 1.0
CONTROL_PERIOD = 0.1


def _normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _quaternion_to_yaw(q):
    """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def go_to_goal(pose, target):
    """Proportional go-to-goal step.

    pose: (x, y, yaw) of the robot right now.
    target: dict with 'x', 'y' of the waypoint.
    Returns ((linear, angular), reached).
    """
    x, y, yaw = pose
    dx = target['x'] - x
    dy = target['y'] - y
    distance = math.hypot(dx, dy)

    if distance < DISTANCE_TOLERANCE:
        return (0.0, 0.0), True

    heading_to_target = math.atan2(dy, dx)
    heading_error = _normalize_angle(heading_to_target - yaw)

    angular = _clamp(ANGULAR_KP * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

    if abs(heading_error) < HEADING_ALIGN_THRESHOLD:
        linear = _clamp(LINEAR_KP * distance, 0.0, MAX_LINEAR_SPEED)
    else:
        linear = 0.0

    return (linear, angular), False


class WaypointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')

        self.declare_parameter('uri', 'ws://localhost:8765')
        self.uri = self.get_parameter('uri').get_parameter_value().string_value

        self.waypoints = None
        self.pose = None
        self.current_wp_index = 0
        self.mission_complete = False

        self._raw_queue = queue.Queue()
        self._stop_event = threading.Event()

        self._ws_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._ws_thread.start()

        self._cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        self._timer = self.create_timer(0.2, self._drain_queue)
        self._control_timer = self.create_timer(CONTROL_PERIOD, self._control_loop)

    def _run_event_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._connect_loop())
        finally:
            loop.close()

    async def _connect_loop(self):
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self.uri) as websocket:
                    self.get_logger().info(f'Connected to {self.uri}')
                    async for message in websocket:
                        self._raw_queue.put(message)
            except (OSError, websockets.exceptions.WebSocketException) as e:
                self.get_logger().warn(f'WebSocket connection error: {e}; retrying in 2s')
                await asyncio.sleep(2.0)

    def _drain_queue(self):
        while True:
            try:
                raw = self._raw_queue.get_nowait()
            except queue.Empty:
                return

            if self.waypoints is not None:
                # Already parsed the mission once; ignore rebroadcasts.
                continue

            try:
                payload = json.loads(raw)
                waypoints = payload['waypoints']
            except (json.JSONDecodeError, KeyError, TypeError) as e:
                self.get_logger().warn(f'Failed to parse waypoint payload: {e}')
                continue

            self.waypoints = waypoints
            self.get_logger().info(f'Parsed {len(waypoints)} waypoints:')
            for i, wp in enumerate(waypoints):
                self.get_logger().info(
                    f'  [{i}] x={wp["x"]:.3f} y={wp["y"]:.3f} yaw={wp["yaw"]:.4f}'
                )

    def _odom_callback(self, msg):
        position = msg.pose.pose.position
        yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        self.pose = (position.x, position.y, yaw)

    def _control_loop(self):
        if self.mission_complete or self.waypoints is None or self.pose is None:
            return
        if self.current_wp_index >= len(self.waypoints):
            return

        target = self.waypoints[self.current_wp_index]
        (linear, angular), reached = go_to_goal(self.pose, target)
        self._publish_cmd(linear, angular)

        if reached:
            self.get_logger().info(f'Reached waypoint {self.current_wp_index}')
            self.current_wp_index += 1

            if self.current_wp_index >= len(self.waypoints):
                self._publish_cmd(0.0, 0.0)
                self.mission_complete = True
                self.get_logger().info('All waypoints reached')

    def _publish_cmd(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self._cmd_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

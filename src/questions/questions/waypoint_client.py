#!/usr/bin/env python3
import asyncio
import json
import math
import queue
import threading

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

# --- Vision (Stage 3) -------------------------------------------------
# Camera topic verified live via `ros2 topic list` against the running
# sim (turtlebot4_gz_bringup's camera_bridge remaps the OAK-D's Gazebo
# feed to this name) -- not guessed.
CAMERA_TOPIC = '/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = '/oakd/rgb/preview/depth'

# HSV ranges for the three sphere colors defined in level3.sdf (rock_01
# red, rock_02 green, rock_03 yellow/largest -- checked in the world file
# rather than assumed). Saturation/value minimums are set high enough to
# reject the maze's gray walls and pillars (low-saturation grays) while
# still catching the spheres under the scene's dim point light; tuned
# against this world, not universal constants.
COLOR_RANGES = {
    'RED': [
        ((0, 70, 50), (10, 255, 255)),
        ((170, 70, 50), (179, 255, 255)),  # red hue wraps around 0/179
    ],
    'GREEN': [((40, 70, 50), (85, 255, 255))],
    'YELLOW': [((20, 70, 50), (35, 255, 255))],
}

# Ignore contours smaller than this -- filters out stray noise pixels so
# a handful of misclassified pixels can't masquerade as a sphere.
MIN_SPHERE_CONTOUR_AREA = 50.0


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


def detect_spheres(cv_image, depth_image=None):
    """Segment the three colored spheres.

    Returns {color: {'area': px, 'score': size_score, 'clipped': bool}}.

    Converts to HSV before thresholding (rather than matching raw BGR)
    because hue stays roughly constant across a shaded sphere's surface
    even though its BGR values swing a lot between the lit side and the
    shadowed side -- HSV separates "what color" from "how bright", which
    is exactly the split a single point light in the scene needs.

    'score' is depth-normalized when a depth frame is available, not
    just raw pixel area. Raw pixel area alone is misleading whenever the
    spheres sit at different distances: apparent area of a fixed-size
    object shrinks with 1/distance^2, so area * distance^2 gives a size
    estimate that's comparable across spheres regardless of how far each
    happens to be from the camera.

    'clipped' flags a blob whose bounding box touches the image border.
    This matters even with depth normalization: a sphere half-outside
    the frame only contributes the *visible fraction* of its pixels, so
    both its raw area and its depth-normalized score are systematic
    underestimates, not just noisy ones -- no amount of rescaling
    recovers the missing half. Confirmed against this exact world: at
    the final waypoint the larger yellow sphere sat mostly outside the
    frame (a 32x127px sliver) while the smaller red sphere was fully
    visible, so the caller must exclude clipped blobs from the "largest"
    comparison rather than trust their score.
    """
    height, width = cv_image.shape[:2]
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    detections = {}
    for color, ranges in COLOR_RANGES.items():
        mask = None
        for lower, upper in ranges:
            part = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        # A single sphere can split into more than one contour (e.g. a
        # specular highlight breaking the mask in two), so compare each
        # color's single largest contour, not the sum of all of them --
        # summing would let a color with many small false-positive
        # speckles outscore a real, solid sphere of another color.
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < MIN_SPHERE_CONTOUR_AREA:
            continue

        x, y, bw, bh = cv2.boundingRect(largest)
        clipped = x <= 0 or y <= 0 or (x + bw) >= width - 1 or (y + bh) >= height - 1

        score = area
        if depth_image is not None:
            # Median depth over the whole blob, not a single centroid
            # pixel -- a lone sample can land on a specular highlight or
            # right on the contour edge, both noisy spots for depth.
            blob_mask = np.zeros(depth_image.shape, dtype=np.uint8)
            cv2.drawContours(blob_mask, [largest], -1, 255, thickness=cv2.FILLED)
            depths = depth_image[blob_mask == 255]
            depths = depths[np.isfinite(depths) & (depths > 0)]
            if depths.size > 0:
                distance = float(np.median(depths))
                score = area * (distance ** 2)

        detections[color] = {'area': area, 'score': score, 'clipped': clipped}

    return detections


class WaypointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')

        self.declare_parameter('uri', 'ws://localhost:8765')
        self.uri = self.get_parameter('uri').get_parameter_value().string_value

        self.waypoints = None
        self.pose = None
        self.current_wp_index = 0
        self.mission_complete = False

        self._cv_bridge = CvBridge()
        self._camera_sub = None
        self._depth_sub = None
        self._latest_depth = None

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
                self._start_vision()

    def _start_vision(self):
        # Subscribed only now (not from __init__) so the camera/depth
        # pipeline is idle during navigation -- vision is a one-shot
        # check that only makes sense once the robot has actually
        # arrived.
        self.get_logger().info('Starting vision detection...')
        self._depth_sub = self.create_subscription(
            Image, DEPTH_TOPIC, self._depth_callback, 10
        )
        self._camera_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self._camera_callback, 10
        )

    def _depth_callback(self, msg):
        self._latest_depth = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def _camera_callback(self, msg):
        if self._latest_depth is None:
            # Wait for at least one depth frame before scoring -- without
            # it we'd silently fall back to raw pixel area, which is the
            # exact failure mode this depth normalization exists to fix.
            return

        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert camera image: {e}')
            return

        detections = detect_spheres(cv_image, self._latest_depth)

        # One-shot: a single good frame is enough, and detaching here
        # stops the callbacks from re-running on every subsequent frame
        # the camera/depth sensors keep publishing.
        self.destroy_subscription(self._camera_sub)
        self.destroy_subscription(self._depth_sub)
        self._camera_sub = None
        self._depth_sub = None

        if not detections:
            self.get_logger().warn('Vision: no spheres detected in frame.')
            return

        if len(detections) < 3:
            self.get_logger().warn(
                f'Vision: only {len(detections)}/3 spheres visible '
                '(partial occlusion or out of frame) -- reporting from what is visible.'
            )

        clipped_colors = [c for c, d in detections.items() if d['clipped']]
        for color in clipped_colors:
            self.get_logger().warn(
                f'Vision: {color} sphere is clipped by the frame edge -- its visible '
                'area underestimates its true size, excluding it from the size comparison.'
            )

        # Only compare spheres whose full silhouette is in frame; a
        # clipped blob's score is a guaranteed underestimate (see
        # detect_spheres docstring), so including it could crown the
        # wrong color "largest" even after depth normalization.
        reliable = {c: d for c, d in detections.items() if not d['clipped']}
        if not reliable:
            self.get_logger().warn(
                'Vision: every detected sphere is clipped by the frame edge -- '
                'falling back to all detections despite unreliable sizing.'
            )
            reliable = detections

        largest_color = max(reliable, key=lambda c: reliable[c]['score'])
        result = reliable[largest_color]
        self.get_logger().info(
            f'Largest sphere detected: {largest_color} '
            f'(pixel_area={result["area"]:.0f}px, '
            f'depth_normalized_score={result["score"]:.0f})'
        )

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

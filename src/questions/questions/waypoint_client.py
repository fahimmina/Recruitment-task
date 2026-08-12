#!/usr/bin/env python3
import asyncio
import json
import queue
import threading

import rclpy
from rclpy.node import Node
import websockets


class WaypointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')

        self.declare_parameter('uri', 'ws://localhost:8765')
        self.uri = self.get_parameter('uri').get_parameter_value().string_value

        self.waypoints = None
        self._raw_queue = queue.Queue()
        self._stop_event = threading.Event()

        self._ws_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._ws_thread.start()

        self._timer = self.create_timer(0.2, self._drain_queue)

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

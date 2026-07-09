#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
import threading

class WebsocketBroadcaster(Node):
    def __init__(self):
        super().__init__('websocket_broadcaster')
        
        # Format the data as a JSON dictionary
        self.payload = {
            "waypoints": [
                {"x": -5.0, "y": 0.0, "yaw": 0.0},
                {"x": -3.0, "y": 0.8, "yaw": 0.0},
                {"x": -0.9, "y": 0.6, "yaw": -0.3},
                {"x": 0.5, "y": -0.6, "yaw": 0.2},
                {"x": 2.1, "y": 0.3, "yaw": 0.4},
                {"x": 3.0, "y": 1.0, "yaw": 0.5},
                {"x": 4.2, "y": 1.5, "yaw": 1.2},
                {"x": 4.9, "y": 2.0, "yaw": 1.5708}
            ]
        }
        self.get_logger().info('Starting WebSocket server on ws://0.0.0.0:8765')

    async def broadcast_handler(self, websocket, path):
        """Continuously sends the JSON data to any connected client"""
        try:
            while True:
                # Convert dict to JSON string and send
                await websocket.send(json.dumps(self.payload))
                # Broadcast every 2 seconds
                await asyncio.sleep(2.0)
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("A client disconnected.")

    def start_server(self):
        """Starts the asyncio event loop for the websocket server"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        start_ws = websockets.serve(self.broadcast_handler, "0.0.0.0", 8765)
        
        loop.run_until_complete(start_ws)
        loop.run_forever()

def main(args=None):
    rclpy.init(args=args)
    node = WebsocketBroadcaster()
    
    # Run the WebSocket server in a separate daemon thread
    # This allows rclpy to spin normally in the main thread if needed later
    ws_thread = threading.Thread(target=node.start_server, daemon=True)
    ws_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
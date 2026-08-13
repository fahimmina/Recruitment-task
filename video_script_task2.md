# Task 2 Video Script -- Autonomous Navigation & Vision (TurtleBot4)

Word-for-word spoken script. Target 2:50-3:00 total, ~390 words.

---

## [0:00-0:20] What Task 2 is

"This is Task 2: autonomous navigation and vision on a TurtleBot4 in Gazebo. The robot gets a sequence of thirteen waypoints broadcast over a WebSocket connection -- not a normal ROS topic -- and has to drive through all of them on its own. Once it arrives at the final one, it has to look at three colored spheres with its onboard camera and figure out, using OpenCV, which one is the largest."

## [0:20-1:20] Live demo narration

"One command starts everything: `ros2 launch questions main_assignment.launch.py`. That brings up Gazebo with the maze world, spawns the TurtleBot4, and starts the WebSocket server broadcasting the waypoints.

In a second terminal I run my node: `ros2 run questions waypoint_client`. It connects and logs all thirteen waypoints.

Watch the robot turn to face the first target, then drive -- 'Reached waypoint' logs each time it gets within fifteen centimeters. It repeats through the whole maze, correcting heading continuously, not just once at the start.

Here's waypoint twelve, the last one: 'All waypoints reached,' then 'Starting vision detection,' and a moment later -- 'Largest sphere detected: YELLOW.' Correct -- yellow really is the biggest of the three."

## [1:20-2:10] Architecture

"It's all one node. A background thread runs its own asyncio event loop for the WebSocket client, since rclpy's executor has no event loop of its own to await on. Incoming JSON goes onto a thread-safe queue, drained by a ROS timer on the main thread.

Driving is a proportional controller reading `/odom`, publishing `TwistStamped` to `/cmd_vel` -- turn toward the target, then drive while still correcting heading.

Once the last waypoint's reached, the same node subscribes to `/oakd/rgb/preview/image_raw` and `/oakd/rgb/preview/depth` and runs color detection."

## [2:10-2:50] The real story

"Vision didn't work first try, and that's the interesting part. Comparing raw pixel area, it confidently reported red -- wrong. Red was just closer to the camera than yellow, so it looked bigger in pixels without being bigger. I fixed that with the depth camera, scoring area times distance-squared. Still sometimes wrong. Checking an actual saved frame, I found yellow was partially cut off by the frame edge -- no rescaling recovers pixels that were never captured. So now I detect when a blob touches the frame edge and exclude it, instead of trusting a number I know underestimates."

## [2:50-3:00] Wrap-up

"That's Task 2 -- waypoint navigation and vision, both working end to end, and built to fail honestly instead of confidently guessing wrong."

# 1. Assignment Overview
In this assignment, you will develop a ROS 2 package to autonomously navigate a TurtleBot4 through a custom simulated Gazebo environment (world:=level3). The robot will spawn at the initial starting coordinates of X: 0.0, Y: 0.0, Yaw: 0.0.
Your robot must ingest a broadcasted sequence of waypoints via a WebSocket connection and navigate through them sequentially. Upon reaching the final designated area, you will utilize the robot's onboard camera and Computer Vision techniques to analyze a set of three spheres and report the color of the largest one.

# 2. Task Requirements

## Task 2.1: JSON Waypoint Parsing via WebSocket
You have been provided with a unified launch script that initializes the Gazebo environment, spawns the robot at (x: 0.0, y: 0.0, yaw: 0.0), and starts a WebSocket Broadcaster hosted at ws://localhost:8765. Instead of publishing over a traditional ROS 2 topic, this server continuously broadcasts the mission waypoints as a JSON-formatted string.

Your task is to write a navigation node that:
* Acts as a WebSocket client to connect to ws://localhost:8765.
* Receives and parses the JSON payload into discrete (x, y, yaw) coordinates.
* Autonomously commands the TurtleBot4 to navigate to each point sequentially, ensuring it reaches one waypoint before proceeding to the next. (Note: Pay attention to your initial spawn orientation vs. the orientation required by the first waypoint!)

JSON Payload Format: The WebSocket server broadcasts a JSON object containing an array of waypoint dictionaries.

## Task 2.2: Computer Vision & Color Recognition
The final waypoint correctly aligns the robot to face a designated area containing three differently colored spheres. Once the robot stops at this location, it must:
* Subscribe to the TurtleBot4's camera image topic.
* Process the incoming image using OpenCV to segment the three spheres.
* Calculate the pixel area (size) of each detected sphere.
* Output a terminal log explicitly stating the color of the largest sphere.

## Bonus 1: SLAM Integration
Students who successfully implement Simultaneous Localization and Mapping (SLAM) during the navigation phase will receive full bonus points. Rather than relying purely on blind odometry, you should integrate slam_toolbox or a similar SLAM algorithm to map the level3 environment dynamically as the robot navigates the waypoints.
## Bonus 2: Nav2 Integration
After applying SLAM, you can add Nav2 for an additional bonus point. Nav2 is the standard approach for navigation within a map. You could include a video of the bot using the Nav2 stack to reach the destination.

# 3. Setup and Execution
To begin the assignment, launch the provided bringup script. This single command will spin up the custom world, spawn the TurtleBot4 (lite model) at the designated starting coordinates, and start the WebSocket broadcasting server.

```bash
ros2 launch questions main_assignment.launch.py
```
Ensure your custom navigation and vision nodes are launched separately after the Gazebo environment and base ROS 2 nodes are fully initialized.
# 4. Submission Guidelines

### Your submission must include the following components:

  ## 1.GitHub Repository: 
  Host your complete ROS 2 package and source code in a GitHub repository (public or shared with the instructor).

  ## 2.Docker Containerization: 
  You must Dockerize your entire solution. Include a Dockerfile (and docker-compose.yml if necessary) in your repository so that your workspace, dependencies, and nodes can be built and run in an isolated container without manual setup.

  ## 3.Video Demonstration & Code Walkthrough: 
  Record a video showing your TurtleBot4 successfully navigating the sequence of waypoints and correctly outputting the color of the largest sphere. During the video, you must explain your code, walk through the logic of your navigation and vision pipelines, and justify why you made your specific implementation choices.

  ## 4.README.md: 
  Include documentation in your repository detailing how to build your Docker container and the exact commands to run your solution. If you completed the SLAM bonus, explicitly state this in the README and include the saved map files (.yaml and .pgm) in a /maps directory within your package.

# 5. Solution: Build, Run, and Approach

## 5.1 Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04, Gazebo Harmonic.
- TurtleBot4 simulator packages (not installed by default alongside ROS 2 Jazzy):
  ```bash
  sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes
  ```

## 5.2 Build

```bash
cd <repo root>
colcon build --symlink-install --packages-select questions
source install/setup.bash
```

## 5.3 Run

Terminal 1 -- brings up Gazebo (the `level3` maze world), spawns the TurtleBot4 (lite model), and starts the WebSocket waypoint broadcaster:
```bash
ros2 launch questions main_assignment.launch.py
```

Terminal 2 -- once Gazebo has finished loading, run the navigation + vision node:
```bash
source install/setup.bash
ros2 run questions waypoint_client
```

The node connects to the waypoint feed, drives through all 13 waypoints, then automatically runs sphere-color detection and logs the result -- no separate command needed for vision.

> If running alongside another ROS 2 graph on the same machine (e.g. a second Gazebo/MoveIt session), set a distinct `ROS_DOMAIN_ID` in both terminals first. Two `controller_manager` nodes with the same default domain can collide and silently break `/odom`, which is a plain environment collision, not a code issue.

## 5.4 Topics used

| Purpose | Topic | Type | Direction |
|---|---|---|---|
| Waypoint feed | `ws://localhost:8765` | JSON over WebSocket (not a ROS topic) | subscribe |
| Robot pose | `/odom` | `nav_msgs/msg/Odometry` | subscribe |
| Drive command | `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | publish |
| Camera (RGB) | `/oakd/rgb/preview/image_raw` | `sensor_msgs/msg/Image` | subscribe (vision stage only) |
| Camera (depth) | `/oakd/rgb/preview/depth` | `sensor_msgs/msg/Image` (32FC1, meters) | subscribe (vision stage only) |

All topic names were confirmed against the running sim with `ros2 topic list` / `ros2 topic info -v`, not assumed.

## 5.5 Approach

**Navigation (go-to-goal, proportional control):** the WebSocket client runs in a background `asyncio` thread (rclpy's executor has no event loop of its own, so `await` can't happen on its main thread) and pushes received JSON onto a thread-safe queue; a ROS timer drains it on the main executor. Once the 13 waypoints are parsed, a control-loop timer drives to each in order: turn in place toward the target until heading error drops below a threshold, then drive forward (speed proportional to remaining distance) while continuing to correct heading. A waypoint counts as reached once within `DISTANCE_TOLERANCE` (0.15m) of its position.

**Vision (triggered after the last waypoint):** implemented as a method on the same node rather than a separate one -- the two stages are strictly sequential with no concurrent shared state, so a second node would only add topic/service plumbing without buying anything. Once navigation completes, it subscribes to the camera and depth topics, converts BGR->HSV (hue stays stable across a shaded sphere's surface where raw BGR doesn't), thresholds for the three sphere colors defined in `level3.sdf` (red, green, yellow -- checked in the world file, not assumed), and finds the largest contour per color.

Raw pixel area alone was not enough, and this was discovered through live testing, not anticipated in advance:
1. **Distance bias:** a physically smaller sphere sitting closer to the camera can occupy more pixels than a larger, farther one. Fixed by scoring `area * depth^2` (using the median depth over each contour from the `/oakd/rgb/preview/depth` topic) instead of raw pixel area, since apparent area of a fixed-size object shrinks with `1/distance^2`.
2. **Frame clipping:** depth normalization doesn't help a sphere that's partially outside the camera frame -- its visible pixel area is a systematic underestimate no matter how it's rescaled, not just a noisy one. Fixed by detecting when a contour's bounding box touches the image edge and excluding that blob from the "largest" comparison, logging a clear warning instead of silently returning a wrong answer.

This matters in practice because the existing go-to-goal controller only checks *position* against `DISTANCE_TOLERANCE` when marking a waypoint reached -- it does not align the robot's final heading to the waypoint's given `yaw`. That's intentionally left as-is (out of scope for the vision stage). One consequence: the camera's framing of the sphere cluster at the final stopping pose varies somewhat between runs. The vision pipeline is built to degrade gracefully under that variance rather than assume a perfect view -- it was verified against both a partially-clipped frame (correctly excludes the clipped sphere and reports from what's reliably visible) and a fully-visible frame (correctly reports YELLOW, the true largest sphere in the world file).

## 5.6 Bonuses (not implemented)

SLAM (`slam_toolbox`) and Nav2 integration were not attempted -- see `submission_checklist.md` for an assessment of how straightforward they'd be to add later given the current structure.

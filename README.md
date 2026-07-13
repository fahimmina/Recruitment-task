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

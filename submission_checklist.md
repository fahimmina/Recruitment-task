# Task 2 Submission Checklist (TurtleBot4 Autonomous Navigation & Vision)

## 1. GitHub repository

- [x] Fork of the assignment repo, hosted at `github.com/fahimmina/Recruitment-task`.
- [x] Work done on branch `feature/waypoint-client`, PR #1 open against the fork's `main`.
- [x] PR #1 contains all three commits for this stage of work:
  - `Add WebSocket waypoint client node` -- WebSocket client, thread-safe queue, one-shot parse.
  - `Add go-to-goal proportional controller to waypoint_client` -- navigation, live-verified reaching all 13 waypoints.
  - `Add vision node: sphere color detection triggered after navigation` -- HSV + depth-normalized + edge-clipping-aware sphere detection, live-verified detecting YELLOW correctly.
- [ ] PR merged into the fork's `main` (currently left open for review; merge before final submission if a merged `main` is expected).

## 2. .gitignore

- [x] `build/`, `install/`, `log/` excluded (colcon workspace artifacts).
- [x] `__pycache__/`, `*.pyc` excluded.
- [x] `.vscode/`, `.idea/`, OS cruft (`.DS_Store`, `Thumbs.db`) excluded.
- [x] Confirmed nothing under `build/`/`install/`/`log/` is tracked in git (`git status` clean after builds).

## 3. Docker containerization

**Status: not yet done for this package.** `inter_task_1` (a separate repo/task) has a working `Dockerfile` + `docker-compose.yml` built on `osrf/ros:jazzy-desktop`, and its comments explicitly anticipated extending it for later tasks (`# COPY task2_pkg ./task2_pkg <- uncomment once Task 2's package exists`). That pattern is directly reusable here, but it lives in a different repository than this one (`Recruitment-task`), so it needs to be **copied over and adapted**, not just uncommented in place.

What's needed to close this out:
- [ ] Add a `Dockerfile` to this repo (`Recruitment-task`), based on `osrf/ros:jazzy-desktop`.
- [ ] Install `ros-jazzy-turtlebot4-simulator` and `ros-jazzy-irobot-create-nodes` in the image (not part of Task 1's Dockerfile, since Task 1 didn't need TurtleBot4 -- this is the one real addition beyond copy-and-adapt).
- [ ] `COPY src/questions ./src/questions` and `colcon build --symlink-install` inside the image, mirroring Task 1's pattern.
- [ ] Verify GUI passthrough (Gazebo + `gnome-screenshot`-style verification isn't available in a headless container) -- decide whether the container runs headless (`--headless-rendering`) for grading/CI, or forwards X11/Wayland for a visible demo. Task 1's Dockerfile doesn't address GPU/display passthrough either, so this is new work, not reuse.
- [ ] `docker-compose.yml` if multi-service orchestration (sim + nav node as separate containers) ends up being wanted; a single-container `CMD` running the launch file is likely sufficient here since both nodes are lightweight.

## 4. Video demonstration

- [x] Script written: `video_script_task2.md` (word-for-word, ~3 min).
- [ ] Actual screen recording done against a live run.
- [ ] Recording shows: launch command, robot navigating the maze, arrival at the final waypoint, vision log line with the correct color.
- [ ] Code walkthrough included per the assignment's requirement (script covers this in the 1:20-2:10 and 2:10-2:50 sections).

## 5. README.md

- [x] Build instructions (`colcon build --symlink-install --packages-select questions`).
- [x] Exact run commands (two terminals: launch, then `ros2 run questions waypoint_client`).
- [x] Topic names and message types used (waypoint feed, `/odom`, `/cmd_vel`, camera RGB + depth).
- [x] Approach explanation for both navigation and vision, including the depth-normalization and edge-clipping design decisions and why they were needed.
- [x] SLAM bonus section present and honestly states it was **not** attempted (no false claim of completion, no `/maps` directory needed).

## 6. Bonus feasibility assessment (not implemented, per instructions)

- **SLAM (`slam_toolbox`)**: straightforward to bolt on. The robot already publishes `/odom` and the TurtleBot4 bringup provides `/scan` (RPLIDAR). Adding `slam_toolbox`'s launch file alongside the existing `main_assignment.launch.py` (or as a third terminal) requires no changes to `waypoint_client.py` -- SLAM builds its map independently of how the robot is driven. Main follow-up: confirm `/scan`'s exact frame/QoS and pick sync vs. async mode.
- **Nav2**: more work, but the pieces line up reasonably well. Nav2 needs a costmap (from SLAM's map), a controller/planner config tuned for the TurtleBot4's footprint (already published via `/robot_description`), and waypoint goals -- which map naturally onto the *same* `(x, y, yaw)` dicts already being parsed from the WebSocket feed, just fed to a `NavigateToPose`/`FollowWaypoints` action client instead of the hand-written `go_to_goal()` proportional controller. The clean separation between "get waypoints" (`_drain_queue`) and "act on waypoints" (`_control_loop` / `go_to_goal`) in the current code means swapping the driving mechanism wouldn't require touching the WebSocket or vision stages at all.
- Neither was attempted here given time constraints, in line with the instruction to keep Stage 3 (vision) solid before touching bonus work.

## 7. Pre-submission checklist (literal)

- [x] `ros2 launch questions main_assignment.launch.py` boots cleanly with no code changes needed to the provided launch/world files.
- [x] `ros2 run questions waypoint_client` connects to the WebSocket feed and logs all 13 parsed waypoints exactly once (rebroadcasts ignored).
- [x] Robot reaches all 13 waypoints in order (multiple live runs confirmed).
- [x] Vision fires automatically after the last waypoint, with no manual trigger needed.
- [x] Vision correctly reports the true largest sphere (YELLOW) on a live run with a clear camera view.
- [x] Vision degrades gracefully (no crash, clear log warning) when the camera view is partially clipped, instead of silently reporting a wrong answer.
- [x] `colcon test` (flake8/pep257) passes with zero findings in `waypoint_client.py` (pre-existing findings in `websocket_broadcaster.py`/`setup.py` are out of scope, not introduced by this work).
- [x] All work pushed to PR #1 on the fork.
- [ ] Dockerfile added and build verified.
- [ ] Video recorded from the written script.
- [ ] Final PR review / merge to `main`.

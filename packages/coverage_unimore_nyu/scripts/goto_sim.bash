#! /bin/bash

# Go to (Not Relative)
rostopic pub -1 /hummingbird1/autopilot/pose_command geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: -0.50
    y: 0.0
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" &
rostopic pub -1 /hummingbird2/autopilot/pose_command geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.5
    y: 0.5
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: -0.9227
    w: 0.3855" &
rostopic pub -1 /hummingbird3/autopilot/pose_command geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.5
    y: -0.5
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.9227
    w: 0.3855" &
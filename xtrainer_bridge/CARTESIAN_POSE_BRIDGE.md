# Cartesian pose bridge

`cartesian_pose_bridge` adapts the Dobot driver's Cartesian API to standard ROS
messages. It accepts a `geometry_msgs/msg/Pose` or `PoseStamped` command, sends
the command to the driver's `ServoP` service, and republishes
`dobot_msgs_v4/msg/ToolVectorActual` feedback as `geometry_msgs/msg/PoseStamped`.

The bridge is a direct command interface. It does not plan a trajectory or use
MoveIt. The latest pose is retained while one `ServoP` request is in flight, so
high-rate input does not create an unbounded service queue.

## Build

From the workspace root:

```bash
colcon build --packages-select xtrainer_bridge
source install/setup.bash
```

The driver must already be running. For the normal dual-arm launch, start it
with `ros2 launch xtrainer_control start.launch.py`.

## Start one arm

For Arm1 using `PoseStamped` commands:

```bash
ros2 launch xtrainer_bridge cartesian_pose_bridge.launch.py \
  arm_namespace:=Arm1 command_type:=PoseStamped
```

For Arm2:

```bash
ros2 launch xtrainer_bridge cartesian_pose_bridge.launch.py \
  arm_namespace:=Arm2 command_type:=PoseStamped
```

The default Arm1 topics and service are:

```text
/Arm1/cartesian_pose_command       geometry_msgs/msg/PoseStamped
/Arm1/cartesian_pose_state          geometry_msgs/msg/PoseStamped
/Arm1/dobot_msgs_v4/msg/ToolVectorActual
/Arm1/dobot_bringup_ros2/srv/ServoP
```

When `command_type:=Pose`, the command topic type becomes
`geometry_msgs/msg/Pose`. Since an unstamped `Pose` has no frame, the bridge
uses the `command_frame` parameter, which defaults to `base_link`.

## Send a command

This example commands an identity orientation at `(0.30, -0.10, 0.40)` metres
in `base_link`:

```bash
ros2 topic pub --rate 20 /Arm1/cartesian_pose_command \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {
    position: {x: 0.30, y: -0.10, z: 0.40},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

For a command in another frame, set `header.frame_id`. The bridge looks up a TF
transform into `base_frame` (default `base_link`) before calling `ServoP`.

## Read Cartesian state

```bash
ros2 topic echo /Arm1/cartesian_pose_state
```

The published pose uses metres and a quaternion. The bridge converts the
Dobot feedback convention (position in millimetres and Euler `rx, ry, rz` in
degrees) into the ROS convention. The `ToolVectorActual` message has no header,
so the bridge timestamps feedback when it receives it and labels it with
`base_frame`.

The ROS frame transform does not change the Dobot controller's active user/tool
coordinate selection. Configure the controller's user/tool coordinates so the
`ServoP` values have the intended meaning.

## Parameters

The executable accepts these parameters:

| Parameter | Default | Purpose |
|---|---|---|
| `command_topic` | `cartesian_pose_command` | Pose command topic |
| `command_type` | `PoseStamped` | `Pose` or `PoseStamped` |
| `command_frame` | `base_link` | Frame used for unstamped `Pose` |
| `state_topic` | `cartesian_pose_state` | Published `PoseStamped` topic |
| `feedback_topic` | `dobot_msgs_v4/msg/ToolVectorActual` | Driver feedback topic |
| `servo_p_service` | `dobot_bringup_ros2/srv/ServoP` | Driver Cartesian service |
| `base_frame` | `base_link` | Target command/state frame |
| `transform_timeout_sec` | `0.1` | TF lookup timeout |

The Dobot-side conversions can be changed with `dobot_position_unit` (`mm` or
`m`) and `dobot_orientation_unit` (`deg` or `rad`) if the connected controller
configuration differs. ROS command positions and published state positions
remain metres, and ROS orientations remain quaternions.

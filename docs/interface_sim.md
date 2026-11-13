# Interface report (tiago-pro-30-gazebo)

Generated 2026-10-16T22:44:21 by scripts/probe_interface.py with config `/home/user/tiago_ws/config/sim.yaml`, ROS_DOMAIN_ID=42.

## Config check

| stream | topic | type | rate (Hz) | detail |
|---|---|---|---|---|
| joint states | `/joint_states` | sensor_msgs/msg/JointState | 49.6 |  |
| head_camera image | `/head_front_camera/color/image_raw` | sensor_msgs/msg/Image | 29.8 | encoding bgr8, frame `head_front_camera_depth_optical_frame` |
| head_camera camera_info | `/head_front_camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 29.8 | 640x480, frame `head_front_camera_depth_optical_frame`, fx 348.0 fy 348.0 |
| left_camera image | `/gripper_left_camera/color/image_raw` | sensor_msgs/msg/Image | 29.6 | encoding rgb8, frame `gripper_left_camera_color_optical_frame` |
| left_camera camera_info | `/gripper_left_camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 29.8 | 640x480, frame `gripper_left_camera_color_optical_frame`, fx 379.9 fy 379.9 |
| right_camera image | `/gripper_right_camera/color/image_raw` | sensor_msgs/msg/Image | 29.6 | encoding rgb8, frame `gripper_right_camera_color_optical_frame` |
| right_camera camera_info | `/gripper_right_camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 29.6 | 640x480, frame `gripper_right_camera_color_optical_frame`, fx 379.9 fy 379.9 |

### Joints used by the config

- left: `arm_left_1_joint` ok
- left: `arm_left_2_joint` ok
- left: `arm_left_3_joint` ok
- left: `arm_left_4_joint` ok
- left: `arm_left_5_joint` ok
- left: `arm_left_6_joint` ok
- left: `arm_left_7_joint` ok
- left: `gripper_left_finger_joint` ok
- right: `arm_right_1_joint` ok
- right: `arm_right_2_joint` ok
- right: `arm_right_3_joint` ok
- right: `arm_right_4_joint` ok
- right: `arm_right_5_joint` ok
- right: `arm_right_6_joint` ok
- right: `arm_right_7_joint` ok
- right: `gripper_right_finger_joint` ok

### Controller topics

- `/arm_left_controller/joint_trajectory`: trajectory_msgs/msg/JointTrajectory
- `/arm_right_controller/joint_trajectory`: trajectory_msgs/msg/JointTrajectory
- `/gripper_left_controller/joint_trajectory`: trajectory_msgs/msg/JointTrajectory
- `/gripper_right_controller/joint_trajectory`: trajectory_msgs/msg/JointTrajectory
- `/torso_controller/joint_trajectory`: trajectory_msgs/msg/JointTrajectory
- `/head_controller/joint_trajectory`: trajectory_msgs/msg/JointTrajectory

### TF frames

- `base_footprint` -> `gripper_left_grasping_link`: (0.501, 0.221, 0.999) m
- `base_footprint` -> `gripper_right_grasping_link`: (0.502, -0.221, 0.999) m
- `base_footprint` -> `head_front_camera_depth_optical_frame`: (0.150, 0.018, 1.207) m
- `base_footprint` -> `gripper_left_camera_color_optical_frame`: (0.496, 0.222, 1.147) m
- `base_footprint` -> `gripper_right_camera_color_optical_frame`: (0.497, -0.222, 1.147) m

All joint names (from /joint_states):

`arm_left_1_joint`, `arm_left_2_joint`, `arm_left_3_joint`, `arm_left_4_joint`, `arm_left_5_joint`, `arm_left_6_joint`, `arm_left_7_joint`, `arm_right_1_joint`, `arm_right_2_joint`, `arm_right_3_joint`, `arm_right_4_joint`, `arm_right_5_joint`, `arm_right_6_joint`, `arm_right_7_joint`, `gripper_left_finger_joint`, `gripper_right_finger_joint`, `head_1_joint`, `head_2_joint`, `torso_lift_joint`, `wheel_front_left_joint`, `wheel_front_right_joint`, `wheel_rear_left_joint`, `wheel_rear_right_joint`

## Controllers

```
head_controller                           joint_trajectory_controller/JointTrajectoryController  active  
arm_right_controller                      joint_trajectory_controller/JointTrajectoryController  active  
arm_left_gravity_compensation_controller  pal_controllers/GravityCompensationController          inactive
joint_state_broadcaster                   joint_state_broadcaster/JointStateBroadcaster          active  
arm_left_controller                       joint_trajectory_controller/JointTrajectoryController  active  
gripper_left_controller                   joint_trajectory_controller/JointTrajectoryController  active  
arm_right_gravity_compensation_controller pal_controllers/GravityCompensationController          inactive
torso_controller                          joint_trajectory_controller/JointTrajectoryController  active  
gripper_right_controller                  joint_trajectory_controller/JointTrajectoryController  active
```

## Hardware interfaces

```
command interfaces
	arm_left_1_joint/effort [available] [unclaimed]
	arm_left_1_joint/position [available] [claimed]
	arm_left_1_joint/velocity [available] [unclaimed]
	arm_left_2_joint/effort [available] [unclaimed]
	arm_left_2_joint/position [available] [claimed]
	arm_left_2_joint/velocity [available] [unclaimed]
	arm_left_3_joint/effort [available] [unclaimed]
	arm_left_3_joint/position [available] [claimed]
	arm_left_3_joint/velocity [available] [unclaimed]
	arm_left_4_joint/effort [available] [unclaimed]
	arm_left_4_joint/position [available] [claimed]
	arm_left_4_joint/velocity [available] [unclaimed]
	arm_left_5_joint/effort [available] [unclaimed]
	arm_left_5_joint/position [available] [claimed]
	arm_left_5_joint/velocity [available] [unclaimed]
	arm_left_6_joint/effort [available] [unclaimed]
	arm_left_6_joint/position [available] [claimed]
	arm_left_6_joint/velocity [available] [unclaimed]
	arm_left_7_joint/effort [available] [unclaimed]
	arm_left_7_joint/position [available] [claimed]
	arm_left_7_joint/velocity [available] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_1_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_1_joint/velocity [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_2_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_2_joint/velocity [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_3_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_3_joint/velocity [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_4_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_4_joint/velocity [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_5_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_5_joint/velocity [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_6_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_6_joint/velocity [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_7_joint/position [unavailable] [unclaimed]
	arm_left_gravity_compensation_controller/arm_left_7_joint/velocity [unavailable] [unclaimed]
	arm_right_1_joint/effort [available] [unclaimed]
	arm_right_1_joint/position [available] [claimed]
	arm_right_1_joint/velocity [available] [unclaimed]
	arm_right_2_joint/effort [available] [unclaimed]
	arm_right_2_joint/position [available] [claimed]
	arm_right_2_joint/velocity [available] [unclaimed]
	arm_right_3_joint/effort [available] [unclaimed]
	arm_right_3_joint/position [available] [claimed]
	arm_right_3_joint/velocity [available] [unclaimed]
	arm_right_4_joint/effort [available] [unclaimed]
	arm_right_4_joint/position [available] [claimed]
	arm_right_4_joint/velocity [available] [unclaimed]
	arm_right_5_joint/effort [available] [unclaimed]
	arm_right_5_joint/position [available] [claimed]
	arm_right_5_joint/velocity [available] [unclaimed]
	arm_right_6_joint/effort [available] [unclaimed]
	arm_right_6_joint/position [available] [claimed]
	arm_right_6_joint/velocity [available] [unclaimed]
	arm_right_7_joint/effort [available] [unclaimed]
	arm_right_7_joint/position [available] [claimed]
	arm_right_7_joint/velocity [available] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_1_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_1_joint/velocity [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_2_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_2_joint/velocity [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_3_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_3_joint/velocity [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_4_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_4_joint/velocity [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_5_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_5_joint/velocity [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_6_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_6_joint/velocity [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_7_joint/position [unavailable] [unclaimed]
	arm_right_gravity_compensation_controller/arm_right_7_joint/velocity [unavailable] [unclaimed]
	gripper_left_finger_joint/position [available] [claimed]
	gripper_right_finger_joint/position [available] [claimed]
	head_1_joint/position [available] [claimed]
	head_2_joint/position [available] [claimed]
	torso_lift_joint/position [available] [claimed]
	torso_lift_joint/velocity [available] [unclaimed]
	wheel_front_left_joint/velocity [available] [unclaimed]
	wheel_front_right_joint/velocity [available] [unclaimed]
	wheel_rear_left_joint/velocity [available] [unclaimed]
	wheel_rear_right_joint/velocity [available] [unclaimed]
state interfaces
	arm_left_1_joint/effort
	arm_left_1_joint/position
	arm_left_1_joint/velocity
	arm_left_2_joint/effort
	arm_left_2_joint/position
	arm_left_2_joint/velocity
	arm_left_3_joint/effort
	arm_left_3_joint/position
	arm_left_3_joint/velocity
	arm_left_4_joint/effort
	arm_left_4_joint/position
	arm_left_4_joint/velocity
	arm_left_5_joint/effort
	arm_left_5_joint/position
	arm_left_5_joint/velocity
	arm_left_6_joint/effort
	arm_left_6_joint/position
	arm_left_6_joint/velocity
	arm_left_7_joint/effort
	arm_left_7_joint/position
	arm_left_7_joint/velocity
	arm_right_1_joint/effort
	arm_right_1_joint/position
	arm_right_1_joint/velocity
	arm_right_2_joint/effort
	arm_right_2_joint/position
	arm_right_2_joint/velocity
	arm_right_3_joint/effort
	arm_right_3_joint/position
	arm_right_3_joint/velocity
	arm_right_4_joint/effort
	arm_right_4_joint/position
	arm_right_4_joint/velocity
	arm_right_5_joint/effort
	arm_right_5_joint/position
	arm_right_5_joint/velocity
	arm_right_6_joint/effort
	arm_right_6_joint/position
	arm_right_6_joint/velocity
	arm_right_7_joint/effort
	arm_right_7_joint/position
	arm_right_7_joint/velocity
	gripper_left_finger_joint/effort
	gripper_left_finger_joint/position
	gripper_left_finger_joint/velocity
	gripper_right_finger_joint/effort
	gripper_right_finger_joint/position
	gripper_right_finger_joint/velocity
	head_1_joint/effort
	head_1_joint/position
	head_1_joint/velocity
	head_2_joint/effort
	head_2_joint/position
	head_2_joint/velocity
	torso_lift_joint/effort
	torso_lift_joint/position
	torso_lift_joint/velocity
	wheel_front_left_joint/position
	wheel_front_right_joint/position
	wheel_rear_left_joint/position
	wheel_rear_right_joint/position
```

## Nodes

```
/arm_left_controller
/arm_left_gravity_compensation_controller
/arm_right_controller
/arm_right_gravity_compensation_controller
/base_front_laser
/base_imu
/base_rear_laser
/controller_manager
/foxglove_bridge
/gazebo
/gazebo_ros2_control
/gazebo_ros_planar_move
/gripper_left_camera/gripper_left_camera_color_plugin
/gripper_left_controller
/gripper_left_grasper_srv
/gripper_right_camera/gripper_right_camera_color_plugin
/gripper_right_controller
/gripper_right_grasper_srv
/head_controller
/head_front_camera
/head_video_stream
/joint_state_broadcaster
/joystick_relay
/move_group
/move_group_private_104579724270256
/moveit_simple_controller_manager
/play_motion2_executor
/play_motion2_mgr
/relay_field_7769
/robot_state_publisher
/ros2_control_gazebo_system
/tiago_dc_probe
/tiago_dc_quest_bridge
/tiago_dc_recorder
/tiago_dc_scene
/tiago_dc_teleop
/torso_controller
/transform_listener_impl_5f1d5dfbbc40
/transform_listener_impl_5f1d63130220
/twist_marker
/twist_mux
/twist_mux/add_analyzer_node
```

## Topics

```
/arm_left_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/arm_left_controller/follow_joint_trajectory/_action/feedback [control_msgs/action/FollowJointTrajectory_FeedbackMessage]
/arm_left_controller/follow_joint_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/arm_left_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/arm_left_controller/speed_scaling_input [control_msgs/msg/SpeedScalingFactor]
/arm_left_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/arm_left_gravity_compensation_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/arm_right_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/arm_right_controller/follow_joint_trajectory/_action/feedback [control_msgs/action/FollowJointTrajectory_FeedbackMessage]
/arm_right_controller/follow_joint_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/arm_right_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/arm_right_controller/speed_scaling_input [control_msgs/msg/SpeedScalingFactor]
/arm_right_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/arm_right_gravity_compensation_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/assisted_teleop_priority [std_msgs/msg/Bool]
/assisted_vel [geometry_msgs/msg/Twist]
/attached_collision_object [moveit_msgs/msg/AttachedCollisionObject]
/base_imu [sensor_msgs/msg/Imu]
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/collision_object [moveit_msgs/msg/CollisionObject]
/controller_manager/activity [controller_manager_msgs/msg/ControllerManagerActivity]
/controller_manager/introspection_data/full [pal_statistics_msgs/msg/Statistics]
/controller_manager/introspection_data/names [pal_statistics_msgs/msg/StatisticsNames]
/controller_manager/introspection_data/values [pal_statistics_msgs/msg/StatisticsValues]
/controller_manager/statistics/full [pal_statistics_msgs/msg/Statistics]
/controller_manager/statistics/names [pal_statistics_msgs/msg/StatisticsNames]
/controller_manager/statistics/values [pal_statistics_msgs/msg/StatisticsValues]
/diagnostics [diagnostic_msgs/msg/DiagnosticArray]
/display_contacts [visualization_msgs/msg/MarkerArray]
/display_planned_path [moveit_msgs/msg/DisplayTrajectory]
/docking_vel [geometry_msgs/msg/Twist]
/dynamic_joint_states [control_msgs/msg/DynamicJointState]
/execute_task_solution/_action/feedback [moveit_task_constructor_msgs/action/ExecuteTaskSolution_FeedbackMessage]
/execute_task_solution/_action/status [action_msgs/msg/GoalStatusArray]
/execute_trajectory/_action/feedback [moveit_msgs/action/ExecuteTrajectory_FeedbackMessage]
/execute_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/filtered_cloud [sensor_msgs/msg/PointCloud2]
/gripper_left_camera/color/camera_info [sensor_msgs/msg/CameraInfo]
/gripper_left_camera/color/image_raw [sensor_msgs/msg/Image]
/gripper_left_camera/color/image_raw/compressed [sensor_msgs/msg/CompressedImage]
/gripper_left_camera/color/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
/gripper_left_camera/color/image_raw/theora [theora_image_transport/msg/Packet]
/gripper_left_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/gripper_left_controller/follow_joint_trajectory/_action/feedback [control_msgs/action/FollowJointTrajectory_FeedbackMessage]
/gripper_left_controller/follow_joint_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/gripper_left_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/gripper_left_controller/speed_scaling_input [control_msgs/msg/SpeedScalingFactor]
/gripper_left_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/gripper_left_grasper_srv/is_grasped [std_msgs/msg/Bool]
/gripper_right_camera/color/camera_info [sensor_msgs/msg/CameraInfo]
/gripper_right_camera/color/image_raw [sensor_msgs/msg/Image]
/gripper_right_camera/color/image_raw/compressed [sensor_msgs/msg/CompressedImage]
/gripper_right_camera/color/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
/gripper_right_camera/color/image_raw/theora [theora_image_transport/msg/Packet]
/gripper_right_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/gripper_right_controller/follow_joint_trajectory/_action/feedback [control_msgs/action/FollowJointTrajectory_FeedbackMessage]
/gripper_right_controller/follow_joint_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/gripper_right_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/gripper_right_controller/speed_scaling_input [control_msgs/msg/SpeedScalingFactor]
/gripper_right_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/gripper_right_grasper_srv/is_grasped [std_msgs/msg/Bool]
/head_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/head_controller/follow_joint_trajectory/_action/feedback [control_msgs/action/FollowJointTrajectory_FeedbackMessage]
/head_controller/follow_joint_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/head_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/head_controller/speed_scaling_input [control_msgs/msg/SpeedScalingFactor]
/head_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/head_front_camera/color/camera_info [sensor_msgs/msg/CameraInfo]
/head_front_camera/color/image_raw [sensor_msgs/msg/Image]
/head_front_camera/color/image_raw/compressed [sensor_msgs/msg/CompressedImage]
/head_front_camera/color/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
/head_front_camera/color/image_raw/theora [theora_image_transport/msg/Packet]
/head_front_camera/depth/camera_info [sensor_msgs/msg/CameraInfo]
/head_front_camera/depth/color/points [sensor_msgs/msg/PointCloud2]
/head_front_camera/depth/image_rect_raw [sensor_msgs/msg/Image]
/head_front_camera/depth/image_rect_raw/compressed [sensor_msgs/msg/CompressedImage]
/head_front_camera/depth/image_rect_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
/head_front_camera/depth/image_rect_raw/theora [theora_image_transport/msg/Packet]
/input_joy/cmd_vel [geometry_msgs/msg/Twist]
/joint_state_broadcaster/transition_event [lifecycle_msgs/msg/TransitionEvent]
/joint_states [sensor_msgs/msg/JointState]
/joy_priority [std_msgs/msg/Bool]
/joy_priority_action/_action/feedback [twist_mux_msgs/action/JoyPriority_FeedbackMessage]
/joy_priority_action/_action/status [action_msgs/msg/GoalStatusArray]
/joy_turbo_angular_decrease/_action/feedback [twist_mux_msgs/action/JoyTurbo_FeedbackMessage]
/joy_turbo_angular_decrease/_action/status [action_msgs/msg/GoalStatusArray]
/joy_turbo_angular_increase/_action/feedback [twist_mux_msgs/action/JoyTurbo_FeedbackMessage]
/joy_turbo_angular_increase/_action/status [action_msgs/msg/GoalStatusArray]
/joy_turbo_decrease/_action/feedback [twist_mux_msgs/action/JoyTurbo_FeedbackMessage]
/joy_turbo_decrease/_action/status [action_msgs/msg/GoalStatusArray]
/joy_turbo_increase/_action/feedback [twist_mux_msgs/action/JoyTurbo_FeedbackMessage]
/joy_turbo_increase/_action/status [action_msgs/msg/GoalStatusArray]
/joy_turbo_reset/_action/feedback [twist_mux_msgs/action/JoyTurbo_FeedbackMessage]
/joy_turbo_reset/_action/status [action_msgs/msg/GoalStatusArray]
/joy_vel [geometry_msgs/msg/Twist]
/key_vel [geometry_msgs/msg/Twist]
/marker [visualization_msgs/msg/Marker]
/mobile_base_controller/cmd_vel [geometry_msgs/msg/TwistStamped]
/mobile_base_controller/cmd_vel_unstamped [geometry_msgs/msg/Twist]
/mobile_base_controller/odom [nav_msgs/msg/Odometry]
/monitored_planning_scene [moveit_msgs/msg/PlanningScene]
/motion_plan_request [moveit_msgs/msg/MotionPlanRequest]
/move_action/_action/feedback [moveit_msgs/action/MoveGroup_FeedbackMessage]
/move_action/_action/status [action_msgs/msg/GoalStatusArray]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/performance_metrics [gazebo_msgs/msg/PerformanceMetrics]
/planning_scene [moveit_msgs/msg/PlanningScene]
/planning_scene_world [moveit_msgs/msg/PlanningSceneWorld]
/play_motion2/_action/feedback [play_motion2_msgs/action/PlayMotion2_FeedbackMessage]
/play_motion2/_action/status [action_msgs/msg/GoalStatusArray]
/play_motion2/raw/_action/feedback [play_motion2_msgs/action/PlayMotion2Raw_FeedbackMessage]
/play_motion2/raw/_action/status [action_msgs/msg/GoalStatusArray]
/play_motion2_executor/transition_event [lifecycle_msgs/msg/TransitionEvent]
/play_motion2_mgr/transition_event [lifecycle_msgs/msg/TransitionEvent]
/power/is_charging [std_msgs/msg/Bool]
/quest/joy [sensor_msgs/msg/Joy]
/quest/left/pose [geometry_msgs/msg/PoseStamped]
/quest/right/pose [geometry_msgs/msg/PoseStamped]
/recorder/status [std_msgs/msg/String]
/robot_description [std_msgs/msg/String]
/robot_description_semantic [std_msgs/msg/String]
/robot_face/image_raw [sensor_msgs/msg/Image]
/rosout [rcl_interfaces/msg/Log]
/rviz_joy_vel [geometry_msgs/msg/Twist]
/scan_front_raw [sensor_msgs/msg/LaserScan]
/scan_rear_raw [sensor_msgs/msg/LaserScan]
/tab_vel [geometry_msgs/msg/Twist]
/teleop/command [sensor_msgs/msg/JointState]
/teleop/event [std_msgs/msg/String]
/teleop/left/tcp_command [geometry_msgs/msg/PoseStamped]
/teleop/right/tcp_command [geometry_msgs/msg/PoseStamped]
/teleop/status [std_msgs/msg/String]
/text_marker [visualization_msgs/msg/Marker]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/throttle_filtering_points/filtered_points [sensor_msgs/msg/PointCloud2]
/torso_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/torso_controller/follow_joint_trajectory/_action/feedback [control_msgs/action/FollowJointTrajectory_FeedbackMessage]
/torso_controller/follow_joint_trajectory/_action/status [action_msgs/msg/GoalStatusArray]
/torso_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/torso_controller/speed_scaling_input [control_msgs/msg/SpeedScalingFactor]
/torso_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/trajectory_execution_event [std_msgs/msg/String]
```

# rostack-chan
stack-chan hardware using ros2_control


```bash
ros2 launch rostackchan_description rostackchan.launch.py
```

```bash
ros2 control switch_controllers --activate joint_state_broadcaster --activate joint_trajectory_controller --deactivate velocity_controller
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1, joint2],
    points: [
      { positions: [1.5, 1.5], time_from_start: { sec: 2 } },
      { positions: [2.0, 0.0], time_from_start: { sec: 4 } },
      { positions: [1.5, 1.5], time_from_start: { sec: 6 } }
    ]
  }
}"
```
# joystick_gui 실행 명령

```bash
cd /home/amap/FITO/FITO_AMR_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run joystick_gui gui_node
```

또는 launch:

```bash
cd /home/amap/FITO/FITO_AMR_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch joystick_gui gui.launch.py
```

GUI 시작 시 [gui_node.py](../joystick_gui/gui_node.py) auto-start 가 `bridge → diff_drive → odom → wheel → motor_health → teleop` 순으로 자동 기동.

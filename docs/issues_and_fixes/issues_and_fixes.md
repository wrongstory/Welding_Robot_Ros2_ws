# Issues and Fixes

## 2026-07-05

### [Fix] Livox Mid-360 driver "bind failed" — wrong host/LiDAR IP in MID360_config.json

- **문제**: `livox_ros_driver2` startup logged `bind failed` → `Init lds lidar fail!`; no point cloud published on `/livox/lidar`.
- **원인**: config host/LiDAR IPs did not match the real network — `MID360_config.json:14,16,18,20` had `host_net_info.*` = `192.168.1.5` (not present on any local interface), and `MID360_config.json:28` had `lidar_configs[0].ip` = `192.168.1.152` (a different unit). Evidence: `nmcli` profile `lidar-mid360` sets `enp1s0` = `192.168.1.50/24`; LiDAR serial ends `...833` → IP `192.168.1.133` (confirmed REACHABLE via ARP/ping).
- **해결**: set the four `host_net_info` IPs `192.168.1.5` → `192.168.1.50` and `lidar_configs[0].ip` `192.168.1.152` → `192.168.1.133` (5 lines changed).
- **파일**: `Welding_Robot_Ros2_ws/src/Sensor/LIdar/3D/livox_mid360/livox_ros_driver2/config/MID360_config.json`
- **상태**: 완료 (rebuilt `livox_ros_driver2`; live-verified `Init lds lidar success!` + `/livox/lidar` PointCloud2 streaming ~10 Hz into RViz).

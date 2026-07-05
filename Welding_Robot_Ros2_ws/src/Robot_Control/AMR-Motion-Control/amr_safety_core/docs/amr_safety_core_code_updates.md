# amr_safety_core — Code Updates

## 2026-05-29 / 23:19 / (미커밋) / 수정 pose 유효성 판정을 TF lookup → /robot_pose 구독 캐시로 전환

### 수정 (`include/amr_safety_core/safety_subscriber.hpp`)
- `pose_valid()` / `last_pose()` 의 TF map→base_link `lookupTransform` 제거
- `/robot_pose`(PoseStamped, `rclcpp::QoS(KeepLast(10)).reliable()`) 구독 멤버 추가
- 수신 시각 staleness(`pose_timeout`) 비교로 유효성 판정, 캐시된 pose 반환
- config 파라미터에 `robot_pose_topic="/robot_pose"` 추가 (`map_frame`/`base_frame`은 deprecated 주석 처리)
- `tf2_ros` buffer/listener 멤버 및 관련 include 제거

### 배경
동일 토픽-다른 타입 충돌로 amr_motion_control_2wd 크래시 → /robot_pose(PoseStamped) SSOT 통일 및 QoS 정합

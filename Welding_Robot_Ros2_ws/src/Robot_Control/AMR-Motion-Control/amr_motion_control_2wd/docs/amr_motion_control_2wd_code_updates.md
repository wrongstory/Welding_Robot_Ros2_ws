# amr_motion_control_2wd — Code Updates

## 2026-06-09 / (미커밋) / 수정 motion bringup 에 cmd_vel_safety_guard 연결 (cmd_vel 체인 복구)

### 수정 (`launch/motion_control.launch.py`)
- 모션 노드(`amr_motion_control_2wd`, `cmd_vel→cmd_vel_raw` remap) 뒤에 신규 `cmd_vel_safety_guard` Node 추가.
- 끊겨 있던 체인 복구: `motion → /cmd_vel_raw → cmd_vel_safety_guard → /cmd_vel → diff_drive_controller`.
  기존엔 `/cmd_vel_raw` 소비자가 없어 드라이브트레인(`/cmd_vel` 구독)이 명령을 못 받던 상태(B3).
- 게이트 param 명시: `input_topic=/cmd_vel_raw`, `output_topic=/cmd_vel`, `guarded_topic=/cmd_vel_guarded`(ACS Diagnostics 미러), `watchdog_timeout_s=0.3`.

### 수정 (`package.xml`)
- `<exec_depend>cmd_vel_safety_guard</exec_depend>` 추가 — launch 가 해당 노드를 기동하는 런타임 의존 + colcon 빌드 순서 보장.

### 배경 / 검증
- 게이트 노드 본체·안전정책·QoS 매칭 상세는 `cmd_vel_safety_guard/docs/cmd_vel_safety_guard_code_updates.md` 참조(같은 작업 단위).
- `python3 -m py_compile launch/motion_control.launch.py` 통과. ⚠ `colcon build` 미실행(공유 워크스페이스 보호) — 중앙 빌드는 lead 가 수행.

## 2026-06-09 / (미커밋) / 수정 translate·translate_reverse 에 도착 2단 감속 캡 이식 (T-AMR_ros2_ws)

### 출처
[T-AMR_ros2_ws](https://github.com/kuks2309/T-AMR_ros2_ws) `src/Control/AMR-Motion-Control/amr_motion_control_2wd` 의 translate/translate_reverse 에 적용된 「도착 2단 감속 캡」을 본 워크스페이스로 이식(사용자 지시 2026-06-09). 로컬 `reference/Control/AMR-Motion-Control`(2026-06-02 drop)에는 미포함 → GitHub 최신본(`gh api`, private repo)에서 수확.

### 수정 (`src/translate_action_server.cpp`·`.hpp`, `src/translate_reverse_action_server.cpp`·`.hpp`, `config/motion_params.yaml`)
- **도착 2단 감속 캡** — min_vx clamp 직후, safety limit 직전에 `vx_profile` 상한을 추가(`projection >= 0` 구간만):
  - 1단(원거리): `v_cap = approach_gain_·remaining` (거리비례 → 순항속도 따라 감속거리 자동 스케일)
  - 2단(근거리, `remaining < stop_speed_²/(2·stop_decel_)`): `v_cap = sqrt(2·stop_decel_·remaining)` (완만 정속감속, 도착점 0 수렴)
- **도착 시 ramp-to-0** — 도착(arrival, exit_speed=0) 분기에서 즉시 `cmd_vel=0` 컷 대신 `stop_decel_`(완만)로 `prev_cmd_vx` 를 0 까지 감속 발행(완전정지 jerk 저감). translate_reverse 는 절대속도 `prev_cmd_vx` 에 후진 부호 적용해 발행.
- **신규 파라미터 3종 × 2서버** — `translate_approach_gain`/`translate_stop_speed`/`translate_stop_decel`(+ `translate_reverse_*`), 기본 `1.5 / 0.2 / 0.1`. hpp 멤버 + 생성자 `safeParam` + `motion_params.yaml` 노출.

### 이식 범위 (의도적 한정)
- **2단 감속 캡 + ramp-to-0 만** 이식. T-AMR 의 인접 변경인 ① 감속(종료) 구간 PID(Option A) ② 감속률 클램프 ③ `profile_offset`(출발점 뒤 처리, behind_start 대체)는 **미이식** — 사용자 요청이 "2단 감속"으로 한정, 별개 기능이며 본 워크스페이스의 IMU-fused heading·behind_start 와 독립.
- 본 변경은 **vx(선속도) 경로만** 손댐 → 직전 IMU-fused heading 변경(omega 경로)과 직교, 충돌 없음. `projection<0`(behind_start) 구간은 캡 미적용(기존 동작 유지).

### 검증 / 한계
- `colcon build --packages-select amr_motion_control_2wd --symlink-install` 성공 (1min 21s, exit 0).
- ⚠ **런타임/HIL 미검증** — `approach_gain`/`stop_speed`/`stop_decel` 기본값은 T-AMR rosbag 기준값이라 FITO 허브모터(ZLAC706, 정지마찰 큼)에서 재튜닝 필요할 수 있음(실주행 사용자 입회).
- ⚠ ramp-to-0 루프(약 `prev_cmd_vx/stop_decel` 초, 기본 ≲1s)는 내부 `rate.sleep()` 블로킹이며 cancel/safety 미검사(T-AMR 원본 동작 그대로). 0 으로 감속 중이라 위험도 낮음.

## 2026-06-08 / 733cbdc / 수정 translate·translate_reverse·yaw_control 센서 I/O 를 프로젝트 적합화 (IMU-fused heading + 이중 pose 구독 제거 + safety 통일)

### 수정 (`include/amr_motion_control_2wd/motion_common.hpp`)
- `imuYawOffset(pose_yaw, imu_yaw)` / `fuseImuYawToMap(imu_yaw, offset)` 헬퍼 추가 (직진/heading 프리미티브 공용)

### 수정 (`src/translate_action_server.cpp`·`.hpp`, `src/translate_reverse_action_server.cpp`·`.hpp`, `src/yaw_control_action_server.cpp`·`.hpp`)
- **heading 피드백 소스 전환** — heading 오차를 느린 pose-yaw(`/robot_pose`, 본 프로젝트 rtabmap ~1Hz) → **시작 시 1회 오프셋 융합한 고속 IMU yaw**(50~200Hz)로 변경. 위치(projection)·cross-track error 는 `/robot_pose`(map-frame) 그대로 유지.
  - `imu_yaw_offset = imuYawOffset(pose_yaw0, imu_yaw0)` 시작 1회 등록 → 매 주기 `fuseImuYawToMap(imu_yaw, offset)` 로 map-frame heading 환산. translate_reverse 는 reverse 시 +π 그대로 적용.
- **이중 `/robot_pose` 구독 제거** — 별도 `pose_sub_`(→ `robotPoseCallback` → `watchdog_->updatePose`)를 제거하고 watchdog 갱신을 execute 루프의 `lookupRobotPose`(PoseCache) 단일 경로로 통일. 중복이던 `watchdog_->poseReceived()` 사전검사도 제거(`lookupRobotPose` 가 수신/신선도 판정).
- **translate_reverse safety 통일** — raw `/safety/speed_limit`·`safety_status` 구독 + 매직넘버 `status==2` → `SafetySubscriber` 추상(`speed_limit()`/`is_dangerous()`)으로 교체(translate/spin/yaw_control 과 동일). 미사용 include(`std_msgs/Float64`, `tc_msgs/SafetyStatus`) 정리.
- stale 주석 `tf2 map→base_link (~34Hz)` → `/robot_pose` 정정, 초기화 로그에 `heading=imu/data fused` 명시.
- **IMU 의존성이 비로소 실사용** — 기존에는 세 서버 모두 IMU 를 구독·`imu_received_` abort 게이트로만 쓰고 `last_yaw_rad_` 를 제어에 미사용(죽은 의존성)이었음 → heading 융합에 사용.

### 배경
- spin/turn 은 고속 IMU yaw 피드백을 쓰는데 translate/translate_reverse/yaw_control 은 느린 pose-yaw 를 썼고 IMU 구독은 abort 게이트로만 살아있는 비대칭. reference 표준(`trnav_motion_dd/dd_translate_action_server.cpp`)도 pose-yaw + `imuReceived()` 게이트(= `DdActionServerBase` 유산)라 동일 한계.
- 본 프로젝트 `/robot_pose` 가 rtabmap ~1Hz 라 50Hz 제어 루프에서 heading 추종이 부정확(과거 `translate_heading_threshold_deg` 45→85° 상향이 "주행 중 heading 보정"에 의존). spin 이 IMU 를 도입한 것과 동일 사유.
- 설계 근거: 시작 오프셋 seed-once 로 충분 — 느린 IMU yaw 드리프트는 **위치기반 Stanley CTE 항**(`/robot_pose`)이 흡수해 직선으로 되돌림(빠른 외란=IMU heading, 느린 드리프트=pose CTE 로 역할 분담).
- **사용자 결정(2026-06-08)**: heading 소스 = IMU-fused(권장안), 작업 범위 = **구조 정리만 먼저**(게인 YAML 노출 + SIL/HIL 튜닝은 후속). `docs/user_instructions/user_instructions.md` 2026-06-08 07:27 entry 참조.

### 검증 / 한계
- `colcon build --packages-select amr_motion_control_2wd --symlink-install` 성공 (2min 3s, exit 0). 컴파일 에러 0.
- cmd_vel 출력(HAL 경계)·위치 제어·trapezoidal profile·watchdog 동작 불변. `w1/w2_drive_rpm` feedback 은 `amr_interfaces` .action 필드라 0 유지(인터페이스 변경 후순위).
- ⚠ **런타임/HIL 미검증** — heading 신호가 1Hz staircase → 50Hz 연속으로 바뀌면서 `Kd_heading` 미분항 동특성이 크게 달라짐 → 실주행 전 `Kp_heading`/`Kd_heading` 튜닝 필요(후속, 사용자 입회). pure_pursuit 은 본 변경 범위 외(별도 lookahead 제어, 추후 검토).

## 2026-05-30 / 12:14 / (미커밋) / 수정 Spin 180° 무한회전 버그: progress를 방향 인식 [0,360) lift로 정의

### 수정 (`src/spin_action_server.cpp`)
- `measureProgress()` 람다의 진행량 정의를 단순 음수클램프 → **명령 방향(sign) 인식 [0,360) lift**로 변경
  ```cpp
  double delta = normalizeAngle(cur - start_imu) * 180/π;   // [-180,180]
  double prog  = sign * delta;
  if (prog < 0)  prog += 360.0;                              // 방향 인식 lift → [0,360)
  if (prog > (target_abs + 360)/2) prog = 0.0;              // 반대편 호(시작부근 후진/노이즈) → 0
  ```
- 람다 캡처에 `target_abs` 추가. 구조(단일 trapezoidal + fine-correction)·다른 로직 불변.

### 배경 (rosbag `0528_speed_1.5_test_20260530_110709` 확증)
- 기존 `progress = sign*normalizeAngle(cur−start_imu)`, `(d<0)?0:d` 클램프
- target=180°(`rotation_needed=±180`)에서 180° 근처 normalizeAngle이 경계에 위치 → 미세 오버슈트로 부호 반전(+179.8→−175.2) → 클램프 0 → **progress 리셋 → 무한회전**
- 실제: target=180 spin이 89초간 미완료, ~343° 회전 후 사용자 취소. cmd_vel 전부 −z·평균 3.9dps. (0°/5.7° spin은 정상)
- **단일 wrap 차이의 ±180 모호성은 "방향을 모를 때만" 성립.** spin coarse는 단일 방향(sign 고정)이므로 음수일 때 +360 lift로 180을 단조 통과 가능 → 적분/leg분해 불필요.

### 결정 경위
- 5인 팀 워크플로 + Codex 자문은 leg 분해를 추천했으나(중간 정지·시간증가 수반), 사용자 지적("방향을 아니 error 정의만 정확히")이 더 단순·우월 → 방향 인식 lift 채택. (적분 절대 금지 제약 준수: 메모리 없는 단일 샘플)

### 검증
- 시뮬레이션: target 180(start −5.2 / 정확히 0), 179°, 90° 모두 progress 단조 증가·완료; 시작 후진 지터 시 NEW=0(오완료 없음)
- `colcon build --packages-select amr_motion_control_2wd` 성공 (15.4s), 풀노드 SIL 7/7 PASS
- **실로봇 검증 완료 (2026-05-30)**: rosbag `0528_speed_1.5_test_20260530_125451`·`_130435` — target=180° spin 이 ~7.8~7.9s 에 `Spin complete`(error ≤0.29°), 무한회전·cancel·abort 0건. 174.6°/소각도 spin 도 정상. (수정 전 110709: 89s 무한회전 → 해소 확인)

## 2026-05-30 / 10:?? / (미커밋) / 수정 시작 시 이미 도착 상태면 PARAM_ERR 대신 성공 처리

### 수정 (`src/translate_action_server.cpp`)
- 초기 pose 검증 #1: `proj_ini >= target_distance_` → `finish_abort(-2)` (PARAM_ERR) 였던 것을, **도착 판정과 동일 기준** `proj_ini >= target_distance_ - arrival_tolerance_` 이면 **성공(status 0)** 으로 종료하도록 변경.
- 성공 종료는 정상 완료 경로([:550-556])와 동일: cmd_vel 0, path viz clear, result{status=0, actual_distance=proj_ini, lateral=e_d_ini, heading=e_th_ini}, `goal_handle->succeed()`.
- heading(#2)·lateral(#3) 검증은 불변 (도착과 무관한 별개 거부 사유).

### 배경
- 코드 모순: 주행 중 `projection >= target - tol` 이면 도착=성공([:415-418])인데, 시작 시 같은 조건은 PARAM_ERR로 거부([:262])했음. 사용자 지적: 이미 도착했으면 성공 코드를 보내야 함.
- WP2 단독 실행 로그 `path_angle=0.0, robot already past goal` 의 직접 원인. `docs/issues_fixes/issues_and_fixes.md` 참조.

### 검증
- `colcon build --packages-select amr_motion_control_2wd` exit 0 (16.5s). ⚠️ 실기: WP2 단독 Run → 이미 도착 상태면 SUCCESS, 미도착이면 정상 주행 확인 필요.

## 2026-05-29 / 23:19 / (미커밋) / 수정 pose 구독을 PoseStamped /robot_pose SSOT로 전환 및 QoS 정합

### 수정 (`src/translate_reverse_action_server.cpp`, `include/amr_motion_control_2wd/translate_reverse_action_server.hpp`)
- `pose_sub_`를 PoseWithCovarianceStamped(`/rtabmap/localization_pose`, `translate_pose_topic` param 상속) → PoseStamped(`/robot_pose`) + `rclcpp::QoS(10).reliable()` 로 전환
- 콜백명 `rtabmapPoseCallback` → `robotPoseCallback` 으로 변경, msg 접근 `msg->pose.position` 으로 수정
- `pose_qos` param 로직 제거, 초기화 로그에서 qos 출력 제거
- include `geometry_msgs/msg/pose_with_covariance_stamped.hpp` → `geometry_msgs/msg/pose_stamped.hpp` 교체
- `/safety/speed_limit` 구독 QoS를 `rclcpp::QoS(10)` → `rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()` 로 변경

### 수정 (`src/yaw_control_action_server.cpp`, `include/amr_motion_control_2wd/yaw_control_action_server.hpp`)
- translate_reverse와 동일하게 `pose_sub_` 를 PoseStamped `/robot_pose` RELIABLE 로 전환
- 콜백명 `robotPoseCallback` 으로 통일, `pose_qos` 로직·include 정리

### 배경
동일 토픽-다른 타입 충돌로 amr_motion_control_2wd 크래시 → /robot_pose(PoseStamped) SSOT 통일 및 QoS 정합

## 2026-05-29 / 20:33 / (미커밋) / 검증 /robot_pose 마이그레이션 빌드 통과

### 검증 (`include/amr_motion_control_2wd/motion_common.hpp`, 5개 액션 서버)
- `colcon build --packages-select amr_motion_control_2wd waypoint_manager tr_works_pose_publisher` 실행
  - `Summary: 3 packages finished [0.29s]` — 컴파일 에러 0, 빌드 통과 (exit 0)
- `PoseCache`(`/robot_pose` 구독, QoS `rclcpp::QoS(10).reliable()`) 기반 액션 서버 5종 정상 컴파일 확인: spin, translate, translate_reverse, yaw_control, pure_pursuit
- `lookupRobotPose()`/`lookupTfYaw()` 는 stale/미수신 시 false 반환 → 기존 실패 경로(abort) 보존, 시그니처/반환 의미 불변

## 2026-05-29 / 18:40 / (미커밋) / 수정 Spin 진행량 측정: IMU delta 적분 → start IMU 대비 직접 차이

### 수정 (`src/spin_action_server.cpp`)
- `accumulated_angle`(IMU delta 누적) 및 `prev_yaw`, `readImuDelta()` 호출 제거
- 신규: `measureProgress()` 람다 — 등록한 `start_imu_yaw_rad` 대비 절대 변화량을 회전 방향(sign)으로 투영해 매 주기 직접 측정
  - `progress_deg = sign * normalizeAngle(last_yaw_rad_.load() - start_imu_yaw_rad) * 180/π` (음수면 0 클램프)
- 사다리꼴 프로파일/fine-correction의 진행량 입력을 `accumulated_angle` → `progress_deg`로 전부 교체
- settling 후 델타 누적 블록 제거 → `progress_deg` 재측정이 관성 회전 자동 반영

### 배경
- 기존 방식은 매 주기 IMU delta를 적분(누적). 적분은 자기보정이 없어 단일 샘플 글리치/누락이 영구 오차로 남음
- 회전량은 좌표계 무관 물리량 → `start_imu` 등록 후 `target_imu = start_imu + rotation_needed`에 도달할 때까지 직접 차이로 제어 가능
- 이 코드는 `rotation_needed`를 ±180°로 정규화(최단경로)하므로 멀티턴이 없어 적분의 유일한 정당성(>180° 추적)이 부재 → 적분은 단점만 남는 구조였음
- 등록 중이던 `start_imu_yaw_rad`(line 140)이 기존엔 로그용으로만 쓰였으나 이제 제어에 사용

### 영향
- 매 주기 절대 측정 → 단일 샘플 오차가 누적되지 않고 다음 주기에 자동 회복
- 오버슈트 시 `angle_error < 0` → `correction_sign` 자동 역전(기존 로직 유지)으로 양방향 자기보정
- `imu_deadband_rad_` 멤버는 미사용 상태로 잔존(헤더/생성자 미변경, 다른 영향 없음)

### 한계
- target이 ±180°에 근접 + 오버슈트로 |start 대비 변화량| > 180° 시 `normalizeAngle` 부호 반전으로 progress 튐 가능 (fine 속도 3 dps로 발생 가능성 낮음, 가드 미적용)

### 검증
- `colcon build --packages-select amr_motion_control_2wd` 성공 (23.1s)
- 런타임 검증 필요: 실제 spin 목표각 도달 정밀도 확인 예정

## 2026-04-21 / (미커밋) / 수정 Translate heading 검증 임계값 45° → 85°

### 수정 (`config/motion_params.yaml`)
- `translate_heading_threshold_deg: 85.0` 신규 추가 (기본값 45.0 대신 명시적 85.0 로드)

### 배경
- Codex1 미션 WP2→WP3 전환에서 `PARAM_ERR` 발생
- 원인: `translate_action_server.cpp:280` 초기 pose 검증 — heading_err > `heading_threshold_` 이면 `finish_abort(-2)` 로 거부
- WP1→WP2 도착 heading ≈ 2.4°, WP2→WP3 path_angle = 52.6° → 초기 heading error ≈ 50° > 45° → 거부
- `waypoint_manager/segment_planner`에서 DRIVE_TRANSLATE pre-SPIN 자동 삽입을 제거("AUTO 외 명시대로" 원칙)한 직후 발생 — Translate 제어기의 물리적 한계(≤45°)가 걸림
- 사용자 판단: Translate 제어기가 주행 중 heading 보정 가능하므로 임계값 상향이 적절

### 영향
- 85° heading error 상태에서 TRANSLATE 시작 가능 — 제어기는 cross-track + heading 피드백으로 수렴
- 90° (완전 직각) 이상은 여전히 거부 → 물리적 불가능 영역은 pre-SPIN 또는 DRIVE_TURN 필요
- `translate_reverse_heading_threshold_deg`, `yaw_control_heading_threshold_deg` 는 변경 없음 (각각 독립적 용도)

### 검증
- `colcon build --packages-select amr_motion_control_2wd` 성공 (0.72s)
- 런타임 검증 필요: Codex1 재실행 → WP3 도달 확인 예정

## 2026-04-20 / 23:10 - d7f1218 / 추가 — Pure Pursuit Action Server

SIL 테스트(Global Planner + Pure Pursuit 파이프라인, 20 케이스) 을 위해 pure pursuit
액션 서버를 이식. 참조 레포: https://github.com/kuks2309/ros2_3dslam_ws.git

### 추가
- `include/amr_motion_control_2wd/pure_pursuit_action_server.hpp`
- `src/pure_pursuit_action_server.cpp`

### 수정
- `include/amr_motion_control_2wd/motion_common.hpp` — `ActiveAction` enum 에
  `PURE_PURSUIT` 값 추가, `to_string()` 에 해당 분기 추가.
- `src/main.cpp` — `PurePursuitActionServer` 인스턴스 생성 1 줄 추가.
- `CMakeLists.txt` — `add_executable` 리스트에 `src/pure_pursuit_action_server.cpp`
  추가.

### 주요 설계 (참조 레포 대비 단순화)
- 경로는 `/pure_pursuit/path` 토픽(nav_msgs/Path, `transient_local`)으로 수신
  (액션 goal 에 path 포함시키지 않음 — 기존 action 정의 유지).
- 로봇 pose 는 TF `map → robot_base_frame` lookup (default `base_link`).
- `cmd_vel` 발행 토픽, action 이름, path 토픽, robot_base_frame 모두 파라미터로
  override 가능 (SIL 전용 인스턴스에서 `cmd_vel_sil`, `amr_motion_pure_pursuit_sil`,
  `pure_pursuit/path_sil`, `base_link_sil` 로 구동).
- 액션 상호 배제 (`g_active_action`) 준수: 다른 액션 진행 중이면 goal reject.
- Trapezoidal speed profile, goal tolerance 도달 시 succeed, 측방 오차 초과 시
  abort, 전체 timeout 초과 시 abort.

### 검증 (SIL 20 케이스)
- anchor=(0.000, 0.000, 0.000), base goal=(5.736, -0.082), Δx ∈ [-3, +5] m,
  Δy ∈ {-0.4, 0, +0.4}.
- 결과: 20/20 success, final error 0.217–0.270 m (tolerance 0.25 m),
  elapsed 8.88–35.52 s.
- 결과 파일: `experiments/sil_global_planner_pp_20260419/results/{case_01..20,summary.csv}`.

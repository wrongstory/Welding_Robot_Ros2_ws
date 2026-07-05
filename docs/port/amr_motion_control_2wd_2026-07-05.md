# Port record — amr_motion_control_2wd (DD, 2WD)

Date: 2026-07-05 (KST)
Source: `~/Project/JJH/FITO/FITO_AMR_ros2_ws/src/Robot_Control/AMR-Motion-Control`
Target: `Welding_Robot_Ros2_ws/src/Robot_Control/AMR-Motion-Control`
ROS distro: humble

## Scope
User request: port `amr_motion_control_2wd` only; do not apply the runtime safety guard.
Chosen option A: keep the mandatory build dependency `amr_safety_core`; omit the
runtime guard node `cmd_vel_safety_guard`.

## Packages ported
| Package | Fidelity | Note |
| --- | --- | --- |
| `amr_motion_control_2wd` | **verbatim** (diff 0 vs origin) | the DD motion node (6 action servers) |
| `amr_safety_core` | **verbatim** (diff 0 vs origin) | header-only lib, hard compile dep of the node |
| `tc_msgs` | **modified (trimmed)** | only `SafetyStatus.msg` kept; motor-specific msgs omitted |
| `amr_interfaces` | (already present) | unchanged |

## Divergences from origin (intentional)
1. **`cmd_vel_safety_guard` NOT ported** — per user, no runtime safety guard. The
   motion node builds/runs without it (it was only an `exec_depend`).
2. **`tc_msgs` trimmed to `SafetyStatus.msg`** — the only tc_msgs type used by the
   port set is `tc_msgs::msg::SafetyStatus` (via `amr_safety_core::SafetySubscriber`).
   The motion node itself references no tc_msgs type directly. All motor-specific
   messages (WheelMotor, WheelMotorState, MotorDriver, MotorStatus, Odometry, Io,
   AmrAlarm) and the services/actions were omitted because they vary per motor and
   must be re-defined for the target drivetrain (user instruction 2026-07-05 11:22).

## Runtime safety behavior (option A)
`amr_safety_core::SafetySubscriber` is compiled in and subscribes to `/safety/*`,
but with no publisher present it no-ops: `speed_limit_` defaults to `+inf`
(`std::min(vx, inf)` = no clamp) and `safety_status_` defaults to `STATUS_NORMAL`
(`is_dangerous()` = false). So "safety not applied" holds at runtime.

## Verification
- `colcon build --packages-select tc_msgs amr_interfaces amr_safety_core amr_motion_control_2wd`
  → **Summary: 4 packages finished [34.0s]**, exit 0.
- Node smoke run: `ros2 run amr_motion_control_2wd amr_motion_control_2wd_node`
  → all 6 action servers initialized, "node started", no crash without /safety publishers.
- Action servers exposed: `/amr_spin_action`, `/amr_turn_action`, `/amr_yaw_control_action`,
  `/amr_translate_action`, `/amr_translate_reverse_action`, `/amr_motion_pure_pursuit`.

## Follow-ups
- Define motor-specific `tc_msgs` messages for the target drivetrain when wiring the
  motor driver / kinematics layer.
- If runtime safety is wanted later, port `cmd_vel_safety_guard` and its `/safety/*` publishers.

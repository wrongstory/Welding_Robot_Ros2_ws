// CiA 402 상태머신 디코딩/지령 — 순수 함수(부수효과 없음).
//
// 출처: servo_core/src/cia402.cpp 의 미러(동일 DS402 표준 상수·로직).
//   - 이 도구는 보드(원격)에 독립 배치되므로 servo_core 를 링크하지 않고
//     검증된 로직만 헤더-온리로 복제한다. 원본 변경 시 양쪽을 함께 갱신할 것.
// 표준: CiA 402 (IEC 61800-7-201) Drive State Machine.
//   Statusword=0x6041, Controlword=0x6040 — L7NH manual §3 Object Dictionary 참조.
#pragma once

#include <cstdint>

namespace cia402
{

enum class DriveState : uint8_t
{
    kNotReadyToSwitchOn,
    kSwitchOnDisabled,
    kReadyToSwitchOn,
    kSwitchedOn,
    kOperationEnabled,
    kQuickStopActive,
    kFaultReactionActive,
    kFault,
    kUnknown,
};

// --- Statusword(0x6041) 비트 마스크 (DS402 표준) ---
inline constexpr uint16_t kMask4F = 0x004F; // bits 0,1,2,3,6
inline constexpr uint16_t kMask6F = 0x006F; // bits 0,1,2,3,5,6
inline constexpr uint16_t kTargetReachedBit = 1u << 10;
inline constexpr uint16_t kFaultBit = 1u << 3;

// --- Controlword(0x6040) 명령어 (DS402 표준) ---
inline constexpr uint16_t kCwShutdown = 0x0006;        // → Ready to switch on
inline constexpr uint16_t kCwSwitchOn = 0x0007;        // → Switched on
inline constexpr uint16_t kCwEnableOperation = 0x000F; // → Operation enabled
inline constexpr uint16_t kCwQuickStop = 0x0002;       // → Quick stop active
inline constexpr uint16_t kCwFaultReset = 0x0080;      // bit7 상승에지로 알람 리셋

inline DriveState decode_state(uint16_t sw)
{
    if ((sw & kMask4F) == 0x0040)
        return DriveState::kSwitchOnDisabled;
    if ((sw & kMask6F) == 0x0021)
        return DriveState::kReadyToSwitchOn;
    if ((sw & kMask6F) == 0x0023)
        return DriveState::kSwitchedOn;
    if ((sw & kMask6F) == 0x0027)
        return DriveState::kOperationEnabled;
    if ((sw & kMask6F) == 0x0007)
        return DriveState::kQuickStopActive;
    if ((sw & kMask4F) == 0x000F)
        return DriveState::kFaultReactionActive;
    if ((sw & kMask4F) == 0x0008)
        return DriveState::kFault;
    if ((sw & kMask4F) == 0x0000)
        return DriveState::kNotReadyToSwitchOn;
    return DriveState::kUnknown;
}

// 현재 상태에서 "Operation enabled" 로 한 단계 전진시키는 Controlword.
// 상태머신은 한 주기에 한 단계씩 천이하므로 주기 루프에서 반복 호출한다.
inline uint16_t controlword_to_enable(DriveState current)
{
    switch (current)
    {
    case DriveState::kFault:
        return kCwFaultReset; // 먼저 리셋
    case DriveState::kSwitchOnDisabled:
        return kCwShutdown;
    case DriveState::kReadyToSwitchOn:
        return kCwSwitchOn;
    case DriveState::kSwitchedOn:
    case DriveState::kOperationEnabled:
        return kCwEnableOperation;
    case DriveState::kQuickStopActive:
        return kCwShutdown; // quick stop 해제 후 재진입
    default:
        break;
    }
    return kCwShutdown; // kNotReadyToSwitchOn/kUnknown 등 — 안전 기본값
}

inline uint16_t controlword_disable() { return kCwShutdown; }
inline uint16_t controlword_quick_stop() { return kCwQuickStop; }
inline uint16_t controlword_fault_reset() { return kCwFaultReset; }

inline bool is_target_reached(uint16_t sw) { return (sw & kTargetReachedBit) != 0; }
inline bool has_fault(uint16_t sw) { return (sw & kFaultBit) != 0; }

inline const char *state_name(DriveState s)
{
    switch (s)
    {
    case DriveState::kNotReadyToSwitchOn: return "NotReadyToSwitchOn";
    case DriveState::kSwitchOnDisabled:   return "SwitchOnDisabled";
    case DriveState::kReadyToSwitchOn:    return "ReadyToSwitchOn";
    case DriveState::kSwitchedOn:         return "SwitchedOn";
    case DriveState::kOperationEnabled:   return "OperationEnabled";
    case DriveState::kQuickStopActive:    return "QuickStopActive";
    case DriveState::kFaultReactionActive:return "FaultReactionActive";
    case DriveState::kFault:              return "Fault";
    default:                              return "Unknown";
    }
}

} // namespace cia402

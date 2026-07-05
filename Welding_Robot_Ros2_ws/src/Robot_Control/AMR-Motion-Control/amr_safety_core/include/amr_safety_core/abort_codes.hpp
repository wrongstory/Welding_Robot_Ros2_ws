#pragma once
#include <cstdint>

namespace amr_safety_core::abort_codes {

constexpr int8_t CANCEL                  = -1;
constexpr int8_t POSE_VALIDATION_FAILED  = -2;
constexpr int8_t LOCALIZATION_LOST       = -3;
constexpr int8_t LOCALIZATION_HEALTH_FAIL = -4;
constexpr int8_t THREAD_FAILURE          = -5;
constexpr int8_t SAFETY_DANGEROUS        = -6;  // safety_status DANGEROUS / e-stop

// alias: SafetySubscriber pose_valid 실패 (값 동일 = LOCALIZATION_HEALTH_FAIL)
constexpr int8_t POSE_INVALID            = -4;

}  // namespace amr_safety_core::abort_codes

// Copyright 2026 T-AMR
#pragma once

#include <cstdint>

namespace amr_safety_core
{

enum class SafetyState { NORMAL, WARNING, DANGEROUS, LATCHED };

class SafetyStateMachine
{
public:
  explicit SafetyStateMachine(
    int warning_enter_count = 2,
    int warning_exit_count = 5,
    bool dangerous_latch = true)
  : warning_enter_count_(warning_enter_count),
    warning_exit_count_(warning_exit_count),
    dangerous_latch_(dangerous_latch),
    state_(SafetyState::NORMAL),
    warn_consecutive_(0),
    normal_consecutive_(0),
    warn_scale_(0.5)
  {}

  void apply(uint8_t status_msg)
  {
    // STATUS_DANGEROUS = 2
    if (status_msg == 2u) {
      if (dangerous_latch_) {
        state_ = SafetyState::LATCHED;
      } else {
        state_ = SafetyState::DANGEROUS;
      }
      warn_consecutive_ = 0;
      normal_consecutive_ = 0;
      return;
    }

    // Once LATCHED, stay LATCHED regardless of input
    if (state_ == SafetyState::LATCHED) {
      return;
    }

    // STATUS_WARNING = 1
    if (status_msg == 1u) {
      warn_consecutive_++;
      normal_consecutive_ = 0;
      if (state_ == SafetyState::NORMAL && warn_consecutive_ >= warning_enter_count_) {
        state_ = SafetyState::WARNING;
      }
      return;
    }

    // STATUS_NORMAL = 0
    normal_consecutive_++;
    warn_consecutive_ = 0;
    if (state_ == SafetyState::WARNING && normal_consecutive_ >= warning_exit_count_) {
      state_ = SafetyState::NORMAL;
    }
  }

  void force_dangerous()
  {
    state_ = SafetyState::LATCHED;
    warn_consecutive_ = 0;
    normal_consecutive_ = 0;
  }

  bool release_latch(bool normal_debounce_ok)
  {
    if (normal_debounce_ok && state_ == SafetyState::LATCHED) {
      state_ = SafetyState::NORMAL;
      warn_consecutive_ = 0;
      normal_consecutive_ = 0;
      return true;
    }
    return false;
  }

  SafetyState state() const {return state_;}

  double scale() const
  {
    switch (state_) {
      case SafetyState::NORMAL:    return 1.0;
      case SafetyState::WARNING:   return warn_scale_;
      case SafetyState::DANGEROUS: return 0.0;
      case SafetyState::LATCHED:   return 0.0;
    }
    return 0.0;
  }

  void set_warn_scale(double s) {warn_scale_ = s;}

private:
  int warning_enter_count_;
  int warning_exit_count_;
  bool dangerous_latch_;
  SafetyState state_;
  int warn_consecutive_;
  int normal_consecutive_;
  double warn_scale_;
};

}  // namespace amr_safety_core

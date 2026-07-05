// Copyright 2026 T-AMR
#include <gtest/gtest.h>
#include "amr_safety_core/safety_state_machine.hpp"

using amr_safety_core::SafetyState;
using amr_safety_core::SafetyStateMachine;

static constexpr uint8_t NORMAL = 0;
static constexpr uint8_t WARNING = 1;
static constexpr uint8_t DANGEROUS = 2;

// Test 1: NORMAL->WARNING requires warning_enter_count consecutive WARNINGs
TEST(SafetyStateMachineTest, NormalToWarningHysteresis)
{
  SafetyStateMachine sm(2, 5, true);
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);

  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);  // only 1, need 2

  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::WARNING);  // 2 consecutive -> transitions
}

// Test 2: WARNING->NORMAL requires warning_exit_count consecutive NORMALs
TEST(SafetyStateMachineTest, WarningToNormalHysteresis)
{
  SafetyStateMachine sm(2, 5, true);
  sm.apply(WARNING);
  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::WARNING);

  for (int i = 0; i < 4; i++) {
    sm.apply(NORMAL);
    EXPECT_EQ(sm.state(), SafetyState::WARNING);  // not yet 5
  }
  sm.apply(NORMAL);
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);  // 5th consecutive -> transitions
}

// Test 3: DANGEROUS -> LATCHED when dangerous_latch=true
TEST(SafetyStateMachineTest, DangerousToLatched)
{
  SafetyStateMachine sm(2, 5, true);
  sm.apply(DANGEROUS);
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);
}

// Test 4: DANGEROUS -> DANGEROUS (not LATCHED) when dangerous_latch=false
TEST(SafetyStateMachineTest, DangerousNonLatch)
{
  SafetyStateMachine sm(2, 5, false);
  sm.apply(DANGEROUS);
  EXPECT_EQ(sm.state(), SafetyState::DANGEROUS);
}

// Test 5: LATCHED stays LATCHED even if NORMAL is received
TEST(SafetyStateMachineTest, LatchedStaysLatched)
{
  SafetyStateMachine sm(2, 5, true);
  sm.apply(DANGEROUS);
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);

  for (int i = 0; i < 10; i++) {
    sm.apply(NORMAL);
  }
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);
}

// Test 6: release_latch returns false if normal_debounce_ok=false
TEST(SafetyStateMachineTest, ReleaseLatchFailsIfNotDebounced)
{
  SafetyStateMachine sm(2, 5, true);
  sm.apply(DANGEROUS);
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);

  bool result = sm.release_latch(false);
  EXPECT_FALSE(result);
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);
}

// Test 7: release_latch returns true and state->NORMAL if allowed
TEST(SafetyStateMachineTest, ReleaseLatchSucceeds)
{
  SafetyStateMachine sm(2, 5, true);
  sm.apply(DANGEROUS);
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);

  bool result = sm.release_latch(true);
  EXPECT_TRUE(result);
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);
}

// Test 8: force_dangerous always latches regardless of current state
TEST(SafetyStateMachineTest, ForceDangerousAlwaysLatches)
{
  SafetyStateMachine sm(2, 5, true);
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);
  sm.force_dangerous();
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);

  // release then force again from WARNING
  sm.release_latch(true);
  sm.apply(WARNING);
  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::WARNING);
  sm.force_dangerous();
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);
}

// Test 9: scale() returns correct values for each state
TEST(SafetyStateMachineTest, ScaleValues)
{
  SafetyStateMachine sm(2, 5, true);

  // NORMAL -> scale 1.0
  EXPECT_DOUBLE_EQ(sm.scale(), 1.0);

  // WARNING -> scale 0.5 (default warn_scale)
  sm.apply(WARNING);
  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::WARNING);
  EXPECT_DOUBLE_EQ(sm.scale(), 0.5);

  // set_warn_scale changes WARNING scale
  sm.set_warn_scale(0.3);
  EXPECT_DOUBLE_EQ(sm.scale(), 0.3);

  // LATCHED -> scale 0.0
  sm.apply(DANGEROUS);
  EXPECT_EQ(sm.state(), SafetyState::LATCHED);
  EXPECT_DOUBLE_EQ(sm.scale(), 0.0);
}

// Test 10: Hysteresis counter resets when switching direction
TEST(SafetyStateMachineTest, HysteresisCounterReset)
{
  SafetyStateMachine sm(3, 3, true);

  // Apply 2 WARNINGs (not enough to enter WARNING), then 1 NORMAL
  sm.apply(WARNING);
  sm.apply(WARNING);
  sm.apply(NORMAL);  // resets warn counter
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);

  // Now need 3 more WARNINGs to enter WARNING
  sm.apply(WARNING);
  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::NORMAL);
  sm.apply(WARNING);
  EXPECT_EQ(sm.state(), SafetyState::WARNING);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

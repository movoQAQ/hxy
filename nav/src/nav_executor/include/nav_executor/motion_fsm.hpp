#pragma once
#include <cstdint>
namespace nav_executor {
enum class MotionState : uint8_t {
  IDLE,
  FOLLOW,
  HOLD,
  PREPARE_SPIN,
  SPIN,
  ESCAPE,
  HAZARD_RECOVERY
};
enum class MotionEvent {
  STOP,
  PATH_ACCEPTED,
  GOAL_REACHED,
  SPIN_REQUESTED,
  SPIN_READY,
  SPIN_RELEASED,
  STALLED,
  HAZARD,
  RECOVERY_SUCCEEDED,
  RECOVERY_FAILED
};
class MotionFSM {
public:
  MotionState state() const { return state_; }
  bool recovering() const {
    return state_ == MotionState::ESCAPE ||
           state_ == MotionState::HAZARD_RECOVERY;
  }
  bool overridesPath() const {
    return recovering() || state_ == MotionState::SPIN ||
           state_ == MotionState::PREPARE_SPIN;
  }
  MotionState consume(MotionEvent event);
  static const char *name(MotionState state);

private:
  MotionState state_{MotionState::IDLE};
};
} // namespace nav_executor

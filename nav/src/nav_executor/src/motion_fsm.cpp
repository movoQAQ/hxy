#include "nav_executor/motion_fsm.hpp"
namespace nav_executor {
MotionState MotionFSM::consume(MotionEvent event) {
  switch (event) {
  case MotionEvent::STOP:
  case MotionEvent::RECOVERY_FAILED:
  case MotionEvent::RECOVERY_SUCCEEDED:
    state_ = MotionState::IDLE;
    break;
  case MotionEvent::PATH_ACCEPTED:
    if (!overridesPath())
      state_ = MotionState::FOLLOW;
    break;
  case MotionEvent::GOAL_REACHED:
    if (state_ == MotionState::FOLLOW)
      state_ = MotionState::HOLD;
    break;
  case MotionEvent::SPIN_REQUESTED:
    if (!recovering())
      state_ = MotionState::PREPARE_SPIN;
    break;
  case MotionEvent::SPIN_READY:
    if (state_ == MotionState::PREPARE_SPIN)
      state_ = MotionState::SPIN;
    break;
  case MotionEvent::SPIN_RELEASED:
    if (state_ == MotionState::PREPARE_SPIN || state_ == MotionState::SPIN)
      state_ = MotionState::IDLE;
    break;
  case MotionEvent::STALLED:
    if (state_ == MotionState::FOLLOW)
      state_ = MotionState::ESCAPE;
    break;
  case MotionEvent::HAZARD:
    state_ = MotionState::HAZARD_RECOVERY;
    break;
  }
  return state_;
}
const char *MotionFSM::name(MotionState state) {
  switch (state) {
  case MotionState::IDLE:
    return "IDLE";
  case MotionState::FOLLOW:
    return "FOLLOW";
  case MotionState::HOLD:
    return "HOLD";
  case MotionState::PREPARE_SPIN:
    return "PREPARE_SPIN";
  case MotionState::SPIN:
    return "SPIN";
  case MotionState::ESCAPE:
    return "ESCAPE";
  case MotionState::HAZARD_RECOVERY:
    return "HAZARD_RECOVERY";
  }
  return "UNKNOWN";
}
} // namespace nav_executor

#pragma once

#include <mutex>
#include <shared_mutex>




enum class State
{
  UNINITIALIZED = 0,
  INITIALIZED = 1,
  ARMED = 2,
  TAKEOFF = 3,
  LAND = 4,
  HOVER_ONBOARD = 5,
  HOVER_OFFBOARD = 6,
  POSITION = 7
};




class QuadState
{
public:

  QuadState() {};

  inline void setState(const State &state);

  inline bool isValidTransition(const State& state);

private:
  int a{0};
  State state_{State::UNINITIALIZED};

  std::mutex mutex_;

};



inline void QuadState::setState(const State &state)
{
  mutex_.lock(); // TODO we should consider using lock guards
  
  state_ = state;
  
  mutex_.unlock();
}


inline bool QuadState::isValidTransition(const State& new_state)
{
  mutex_.lock();

  bool valid = false;

  switch (state_)
  {
    case State::UNINITIALIZED:
      valid = (new_state == State::INITIALIZED);
      break;
    case State::INITIALIZED:
      valid = (new_state == State::ARMED);
      break;
    case State::ARMED:
      valid = (new_state == State::TAKEOFF) || (new_state == State::ARMED);
      break;
    case State::TAKEOFF:
      valid = (new_state == State::HOVER_ONBOARD) || (new_state == State::HOVER_OFFBOARD) || (new_state == State::POSITION);
      break;
    case State::LAND:
      valid = (new_state == State::HOVER_ONBOARD) || (new_state == State::HOVER_OFFBOARD) || (new_state == State::POSITION);
      break;
    case State::HOVER_ONBOARD:
      valid = (new_state == State::HOVER_OFFBOARD) || (new_state == State::POSITION) || (new_state == State::LAND);
      break;
    case State::HOVER_OFFBOARD:
      valid = (new_state == State::HOVER_ONBOARD) || (new_state == State::POSITION) || (new_state == State::LAND);
      break;
    case State::POSITION:
      valid = (new_state == State::HOVER_ONBOARD) || (new_state == State::HOVER_OFFBOARD) || (new_state == State::TAKEOFF) || (new_state == State::LAND);
      break;
    default:
      valid = false;
      break;
  }

  mutex_.unlock();

  return valid;
}
#pragma once

#include <array>
#include <atomic>




class Telemetry
{
public:
  Telemetry() {
    p_x.store(0);
    p_y.store(0);
    p_z.store(0);

    roll.store(0);
    pitch.store(0);
    yaw.store(0);

    v_x.store(0);
    v_y.store(0);
    v_z.store(0);
  }
  ~Telemetry() {}

  inline void setPosition(const std::array<float,3> &position) {
    p_x.store(position[0]);
    p_y.store(position[1]);
    p_z.store(position[2]);
  }

  inline void setAttitude(const std::array<float,3> &orientation) {
    roll.store(orientation[0]);
    pitch.store(orientation[1]);
    yaw.store(orientation[2]);
  }

  inline void setVelocity(const std::array<float,3> &velocity) {
    v_x.store(velocity[0]);
    v_y.store(velocity[1]);
    v_z.store(velocity[2]);
  }

  inline std::array<float,3> getPosition() const {
    return {p_x.load(), p_y.load(), p_z.load()};
  }

  inline std::array<float,3> getAttitude() const {
    return {roll.load(), pitch.load(), yaw.load()};
  }

  inline std::array<float,3> getVelocity() const {
    return {v_x.load(), v_y.load(), v_z.load()};
  }

private:
  std::atomic<float> p_x;
  std::atomic<float> p_y;
  std::atomic<float> p_z;

  std::atomic<float> roll;
  std::atomic<float> pitch;
  std::atomic<float> yaw;

  std::atomic<float> v_x;
  std::atomic<float> v_y;
  std::atomic<float> v_z;
};
#include "quad_control/mavsdk_wrapper.hpp"

MavsdkWrapper::MavsdkWrapper() {}


int MavsdkWrapper::initialize(const std::string& port)
{
  // create mavsdk instance
  mavsdk_ = std::make_shared<mavsdk::Mavsdk>();

  
  // create connection
  mavsdk::ConnectionResult connection_result = mavsdk_->add_any_connection(port);
  if (connection_result != mavsdk::ConnectionResult::Success) { return 1; } // Connection failed


  // connect to system
  system_ = get_system(*mavsdk_);
  if (!system_) { return 2; } // System connection failed


  // instantiate plugins
  action_ = std::make_shared<mavsdk::Action>(system_);
  offboard_ = std::make_shared<mavsdk::Offboard>(system_);
  telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
  passthrough_ = std::make_shared<mavsdk::MavlinkPassthrough>(system_);

  return 0;
}


int MavsdkWrapper::sendArmRequest() const
{
  return int(action_->arm());
}

int MavsdkWrapper::sendTakeoffRequest() const
{
  return int(action_->takeoff());
}

int MavsdkWrapper::sendLandRequest() const
{
  return int(action_->land());
}

int MavsdkWrapper::sendOffboardRequest() const
{
  return int(offboard_->start());
}

int MavsdkWrapper::sendPositionMessage (const std::array<float,3> &position, const float yaw) const
{
  // create message
  mavsdk::Offboard::PositionNedYaw pos_msg{}; // TODO should we instantiate the message on every function call or rather once as a resuable member variable?

  // fill message: transform from north-west-up to PX4's north-east-down
  pos_msg.north_m = position[0];
  pos_msg.east_m = -position[1];
  pos_msg.down_m = -position[2];
  pos_msg.yaw_deg = -yaw;

  // send message to px4
  return int(offboard_->set_position_ned(pos_msg)); // TODO is this thread safe?
}

int MavsdkWrapper::sendAccelerationMessage (const std::array<float,3> &acceleration) const
{
  mavsdk::Offboard::AccelerationNed acc_msg{};

  // transform from north-west-up to PX4's north-east-down frame
  acc_msg.north_m_s2 = acceleration[0];
  acc_msg.east_m_s2 = -acceleration[1];
  acc_msg.down_m_s2 = -acceleration[2];

  return int(offboard_->set_acceleration_ned(acc_msg));
}


bool MavsdkWrapper::isArmable() const
{
  return telemetry_->health().is_armable;
}

bool MavsdkWrapper::isLocalPositionOk() const
{
  return telemetry_->health().is_local_position_ok;
}

std::array<float, 3> MavsdkWrapper::get_ned_posiition() {
  // get px4 pose
    auto px4_pos_vel = telemetry_->position_velocity_ned();
    std::array<float, 3> ned_position =  {px4_pos_vel.position.north_m, px4_pos_vel.position.east_m, px4_pos_vel.position.down_m};
    return ned_position;
}

std::string actionResultToString(const int index) {
  assert(index >= 0 && index < 13);
  return MAVSDK_ACTION_RESULTS[index];
}


std::string offboardResultToString(const int index) {
  assert(index >= 0 && index < 8);
  return MAVSDK_OFFBOARD_RESULTS[index];
}



std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk &mavsdk) {
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    }
  });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
    std::cerr << "No autopilot found.\n";
    return nullptr;
  }

  // Get discovered system now.
  return fut.get();
}

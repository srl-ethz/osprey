#pragma once

#include <cassert>
#include <iostream>
#include <future>
#include <math.h>

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


const std::string MAVSDK_OFFBOARD_RESULTS[] = { "Unknown",
                                                "Success",
                                                "NoSystem",
                                                "ConnectionError",
                                                "Busy",
                                                "CommandDenied",
                                                "Timeout",
                                                "NoSetpointSet"};

const std::string MAVSDK_ACTION_RESULTS[] = { "Unknown",
                                              "Success",
                                              "NoSystem",
                                              "ConnectionError",
                                              "Busy",
                                              "CommandDenied",
                                              "CommandDeniedLandedStateUnknown",
                                              "CommandDeniedNotLanded",
                                              "Timeout",
                                              "VtolTransitionSupportUnknown",
                                              "NoVtolTransitionSupport",
                                              "ParameterError",
                                              "Unsupported"};


class MavsdkWrapper
{
public:
  MavsdkWrapper();
  ~MavsdkWrapper() {};

  int initialize(const std::string &port);

  int sendArmRequest() const;

  int sendTakeoffRequest() const;

  int sendLandRequest() const;

  // PRE: steady stream of offboard messages
  int sendOffboardRequest() const;

  int sendPositionMessage (const std::array<float,3> &position, const float yaw = 0) const;

  int sendAccelerationMessage (const std::array<float,3> &acceleration) const;

  bool isArmable() const;

  bool isLocalPositionOk() const;

  std::array<float, 3> get_ned_posiition();

  // TODO get battery status

private:
  // mavsdk
  std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_;
};



// helpers
std::string actionResultToString(const int index);

std::string offboardResultToString(const int index);


/**
 * Find PX4 flight controller.
 * 
 * @return Shared pointer to discovered system.
*/
std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk &mavsdk);
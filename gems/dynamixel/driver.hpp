#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include "driver_constants.hpp"
#include "driver_types.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/math/types.hpp"
#include "external/robotis/c++/include/dynamixel_sdk/dynamixel_sdk.h"
#include "util.hpp"

namespace dynamixel_sdk = dynamixel;

namespace isaac {
namespace dynamixel {

class Driver {
 private:
  dynamixel_sdk::PacketHandler *packet_handler_;
  dynamixel_sdk::PortHandler *port_handler_;

  isaac::Vector3i GetServoValuesInt(const Eigen::Ref<const isaac::Vector3i>& servo_ids, int control_table_address, std::string name);
  void SetServoValuesInt(const Eigen::Ref<const isaac::Vector3i>& servo_ids, const Eigen::Ref<const isaac::Vector3i>& servo_values, int control_table_address, std::string name);

 public:
  Configuration configuration_;

  void Connect();
  void Disconnect();
  dynamixel_sdk::PacketHandler *GetPacketHandler();
  dynamixel_sdk::PortHandler *GetPortHandler();
  isaac::Vector3i GetPresentSpeeds(const Eigen::Ref<const isaac::Vector3i>& servo_ids);
  isaac::Vector3i GetRealtimeTicks(const Eigen::Ref<const isaac::Vector3i>& servo_ids);
  bool OpenPort();
  int RpmToSpeed(double &rpm);
  bool SetBaudRate();
  void SetConfiguration(Configuration& configuration);
  void SetMovingSpeeds(const Eigen::Ref<const isaac::Vector3i>& servo_ids, const Eigen::Ref<const isaac::Vector3i>& servo_values);
  void SetTorqueLimit(const Eigen::Ref<const isaac::Vector3i>& servo_ids, int limit);
  double SpeedToRpm(int &speed);
  void ToggleTorque(const Eigen::Ref<const isaac::Vector3i>& servo_ids, bool enabled);
};

}  // namespace dynamixel
}  // namespace isaac
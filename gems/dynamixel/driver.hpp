#pragma once

#include <array>
#include <string>
#include <vector>
#include "driver_constants.hpp"
#include "driver_types.hpp"
#include "engine/core/logger.hpp"
#include "external/robotis/c++/include/dynamixel_sdk/dynamixel_sdk.h"
#include "util.hpp"

namespace dynamixel_sdk = dynamixel;

namespace isaac {
namespace dynamixel {

class Driver {
 private:
  dynamixel_sdk::PacketHandler *packet_handler_;
  dynamixel_sdk::PortHandler *port_handler_;

  std::vector<int> GetServoValueIds(std::vector<ServoValue<int>> servo_values);
  std::vector<ServoValue<int>> GetServoValuesInt(std::vector<int> &servo_ids, int control_table_address, std::string name);
  void SetServoValuesInt(std::vector<ServoValue<int>> servo_values, int control_table_address, std::string name);

 public:
  Configuration configuration_;

  void Connect();
  void Disconnect();
  dynamixel_sdk::PacketHandler *GetPacketHandler();
  dynamixel_sdk::PortHandler *GetPortHandler();
  std::vector<ServoValue<int>> GetPresentSpeeds(std::vector<int> &servo_ids);
  std::vector<ServoValue<int>> GetRealtimeTicks(std::vector<int> &servo_ids);
  bool OpenPort();
  int RpmToSpeed(double rpm);
  bool SetBaudRate();
  void SetConfiguration(Configuration configuration);
  void SetMovingSpeeds(std::vector<ServoValue<int>> &speeds);
  void SetTorqueLimit(std::vector<int> &servo_ids, int limit);
  double SpeedToRpm(int speed);
  void ToggleTorque(std::vector<int> &servo_ids, bool enabled);
};

}  // namespace dynamixel
}  // namespace isaac
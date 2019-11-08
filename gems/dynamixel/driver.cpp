#include "driver.hpp"

namespace isaac {
namespace dynamixel {

// private

// TODO: generic template?
isaac::Vector3i Driver::GetServoValuesInt(CommandByteSize command_byte_size,
                                          const Eigen::Ref<const isaac::Vector3i>& servo_ids, int control_table_address,
                                          std::string name) {
  uint8_t error;
  int result;
  int value;

  isaac::Vector3i servo_values(servo_ids.rows(), 1);

  for (int i = 0; i < servo_ids.rows(); i++) {
    if (configuration_.debug == true) {
      LOG_DEBUG("Getting %s for servo ID %d.\n", name.c_str(), servo_ids(i));
    }

    switch (command_byte_size) {
      case ONE:
        uint8_t value_uint8;
        result = packet_handler_->read1ByteTxRx(port_handler_, servo_ids(i), control_table_address, &value_uint8, &error);
        value = static_cast<int>(value_uint8);
        break;
      case TWO:
        uint16_t value_uint16;
        result = packet_handler_->read2ByteTxRx(port_handler_, servo_ids(i), control_table_address, &value_uint16, &error);
        value = static_cast<int>(value_uint16);
        break;
    }

    if (result != configuration_.control_values.success) {
      LOG_ERROR(
          "Error communicating with servo ID %d: %s (result: %i) while "
          "getting %s.\n",
          servo_ids(i), packet_handler_->getTxRxResult(result), result, name.c_str());
      throw CommandError::COMMUNICATION;
    } else if (error != 0) {
      if (configuration_.debug == true) {
        LOG_ERROR("Error getting %s for servo ID %d: %s\n", name.c_str(), servo_ids(i),
                  packet_handler_->getRxPacketError(error));
      }
      throw CommandError::SPECIFIED;
    } else {
      if (configuration_.debug == true) {
        LOG_DEBUG("Got %s for servo ID %d: %i.\n", name.c_str(), servo_ids(i), value);
      }
    }

    servo_values(i, 0) = value;
  }

  return servo_values;
}

void Driver::SetServoValuesInt(CommandByteSize command_byte_size, const Eigen::Ref<const isaac::Vector3i>& servo_ids,
                               const Eigen::Ref<const isaac::Vector3i>& servo_values, int control_table_address,
                               std::string name) {
  ASSERT(servo_ids.rows() == servo_values.rows(), "Size of servo_ids and servo_values should match.");

  uint8_t error;
  int result;

  for (int i = 0; i < servo_values.rows(); i++) {
    if (configuration_.debug == true) {
      LOG_DEBUG("Setting %s to %i for servo ID %d.\n", name.c_str(), servo_values(i, 0), servo_ids(i));
    }

    switch (command_byte_size) {
      case ONE:
        result = packet_handler_->write1ByteTxRx(port_handler_, servo_ids(i), control_table_address,
                                                 servo_values(i, 0), &error);
        break;
      case TWO:
        result = packet_handler_->write2ByteTxRx(port_handler_, servo_ids(i), control_table_address,
                                                 servo_values(i, 0), &error);
        break;
    }

    if (result != configuration_.control_values.success) {
      LOG_ERROR(
          "Error communicating with servo ID %d: %s (result: %i) while "
          "setting %s to %i.\n",
          servo_ids(i), packet_handler_->getTxRxResult(result), result, name.c_str(), servo_values(i, 0));
      throw CommandError::COMMUNICATION;
    } else if (error != 0) {
      LOG_ERROR("Error setting %s to %i for servo ID %d: %s.\n", name.c_str(), servo_values(i, 0), servo_ids(i),
                packet_handler_->getRxPacketError(error));
      throw CommandError::SPECIFIED;
    } else {
      if (configuration_.debug == true) {
        LOG_DEBUG("Set %s to %i for servo ID %d.\n", name.c_str(), servo_values(i, 0), servo_ids(i));
      }
    }
  }
}

// public

void Driver::Connect() {
  port_handler_ = GetPortHandler();
  packet_handler_ = GetPacketHandler();

  if (!OpenPort()) {
    throw "Failed to open port.";
  }

  if (!SetBaudRate()) {
    throw "Failed to set baud rate.";
  }
}

void Driver::Disconnect() { port_handler_->closePort(); }

dynamixel_sdk::PacketHandler* Driver::GetPacketHandler() {
  if (configuration_.debug == true) {
    LOG_DEBUG("Creating packet handler for protocol version %f.\n", configuration_.protocol_version);
  }

  return dynamixel_sdk::PacketHandler::getPacketHandler(configuration_.protocol_version);
}

dynamixel_sdk::PortHandler* Driver::GetPortHandler() {
  if (configuration_.debug == true) {
    LOG_DEBUG("Creating port handler for device %s.\n", configuration_.device_name.c_str());
  }

  return dynamixel_sdk::PortHandler::getPortHandler(configuration_.device_name.c_str());
}

isaac::Vector3i Driver::GetPresentSpeeds(const Eigen::Ref<const isaac::Vector3i>& servo_ids) {
  return GetServoValuesInt(TWO, servo_ids, configuration_.control_table.ram.moving_speed, "present speed");
}

isaac::Vector3i Driver::GetRealtimeTicks(const Eigen::Ref<const isaac::Vector3i>& servo_ids) {
  return GetServoValuesInt(TWO, servo_ids, configuration_.control_table.ram.realtime_tick, "realtime tick");
}

bool Driver::OpenPort() { return port_handler_->openPort(); }

void Driver::Reboot(const Eigen::Ref<const isaac::Vector3i>& servo_ids) {
  uint8_t error;
  int result;

  for (int i = 0; i < servo_ids.size(); i++) {
    result = packet_handler_->reboot(port_handler_, servo_ids(i), &error);

    if (result != configuration_.control_values.success) {
      LOG_ERROR(
          "Error communicating with servo ID %d: %s (result: %i) while "
          "rebooting.\n",
          servo_ids(i), packet_handler_->getTxRxResult(result), result);
      throw CommandError::COMMUNICATION;
    } else if (error != 0) {
      if (configuration_.debug == true) {
        LOG_ERROR("Error rebooting servo ID %d: %s\n", servo_ids(i),
                  packet_handler_->getRxPacketError(error));
      }
      throw CommandError::SPECIFIED;
    } else {
      if (configuration_.debug == true) {
        LOG_DEBUG("Rebooted servo ID %d.\n", servo_ids(i));
      }
    }
  }
}

int Driver::RpmToSpeed(double& rpm) {
  if (rpm == 0) {
    return 0;
  } else if (rpm > 0) {  // CW
    int value = static_cast<int>(std::nearbyint(rpm / configuration_.control_values.moving_speed_rpm_unit)) +
                configuration_.control_values.moving_speed_cw_minimum;
    return clamp(value, configuration_.control_values.moving_speed_cw_minimum,
                 configuration_.control_values.moving_speed_cw_maximum);
  } else {  // CCW
    int value = static_cast<int>(std::nearbyint(-rpm / configuration_.control_values.moving_speed_rpm_unit)) +
                configuration_.control_values.moving_speed_ccw_minimum;
    return clamp(value, configuration_.control_values.moving_speed_ccw_minimum,
                 configuration_.control_values.moving_speed_ccw_maximum);
  }
}

bool Driver::SetBaudRate() { return port_handler_->setBaudRate(configuration_.baud_rate); }

void Driver::SetConfiguration(Configuration& configuration) { configuration_ = configuration; }

void Driver::SetMaxTorque(const Eigen::Ref<const isaac::Vector3i>& servo_ids, const Eigen::Ref<const isaac::Vector3i>& servo_values) {
  SetServoValuesInt(TWO, servo_ids, servo_values, configuration_.control_table.eeprom.max_torque, "max torque");
}

void Driver::SetMovingSpeeds(const Eigen::Ref<const isaac::Vector3i>& servo_ids,
                             const Eigen::Ref<const isaac::Vector3i>& servo_values) {
  SetServoValuesInt(TWO, servo_ids, servo_values, configuration_.control_table.ram.moving_speed, "moving speed");
}

void Driver::SetShutdown(const Eigen::Ref<const isaac::Vector3i>& servo_ids, const Eigen::Ref<const isaac::Vector3i>& servo_values) {
  SetServoValuesInt(ONE, servo_ids, servo_values, configuration_.control_table.eeprom.shutdown, "shutdown");
}

void Driver::SetTorqueLimit(const Eigen::Ref<const isaac::Vector3i>& servo_ids, const Eigen::Ref<const isaac::Vector3i>& servo_values) {
  SetServoValuesInt(TWO, servo_ids, servo_values, configuration_.control_table.ram.torque_limit, "torque limit");
}

double Driver::SpeedToRpm(int& speed) {
  if (speed > 0 && speed <= configuration_.control_values.moving_speed_cw_maximum) {  // CW
    return static_cast<double>(speed) * configuration_.control_values.moving_speed_rpm_unit;
  } else if (speed >= configuration_.control_values.moving_speed_ccw_minimum &&
             speed <= configuration_.control_values.moving_speed_ccw_maximum) {  // CCW
    return static_cast<double>(speed) -
           configuration_.control_values.moving_speed_ccw_minimum * configuration_.control_values.moving_speed_rpm_unit;
  } else {
    return 0.0;
  }
}

void Driver::ToggleTorque(const Eigen::Ref<const isaac::Vector3i>& servo_ids, bool enabled) {
  int torque_enable_value =
      enabled ? configuration_.control_values.torque_enable : configuration_.control_values.torque_disable;

  isaac::Vector3i servo_values;
  for (int i = 0; i < servo_ids.rows(); i++) {
    servo_values(i, 0) = torque_enable_value;
  }

  SetServoValuesInt(ONE, servo_ids, servo_values, configuration_.control_table.ram.torque_enable, "torque enabled");
}

}  // namespace dynamixel
}  // namespace isaac

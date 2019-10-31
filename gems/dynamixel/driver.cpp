#include "driver.hpp"

namespace isaac {
namespace dynamixel {

// private

// TODO: make generic?
isaac::MatrixXi Driver::GetServoValuesInt(const Eigen::Ref<const isaac::MatrixXi>& servo_ids, int control_table_address,
                                          std::string name) {
  uint8_t error;
  int result;
  uint16_t value;

  isaac::MatrixXi servo_values(servo_ids.rows(), 1);

  for (int i = 0; i < servo_ids.rows(); i++) {
    if (configuration_.debug == true) {
      LOG_DEBUG("Getting %s for servo ID %d.\n", name.c_str(), servo_ids(i, 0));
    }

    result = packet_handler_->read2ByteTxRx(port_handler_, servo_ids(i, 0), control_table_address, &value, &error);

    if (result != configuration_.control_values.success) {
      LOG_ERROR(
          "Error communicating with servo ID %d: %s (result: %i) while "
          "getting %s.\n",
          servo_ids(i, 0), packet_handler_->getTxRxResult(result), result, name.c_str());
      throw error;
    } else if (error != 0) {
      if (configuration_.debug == true) {
        LOG_ERROR("Error getting %s for servo ID %d: %s\n", name.c_str(), servo_ids(i, 0),
                  packet_handler_->getRxPacketError(error));
      }
      throw error;
    } else {
      if (configuration_.debug == true) {
        LOG_DEBUG("Got %s for servo ID %d: %i.\n", name.c_str(), servo_ids(i, 0), value);
      }
    }

    servo_values(i, 0) = value;
  }

  return servo_values;
}

void Driver::SetServoValuesInt(const Eigen::Ref<const isaac::MatrixXi>& servo_ids, const Eigen::Ref<const isaac::MatrixXi>& servo_values,
                               int control_table_address, std::string name) {
  ASSERT(servo_ids.rows() == servo_values.rows(), "Size of servo_ids and servo_values should match.");

  uint8_t error;
  int result;

  for (int i = 0; i < servo_values.rows(); i++) {
    if (configuration_.debug == true) {
      LOG_DEBUG("Setting %s to %i for servo ID %d.\n", name.c_str(), servo_values(i, 0), servo_ids(i, 0));
    }

    result = packet_handler_->write2ByteTxRx(port_handler_, servo_ids(i, 0), control_table_address, servo_values(i, 0),
                                             &error);

    if (result != configuration_.control_values.success) {
      LOG_ERROR(
          "Error communicating with servo ID %d: %s (result: %i) while "
          "setting %s to %i.\n",
          servo_ids(i, 0), packet_handler_->getTxRxResult(result), result, name.c_str(), servo_values(i, 0));
      throw error;
    } else if (error != 0) {
      LOG_ERROR("Error setting %s to %i for servo ID %d: %s.\n", name.c_str(), servo_values(i, 0), servo_ids(i, 0),
                packet_handler_->getRxPacketError(error));
      throw error;
    } else {
      if (configuration_.debug == true) {
        LOG_DEBUG("Set %s to %i for servo ID %d.\n", name.c_str(), servo_values(i, 0), servo_ids(i, 0));
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

isaac::MatrixXi Driver::GetPresentSpeeds(const Eigen::Ref<const isaac::MatrixXi>& servo_ids) {
  return GetServoValuesInt(servo_ids, configuration_.control_table.moving_speed, "present speed");
}

isaac::MatrixXi Driver::GetRealtimeTicks(const Eigen::Ref<const isaac::MatrixXi>& servo_ids) {
  return GetServoValuesInt(servo_ids, configuration_.control_table.realtime_tick, "realtime tick");
}

bool Driver::OpenPort() { return port_handler_->openPort(); }

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

void Driver::SetMovingSpeeds(const Eigen::Ref<const isaac::MatrixXi>& servo_ids, const Eigen::Ref<const isaac::MatrixXi>& servo_values) {
  SetServoValuesInt(servo_ids, servo_values, configuration_.control_table.moving_speed, "moving speed");
}

void Driver::SetTorqueLimit(const Eigen::Ref<const isaac::MatrixXi>& servo_ids, int limit) {
  isaac::MatrixXi servo_values(servo_ids.rows(), 1);
  for (int i = 0; i < servo_ids.rows(); i++) {
    servo_values(i, 0) = limit;
  }

  SetServoValuesInt(servo_ids, servo_values, configuration_.control_table.torque_limit, "torque limit");
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

void Driver::ToggleTorque(const Eigen::Ref<const isaac::MatrixXi>& servo_ids, bool enabled) {
  uint8_t error;
  int result;
  int torque_enable_value =
      enabled ? configuration_.control_values.torque_enable : configuration_.control_values.torque_disable;

  for (int i = 0; i < servo_ids.size(); i++) {
    if (configuration_.debug == true) {
      LOG_DEBUG(
          "%s torque for servo ID %d (torque enable address: %i, torque "
          "enable value: %i).\n",
          enabled ? "Enabling" : "Disabling", servo_ids(i, 0), configuration_.control_table.torque_enable,
          torque_enable_value);
    }

    result = packet_handler_->write1ByteTxRx(port_handler_, servo_ids(i, 0), configuration_.control_table.torque_enable,
                                             torque_enable_value, &error);

    if (result != configuration_.control_values.success) {
      LOG_ERROR(
          "Error communicating with servo ID %d: %s (result: %i) while %s "
          "torque.\n",
          servo_ids(i, 0), packet_handler_->getTxRxResult(result), result, enabled ? "enabling" : "disabling");
      throw error;
    } else if (error != 0) {
      LOG_ERROR("Error %s torque for servo ID %d: %s.\n", enabled ? "enabling" : "disabling", servo_ids(i, 0),
                packet_handler_->getRxPacketError(error));
      throw error;
    } else {
      if (configuration_.debug == true) {
        LOG_DEBUG("%s torque for servo ID %d.\n", enabled ? "Enabled" : "Disabled", servo_ids(i, 0));
      }
    }
  }
}

}  // namespace dynamixel
}  // namespace isaac

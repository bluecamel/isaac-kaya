#pragma once

#include <cmath>
#include <string>

namespace isaac {
namespace dynamixel {

enum CommandByteSize {
  ONE = 1,
  TWO = 2
};

enum CommandError {
  COMMUNICATION,
  SPECIFIED
};

struct ControlTableEEPROM {
  int model_number;
  int firmware_version;
  int id;
  int baud_rate;
  int return_delay_time;
  int cw_angle_limit;
  int ccw_angle_limit;
  int temperature_limit;
  int min_voltage_limit;
  int max_voltage_limit;
  int max_torque;
  int status_return_level;
  int alarm_led;
  int shutdown;
  int multi_turn_offset;
  int resolution_divider;
};

struct ControlTableRAM {
  int torque_enable;
  int led;
  int d_gain;
  int i_gain;
  int p_gain;
  int goal_position;
  int moving_speed;
  int torque_limit;
  int present_position;
  int present_speed;
  int present_load;
  int present_voltage;
  int present_temperature;
  int registered;
  int moving;
  int lock;
  int punch;
  int realtime_tick;
  int goal_acceleration;
};

struct ControlTable {
  ControlTableEEPROM eeprom;
  ControlTableRAM ram;
};

struct ControlValues {
  int success;
  int moving_speed_ccw_minimum;
  int moving_speed_ccw_maximum;
  int moving_speed_cw_minimum;
  int moving_speed_cw_maximum;
  double moving_speed_rpm_unit;
  int torque_disable;
  int torque_enable;
  int torque_limit_minimum;
  int torque_limit_maximum;
};

struct Configuration {
  int baud_rate;
  ControlTable control_table;
  ControlValues control_values;
  bool debug;
  std::string device_name;
  float protocol_version;
};

}  // namespace dynamixel
}  // namespace isaac
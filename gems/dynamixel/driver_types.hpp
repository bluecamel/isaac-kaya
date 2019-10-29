#pragma once

#include <cmath>
#include <string>

namespace isaac {
namespace dynamixel {

struct ControlTable {
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

struct ControlValues {
  int success;
  int moving_speed_ccw_minimum;
  int moving_speed_ccw_maximum;
  int moving_speed_cw_minimum;
  int moving_speed_cw_maximum;
  float moving_speed_rpm_unit;
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

template<class T>
struct ServoValue {
  int servo_id;
  T value;
};

}  // namespace dynamixel
}  // namespace isaac
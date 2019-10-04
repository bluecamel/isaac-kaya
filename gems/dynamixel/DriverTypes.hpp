#pragma once

#include <cmath>
#include <sstream>
#include <string>

namespace isaac {
namespace Dynamixel {

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
    int realtime_kick;
    int goal_acceleration;
};

struct ControlValues {
    int success;
    int moving_speed_ccw_minimum;
    int moving_speed_ccw_maximum;
    int moving_speed_cw_minimum;
    int moving_speed_cw_maximum;
    float moving_speed_rpm_unit;
};

struct Configuration {
    int baud_rate;
    ControlTable control_table;
    ControlValues control_values;
    bool debug;
    std::string device_name;
    float protocol_version;
};

struct ServoSpeed {
    int servo_id;
    int speed;
};

template <typename T>
    std::string NumberToString ( T Number ) {
        std::ostringstream ss;
        ss << Number;
        return ss.str();
    }

template <typename T>
    T clamp(const T& value, const T& low, const T& high) {
        return std::max(std::min(value, high), low);
    }

} // namespace Dynamixel
} // namspace isaac
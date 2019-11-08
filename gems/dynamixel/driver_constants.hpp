#pragma once

#include "driver_types.hpp"

namespace isaac {
namespace dynamixel {

static const ControlTableEEPROM kControlTableEEPROM_MX_12W = {
    0,   // model_number;
    2,   // firmware_version;
    3,   // id;
    4,   // baud_rate;
    5,   // return_delay_time;
    6,   // cw_angle_limit;
    8,   // ccw_angle_limit;
    11,  // temperature_limit;
    12,  // min_voltage_limit;
    13,  // max_voltage_limit;
    14,  // max_torque;
    16,  // status_return_level;
    17,  // alarm_led;
    18,  // shutdown;
    20,  // multi_turn_offset;
    22   // resolution_divider;
};

static const ControlTableRAM kControlTableRAM_MX_12W = {
    24,  // torque_enable
    25,  // led
    26,  // d_gain
    27,  // i_gain
    28,  // p_gain
    30,  // goal_position
    32,  // moving_speed
    34,  // torque_limit
    36,  // present_position
    38,  // present_speed
    40,  // present_load
    42,  // present_voltage
    43,  // present_temperature
    44,  // registered
    46,  // moving
    47,  // lock
    48,  // punch
    50,  // realtime_tick
    73   // goal_acceleration
};

static const ControlTable kControlTable_MX_12W = {kControlTableEEPROM_MX_12W, kControlTableRAM_MX_12W};

static const ControlValues kControlValues_MX_12W = {
    0,      // success
    1024,   // moving_speed_ccw_minimum
    2047,   // moving_speed_ccw_maximum
    0,      // moving_speed_cw_minimum
    1023,   // moving_speed_cw_maximum
    0.916,  // moving_speed_rpm_unit
    0,      // torque_disable
    1,      // torque_enable
    0,      // torque_limit_minimum
    1023    // torque_limit_maximum
};

}  // namespace dynamixel
}  // namespace isaac

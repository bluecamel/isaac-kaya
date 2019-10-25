#pragma once

#include <iostream>
#include <chrono>
#include <ctime>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/constants.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/messages.hpp"
#include "messages/state/holonomic_base.hpp"
#include "messages/tensor.hpp"
#include "gems/dynamixel/driver.hpp"
#include "gems/kaya_kinematics/kinematics.hpp"

namespace isaac {
namespace kaya {

class BaseDriver: public alice::Codelet {
    private:
        isaac::dynamixel::Driver dynamixel_driver_;
        isaac::kaya::Kinematics kinematics_;
        isaac::kaya::SpeedsAtTime previous_speeds_;
        std::vector<int> servo_ids_;

        void ConfigureKinematics();
        void DynamixelStart();
        void DynamixelStop();
        void LoadDynamixelDriver();
        void Move(messages::HolonomicBaseControls command);
        void Report(messages::HolonomicBaseControls command);

    public:
        void tick() override;
        void start() override;
        void stop() override;

        ISAAC_PARAM(int, baudrate, 1000000);
        ISAAC_PARAM(bool, debug_mode, false);
        ISAAC_PARAM(float, dynamixel_protocol_version, 1.0);
        ISAAC_PARAM(double, max_angular_speed, 0.3);
        ISAAC_PARAM(double, max_safe_speed, 0.3);
        ISAAC_PARAM(double, orthogonal_rotation_angle, 0);
        ISAAC_PARAM(int, servo_back, 2);
        ISAAC_PARAM(int, servo_front_left, 1);
        ISAAC_PARAM(int, servo_front_right, 3);
        ISAAC_PARAM(double, torque_limit, 0.5);
        ISAAC_PARAM(double, wheel_base_length, 0.125);
        ISAAC_PARAM(double, wheel_1_angle, M_PI / 3);
        ISAAC_PARAM(double, wheel_2_angle, M_PI);
        ISAAC_PARAM(double, wheel_3_angle, - M_PI / 3);
        ISAAC_PARAM(double, wheel_radius, 0.04);
        ISAAC_PARAM(std::string, usb_port, "/dev/ttyUSB0");

        ISAAC_PROTO_RX(StateProto, command);
        ISAAC_PROTO_TX(StateProto, state);
};

} // namespace kaya
} // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::kaya::BaseDriver);
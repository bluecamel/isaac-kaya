#include "BaseDriver.hpp"

namespace isaac {
namespace Kaya {

// public

void BaseDriver::start() {
    loadParameters();
    configureKinematics();
    dynamixelStart();
    tickPeriodically();
}

void BaseDriver::stop() {
    dynamixelStop();
}

void BaseDriver::tick() {
    if (rx_command().available()) {
        messages::HolonomicBaseControls command;
        ASSERT(FromProto(rx_command().getProto(), rx_command().buffers(), command), "Failed to parse holonomic base command.");

        try {
            move(command);
        }
        catch (char const* e) {
            std::printf("Stopping due to error: %s", e);
            stop();
            return;
        }

        report(command);
    }
}

// private

void BaseDriver::configureKinematics() {
    kinematicsConfiguration = isaac::Kaya::KinematicsConfiguration {
        get_max_angular_speed(),
        get_max_safe_speed(),
        get_orthogonal_rotation_angle(),
        get_wheel_base_length(),
        get_wheel_1_angle(),
        get_wheel_2_angle(),
        get_wheel_3_angle(),
        get_wheel_radius()
    };

    kinematics.setConfiguration(kinematicsConfiguration);
}

void BaseDriver::loadDynamixelDriver() {
    dynamixelConfiguration = isaac::Dynamixel::Configuration {
        get_baudrate(),
        isaac::Dynamixel::ControlTable_MX_12W,
        isaac::Dynamixel::ControlValues_MX_12W,
        get_debug_mode(),
        get_usb_port(),
        get_dynamixel_protocol_version()
    };

    dynamixelDriver.setConfiguration(dynamixelConfiguration);

    servoIds = { get_servo_front_left(), get_servo_back(), get_servo_front_right() };
}

void BaseDriver::loadParameters() {

}

void BaseDriver::dynamixelStart() {
    loadDynamixelDriver();
    dynamixelDriver.connect();
    dynamixelDriver.toggleTorque(servoIds, true);
    dynamixelDriver.setTorqueLimit(servoIds, get_torque_limit() * 1023);
}

void BaseDriver::dynamixelStop() {
    dynamixelDriver.setTorqueLimit(servoIds, 0);
    dynamixelDriver.toggleTorque(servoIds, false);
    dynamixelDriver.disconnect();
}

void BaseDriver::move(messages::HolonomicBaseControls command) {
    isaac::MatrixXd robot_velocities(3, 1);
    robot_velocities << command.speed_x(), command.speed_y(), command.angular_speed();

    isaac::MatrixXd wheel_velocities(3, 1);
    wheel_velocities << kinematics.wheelVelocities(robot_velocities);

    isaac::MatrixXd wheel_rpms(3, 1);
    wheel_rpms << kinematics.angularVelocitiesToRpms(wheel_velocities);

    std::vector<isaac::Dynamixel::ServoSpeed> servo_speeds = {
        { 1, dynamixelDriver.rpmToSpeed(wheel_rpms(0, 0)) },
        { 2, dynamixelDriver.rpmToSpeed(wheel_rpms(1, 0)) },
        { 3, dynamixelDriver.rpmToSpeed(wheel_rpms(2, 0)) }
    };

    dynamixelDriver.setMovingSpeeds(servo_speeds);
}

void BaseDriver::report(messages::HolonomicBaseControls command) {
    std::vector<isaac::Dynamixel::ServoSpeed> servo_speeds = dynamixelDriver.getPresentSpeeds(servoIds);

    isaac::MatrixXd wheel_rpms(3, 1);
    wheel_rpms << dynamixelDriver.speedToRpm(servo_speeds.at(0).speed),
                  dynamixelDriver.speedToRpm(servo_speeds.at(1).speed),
                  dynamixelDriver.speedToRpm(servo_speeds.at(2).speed);
    
    isaac::MatrixXd wheel_velocities(3, 1);
    wheel_velocities << kinematics.rpmsToAngularVelocities(wheel_rpms);

    isaac::MatrixXd robot_velocities(3, 1);
    robot_velocities << kinematics.robotVelocities(wheel_velocities);

    messages::HolonomicBaseState state;
    state.pos_x() = 0.0; // TODO
    state.pos_y() = 0.0; // TODO
    state.heading() = 0.0; // TODO
    state.speed_x() = robot_velocities(0, 0);
    state.speed_y() = robot_velocities(1, 0);
    state.angular_speed() = robot_velocities(2, 0);
    state.acceleration_x() = 0.0; // TODO
    state.acceleration_y() = 0.0; // TODO
    ToProto(state, tx_state().initProto(), tx_state().buffers());
    tx_state().publish();
}

} // namespace Kaya
} // namespace isaac

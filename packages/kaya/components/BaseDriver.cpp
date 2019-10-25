#include "BaseDriver.hpp"

namespace isaac {
namespace kaya {

// public

void BaseDriver::start() {
    previous_speeds_ = { 0.0, 0.0, std::chrono::system_clock::now()};
    ConfigureKinematics();
    DynamixelStart();
    tickPeriodically();
}

void BaseDriver::stop() {
    DynamixelStop();
}

void BaseDriver::tick() {
    if (rx_command().available()) {
        messages::HolonomicBaseControls command;
        ASSERT(FromProto(rx_command().getProto(), rx_command().buffers(), command), "Failed to parse holonomic base command.");

        try {
            Move(command);
        }
        catch (char const* e) {
            std::printf("Stopping due to error: %s", e);
            stop();
            return;
        }

        Report(command);
    }
}

// private

void BaseDriver::ConfigureKinematics() {
    isaac::kaya::KinematicsConfiguration kinematics_configuration = {
        get_max_angular_speed(),
        get_max_safe_speed(),
        get_orthogonal_rotation_angle(),
        get_wheel_base_length(),
        get_wheel_1_angle(),
        get_wheel_2_angle(),
        get_wheel_3_angle(),
        get_wheel_radius()
    };

    kinematics_.SetConfiguration(kinematics_configuration);
}

void BaseDriver::LoadDynamixelDriver() {
    isaac::dynamixel::Configuration dynamixel_configuration = {
        get_baudrate(),
        isaac::dynamixel::kControlTable_MX_12W,
        isaac::dynamixel::kControlValues_MX_12W,
        get_debug_mode(),
        get_usb_port(),
        get_dynamixel_protocol_version()
    };

    dynamixel_driver_.SetConfiguration(dynamixel_configuration);

    servo_ids_ = { get_servo_front_left(), get_servo_back(), get_servo_front_right() };
}

void BaseDriver::DynamixelStart() {
    LoadDynamixelDriver();
    dynamixel_driver_.Connect();
    dynamixel_driver_.ToggleTorque(servo_ids_, true);
    dynamixel_driver_.SetTorqueLimit(servo_ids_, get_torque_limit() * dynamixel_driver_.configuration_.control_values.torque_limit_maximum);
}

void BaseDriver::DynamixelStop() {
    dynamixel_driver_.SetTorqueLimit(servo_ids_, 0);
    dynamixel_driver_.ToggleTorque(servo_ids_, false);
    dynamixel_driver_.Disconnect();
}

void BaseDriver::Move(messages::HolonomicBaseControls command) {
    isaac::MatrixXd robot_velocities(3, 1);
    robot_velocities << command.speed_x(), command.speed_y(), command.angular_speed();

    isaac::MatrixXd wheel_velocities(3, 1);
    wheel_velocities << kinematics_.WheelVelocities(robot_velocities);

    isaac::MatrixXd wheel_rpms(3, 1);
    wheel_rpms << kinematics_.AngularVelocitiesToRpms(wheel_velocities);

    std::vector<isaac::dynamixel::ServoSpeed> servo_speeds = {
        { servo_ids_[0], dynamixel_driver_.RpmToSpeed(wheel_rpms(0, 0)) },
        { servo_ids_[1], dynamixel_driver_.RpmToSpeed(wheel_rpms(1, 0)) },
        { servo_ids_[2], dynamixel_driver_.RpmToSpeed(wheel_rpms(2, 0)) }
    };

    dynamixel_driver_.SetMovingSpeeds(servo_speeds);
}

void BaseDriver::Report(messages::HolonomicBaseControls command) {
    std::vector<isaac::dynamixel::ServoSpeed> servo_speeds = dynamixel_driver_.GetPresentSpeeds(servo_ids_);
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();

    isaac::MatrixXd wheel_rpms(3, 1);
    wheel_rpms << dynamixel_driver_.SpeedToRpm(servo_speeds.at(0).speed),
                  dynamixel_driver_.SpeedToRpm(servo_speeds.at(1).speed),
                  dynamixel_driver_.SpeedToRpm(servo_speeds.at(2).speed);
    
    isaac::MatrixXd wheel_velocities(3, 1);
    wheel_velocities << kinematics_.RpmsToAngularVelocities(wheel_rpms);

    isaac::MatrixXd robot_velocities(3, 1);
    robot_velocities << kinematics_.RobotVelocities(wheel_velocities);

    isaac::kaya::SpeedsAtTime current_speeds = {
      robot_velocities(0, 0),
      robot_velocities(1, 0),
      current_time
    };

    isaac::MatrixXd robot_accelerations(3, 1);
    robot_accelerations << kinematics_.RobotAccelerations(previous_speeds_, current_speeds);

    previous_speeds_ = current_speeds;

    messages::HolonomicBaseDynamics state;
    state.speed_x() = robot_velocities(0, 0);
    state.speed_y() = robot_velocities(1, 0);
    state.angular_speed() = robot_velocities(2, 0);
    state.acceleration_x() = robot_accelerations(0, 0);
    state.acceleration_y() = robot_accelerations(1, 0);
    ToProto(state, tx_state().initProto(), tx_state().buffers());
    tx_state().publish();
}

} // namespace kaya
} // namespace isaac

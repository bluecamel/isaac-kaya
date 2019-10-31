#include "BaseDriver.hpp"

namespace isaac {
namespace kaya {

// public

void BaseDriver::start() {
  LoadConfiguration();
  DynamixelStart();
  tickPeriodically();
}

void BaseDriver::stop() { DynamixelStop(); }

void BaseDriver::tick() {
  if (rx_command().available()) {
    messages::HolonomicBaseControls command;
    ASSERT(FromProto(rx_command().getProto(), rx_command().buffers(), command),
           "Failed to parse holonomic base command.");

    try {
      Move(command);
    } catch (char const* error) {
      LOG_ERROR("Stopping due to error: %s", error);
      stop();
      return;
    }

    Report(command);
  }
}

// private

void BaseDriver::ConfigureKinematics() {
  isaac::kaya::KinematicsConfiguration kinematics_configuration = {
      get_max_angular_speed(), get_max_safe_speed(), get_orthogonal_rotation_angle(),
      get_wheel_base_length(), get_wheel_1_angle(),  get_wheel_2_angle(),
      get_wheel_3_angle(),     get_wheel_radius()};

  kinematics_.SetConfiguration(kinematics_configuration);
}

void BaseDriver::LoadConfiguration() {
  max_angular_speed_ = get_max_angular_speed();
  max_safe_speed_ = get_max_safe_speed();
  previous_speeds_ = {0.0, 0.0, std::chrono::system_clock::now()};
  report_messages_to_sight = get_report_messages_to_sight();
  ConfigureKinematics();
}

void BaseDriver::LoadDynamixelDriver() {
  isaac::dynamixel::Configuration dynamixel_configuration = {
      get_baudrate(), isaac::dynamixel::kControlTable_MX_12W, isaac::dynamixel::kControlValues_MX_12W, get_debug_mode(),
      get_usb_port(), get_dynamixel_protocol_version()};

  dynamixel_driver_.SetConfiguration(dynamixel_configuration);

  servo_ids_ = (isaac::MatrixXi(3, 1) << get_servo_front_left(), get_servo_back(), get_servo_front_right()).finished();
}

void BaseDriver::DynamixelStart() {
  LoadDynamixelDriver();
  dynamixel_driver_.Connect();
  dynamixel_driver_.ToggleTorque(servo_ids_, true);
  dynamixel_driver_.SetTorqueLimit(
      servo_ids_, get_torque_limit() * dynamixel_driver_.configuration_.control_values.torque_limit_maximum);
}

void BaseDriver::DynamixelStop() {
  // dynamixel_driver_.SetTorqueLimit(servo_ids_, 0);
  // dynamixel_driver_.ToggleTorque(servo_ids_, false);
  dynamixel_driver_.Disconnect();
}

void BaseDriver::Move(messages::HolonomicBaseControls& command) {
  isaac::MatrixXd robot_velocities(3, 1);
  robot_velocities << clamp(command.speed_x(), -max_safe_speed_, max_safe_speed_),
      clamp(command.speed_y(), -max_safe_speed_, max_safe_speed_),
      clamp(command.angular_speed(), -max_angular_speed_, max_angular_speed_);

  if (robot_velocities(0, 0) != command.speed_x()) {
    LOG_WARNING("Forward speed command is out of safe bounds. Clamped from %f to %f.", command.speed_x(),
                robot_velocities(0, 0));
  }

  if (robot_velocities(1, 0) < command.speed_y()) {
    LOG_WARNING("Sideway speed command is out of safe bounds. Clamped from %f to %f.", command.speed_y(),
                robot_velocities(1, 0));
  }

  if (robot_velocities(1, 0) < command.speed_y()) {
    LOG_WARNING("Angular speed command is out of safe bounds. Clamped from %f to %f.", command.angular_speed(),
                robot_velocities(2, 0));
  }

  isaac::MatrixXd wheel_velocities(3, 1);
  wheel_velocities << kinematics_.WheelVelocities(robot_velocities);

  isaac::MatrixXd wheel_rpms(3, 1);
  wheel_rpms << kinematics_.AngularVelocitiesToRpms(wheel_velocities);

  isaac::MatrixXi servo_values(3, 1);
  servo_values << dynamixel_driver_.RpmToSpeed(wheel_rpms(0, 0)), dynamixel_driver_.RpmToSpeed(wheel_rpms(1, 0)),
      dynamixel_driver_.RpmToSpeed(wheel_rpms(2, 0));

  dynamixel_driver_.SetMovingSpeeds(servo_ids_, servo_values);

  // report to sight
  show("command.safe_speed.angle", robot_velocities(2, 0));
  show("command.safe_speed.x", robot_velocities(0, 0));
  show("command.safe_speed.y", robot_velocities(1, 0));
  show("command.servo_back_rad_per_sec", wheel_velocities(1, 0));
  show("command.servo_front_left_rad_per_sec", wheel_velocities(0, 0));
  show("command.servo_front_right_rad_per_sec", wheel_velocities(2, 0));

  if (report_messages_to_sight) {
    show("command.message.angular_speed", command.angular_speed());
    show("command.message.speed_x", command.speed_x());
    show("command.message.speed_y", command.speed_y());
  }
}

void BaseDriver::Report(messages::HolonomicBaseControls& command) {
  isaac::MatrixXi servo_speeds(3, 1);
  servo_speeds << dynamixel_driver_.GetPresentSpeeds(servo_ids_);

  std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();

  isaac::MatrixXd wheel_rpms(3, 1);
  wheel_rpms << dynamixel_driver_.SpeedToRpm(servo_speeds(0, 0)), dynamixel_driver_.SpeedToRpm(servo_speeds(1, 0)),
      dynamixel_driver_.SpeedToRpm(servo_speeds(2, 0));

  isaac::MatrixXd wheel_velocities(3, 1);
  wheel_velocities << kinematics_.RpmsToAngularVelocities(wheel_rpms);

  isaac::MatrixXd robot_velocities(3, 1);
  robot_velocities << kinematics_.RobotVelocities(wheel_velocities);

  isaac::kaya::SpeedsAtTime current_speeds = {robot_velocities(0, 0), robot_velocities(1, 0), current_time};

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

  // report to sight
  show("current.motor_back_rad_per_sec", wheel_velocities(1, 0));
  show("current.motor_front_left_rad_per_sec", wheel_velocities(0, 0));
  show("current.motor_front_right_rad_per_sec", wheel_velocities(2, 0));

  isaac::MatrixXi servo_ticks(3, 1);
  servo_ticks << dynamixel_driver_.GetRealtimeTicks(servo_ids_);

  show("current.servo_back_ticks", servo_ticks(1, 0));
  show("current.servo_front_left_ticks", servo_ticks(0, 0));
  show("current.servo_front_right_ticks", servo_ticks(2, 0));

  if (report_messages_to_sight) {
    show("state.message.angular_speed", state.angular_speed());
    show("state.message.speed_x", state.speed_x());
    show("state.message.speed_y", state.speed_y());
    show("state.message.acceleration_x", state.acceleration_x());
    show("state.message.acceleration_y", state.acceleration_y());
  }
}

}  // namespace kaya
}  // namespace isaac

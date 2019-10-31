#include "kinematics.hpp"

namespace isaac {
namespace kaya {

isaac::MatrixXd Kinematics::AngularVelocitiesToRpms(const Eigen::Ref<const isaac::MatrixXd>& wheel_velocities) {
  return wheel_velocities * angular_velocities_to_rpms_factor_;
}

isaac::Matrix3d Kinematics::OrthogonalRotationMatrix(double &angle) {
  return (isaac::MatrixXd(3, 3) <<
          std::cos(angle), std::sin(angle), 0,
          -std::sin(angle), std::cos(angle), 0,
          0, 0, 1
      ).finished();
}

// calculate acceleration from two timed velocities
isaac::MatrixXd Kinematics::RobotAccelerations(isaac::kaya::SpeedsAtTime& previous, isaac::kaya::SpeedsAtTime& current) {
  std::chrono::duration<double> elapsed_seconds = current.time - previous.time;
  return (isaac::MatrixXd(2, 1) <<
          (current.speed_x - previous.speed_x) / elapsed_seconds.count(), // x
          (current.speed_y - previous.speed_y) / elapsed_seconds.count() // y
      ).finished();
}

// forward kinematics (from robot frame to global frame)
isaac::MatrixXd Kinematics::RobotVelocities(const Eigen::Ref<const isaac::MatrixXd>& wheel_velocities) {
  return orthogonal_rotation_matrix_inverse_ * wheel_constraints_inverse_ * wheel_radii_ * wheel_velocities;
}

isaac::MatrixXd Kinematics::RpmsToAngularVelocities(const Eigen::Ref<const isaac::MatrixXd>& wheel_rpms) {
  return wheel_rpms * rpms_to_angular_velocities_factor_;
}

void Kinematics::SetConfiguration(KinematicsConfiguration& configuration) {
  configuration_ = configuration;

  angular_velocities_to_rpms_factor_ = 30 / M_PI;
  rpms_to_angular_velocities_factor_ = M_PI / 30;
  orthogonal_rotation_matrix_ = OrthogonalRotationMatrix(configuration_.orthogonal_rotation_angle);
  orthogonal_rotation_matrix_inverse_ = OrthogonalRotationMatrix(configuration_.orthogonal_rotation_angle).inverse();
  wheel_constraints_ = WheelConstraints();
  wheel_constraints_inverse_ = WheelConstraints().inverse();
  wheel_radii_ = WheelRadii();
  wheel_radii_inverse_ = WheelRadii().inverse();
}

isaac::Matrix3d Kinematics::WheelConstraints() {
  return (isaac::MatrixXd(3, 3) <<
          std::sin(configuration_.wheel_1_angle), -std::cos(configuration_.wheel_1_angle), -configuration_.wheel_base_length,
          std::sin(configuration_.wheel_2_angle), -std::cos(configuration_.wheel_2_angle), -configuration_.wheel_base_length,
          std::sin(configuration_.wheel_3_angle), -std::cos(configuration_.wheel_3_angle), -configuration_.wheel_base_length)
      .finished();
}

isaac::Matrix3d Kinematics::WheelRadii() {
  return (isaac::MatrixXd(3, 3) <<
          configuration_.wheel_radius, 0, 0,
          0, configuration_.wheel_radius, 0,
          0, 0, configuration_.wheel_radius)
      .finished();
}

// inverse kinematics (from global frame to robot frame)
isaac::MatrixXd Kinematics::WheelVelocities(const Eigen::Ref<const isaac::MatrixXd>& robot_velocities) {
  return orthogonal_rotation_matrix_ * wheel_constraints_ * wheel_radii_inverse_ * robot_velocities;
}

}  // namespace kaya
}  // namespace isaac
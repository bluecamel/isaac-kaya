#include "kinematics.hpp"

namespace isaac {
namespace kaya {

isaac::Vector3d Kinematics::AngularVelocitiesToRpms(const Eigen::Ref<const isaac::Vector3d>& wheel_velocities) {
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
isaac::Vector2d Kinematics::RobotAccelerations(isaac::kaya::SpeedsAtTime& previous, isaac::kaya::SpeedsAtTime& current) {
  std::chrono::duration<double> elapsed_seconds = current.time - previous.time;
  return (isaac::VectorXd(2) <<
          (current.speed_x - previous.speed_x),
          (current.speed_y - previous.speed_y)
      ).finished() * (1 / elapsed_seconds.count());
}

// forward kinematics (from robot frame to global frame)
isaac::Vector3d Kinematics::RobotVelocities(const Eigen::Ref<const isaac::Vector3d>& wheel_velocities) {
  return robot_velocities_factor_ * wheel_velocities;
}

isaac::Vector3d Kinematics::RpmsToAngularVelocities(const Eigen::Ref<const isaac::Vector3d>& wheel_rpms) {
  return wheel_rpms * rpms_to_angular_velocities_factor_;
}

void Kinematics::SetConfiguration(KinematicsConfiguration& configuration) {
  configuration_ = configuration;

  angular_velocities_to_rpms_factor_ = 30 / M_PI;
  rpms_to_angular_velocities_factor_ = M_PI / 30;
  orthogonal_rotation_matrix_ = OrthogonalRotationMatrix(configuration_.orthogonal_rotation_angle);
  orthogonal_rotation_matrix_inverse_ = OrthogonalRotationMatrix(configuration_.orthogonal_rotation_angle).inverse();
  robot_velocities_factor_ = orthogonal_rotation_matrix_inverse_ * wheel_constraints_inverse_ * wheel_radii_;
  wheel_constraints_ = WheelConstraints();
  wheel_constraints_inverse_ = WheelConstraints().inverse();
  wheel_radii_ = WheelRadii();
  wheel_radii_inverse_ = WheelRadii().inverse();
  wheel_velocities_factor_ = orthogonal_rotation_matrix_ * wheel_constraints_ * wheel_radii_inverse_;
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
isaac::Vector3d Kinematics::WheelVelocities(const Eigen::Ref<const isaac::Vector3d>& robot_velocities) {
  return wheel_velocities_factor_ * robot_velocities;
}

}  // namespace kaya
}  // namespace isaac
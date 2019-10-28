#pragma once

#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include "engine/core/math/types.hpp"

namespace isaac {
namespace kaya {

struct KinematicsConfiguration {
  double max_angular_speed;
  double max_safe_speed;
  double orthogonal_rotation_angle;
  double wheel_base_length;
  double wheel_1_angle;
  double wheel_2_angle;
  double wheel_3_angle;
  double wheel_radius;
};

struct SpeedsAtTime {
  double speed_x;
  double speed_y;
  std::chrono::time_point<std::chrono::system_clock> time;
};

class Kinematics {
 private:
  double angular_velocities_to_rpms_factor_;
  KinematicsConfiguration configuration_;
  isaac::Matrix3d orthogonal_rotation_matrix_;
  isaac::Matrix3d orthogonal_rotation_matrix_inverse_;
  double rpms_to_angular_velocities_factor_;
  isaac::Matrix3d wheel_constraints_;
  isaac::Matrix3d wheel_constraints_inverse_;
  isaac::Matrix3d wheel_radii_;
  isaac::Matrix3d wheel_radii_inverse_;

 public:
  isaac::MatrixXd AngularVelocitiesToRpms(isaac::MatrixXd wheel_velocities);
  isaac::Matrix3d OrthogonalRotationMatrix(double angle);
  isaac::MatrixXd RobotAccelerations(isaac::kaya::SpeedsAtTime previous, isaac::kaya::SpeedsAtTime current);
  isaac::MatrixXd RobotVelocities(isaac::MatrixXd wheel_velocities);
  isaac::MatrixXd RpmsToAngularVelocities(isaac::MatrixXd wheel_rpms);
  void SetConfiguration(KinematicsConfiguration _configuration);
  isaac::Matrix3d WheelConstraints();
  isaac::Matrix3d WheelRadii();
  isaac::MatrixXd WheelVelocities(isaac::MatrixXd robot_velocities);
};

}  // namespace kaya
}  // namespace isaac
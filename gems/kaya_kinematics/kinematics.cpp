#include "kinematics.hpp"
#include <iostream>

namespace isaac {
namespace kaya {

isaac::MatrixXd Kinematics::AngularVelocitiesToRpms(isaac::MatrixXd wheel_velocities) {
    return wheel_velocities * angular_velocities_to_rpms_factor_;
}

isaac::Matrix3d Kinematics::OrthogonalRotationMatrix(double angle) {
    return (isaac::MatrixXd(3, 3) <<
        std::cos(angle), std::sin(angle), 0,
        -std::sin(angle), std::cos(angle), 0,
        0, 0, 1
    ).finished();
}

isaac::MatrixXd Kinematics::RobotVelocities(isaac::MatrixXd wheel_velocities) {
    return orthogonal_rotation_matrix_inverse_ * wheel_constraints_inverse_ * wheel_radii_ * wheel_velocities;
}

isaac::MatrixXd Kinematics::RpmsToAngularVelocities(isaac::MatrixXd wheel_rpms) {
    return wheel_rpms * rpms_to_angular_velocities_factor_;
}

void Kinematics::SetConfiguration(KinematicsConfiguration configuration) {
    configuration_ = configuration;

    angular_velocities_to_rpms_factor_ = 30 / M_PI;
    rpms_to_angular_velocities_factor_ = M_PI / 30;
    orthogonal_rotation_matrix_ = OrthogonalRotationMatrix(configuration_.orthogonal_rotation_angle);
    orthogonal_rotation_matrix_inverse_ = OrthogonalRotationMatrix(0).inverse();
    wheel_constraints_ = WheelConstraints();
    wheel_constraints_inverse_ = WheelConstraints().inverse();
    wheel_radii_ = WheelRadii();
    wheel_radii_inverse_ = WheelRadii().inverse();
}

isaac::Matrix3d Kinematics::WheelConstraints() {
    return (isaac::MatrixXd(3, 3) <<
        std::sin(configuration_.wheel_1_angle), -std::cos(configuration_.wheel_1_angle), -configuration_.wheel_base_length,
        std::sin(configuration_.wheel_2_angle), -std::cos(configuration_.wheel_2_angle), -configuration_.wheel_base_length,
        std::sin(configuration_.wheel_3_angle), -std::cos(configuration_.wheel_3_angle), -configuration_.wheel_base_length
    ).finished();
}

isaac::Matrix3d Kinematics::WheelRadii() {
    return (isaac::MatrixXd(3, 3) <<
        configuration_.wheel_radius, 0, 0,
        0, configuration_.wheel_radius, 0,
        0, 0, configuration_.wheel_radius
    ).finished();
}

isaac::MatrixXd Kinematics::WheelVelocities(isaac::MatrixXd robot_velocities) {
    return orthogonal_rotation_matrix_ * wheel_constraints_ * wheel_radii_inverse_ * robot_velocities;
}

} // namespace Kaya
} // namespace isaac
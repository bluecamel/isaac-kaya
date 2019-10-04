#include "Kinematics.hpp"
#include <iostream>

namespace isaac {
namespace Kaya {

isaac::MatrixXd Kinematics::angularVelocitiesToRpms(isaac::MatrixXd wheel_velocities) {
    return wheel_velocities * _angularVelocitiesToRpmsFactor;
}

isaac::Matrix3d Kinematics::orthogonalRotationMatrix(double angle) {
    return (isaac::MatrixXd(3, 3) <<
        std::cos(angle), std::sin(angle), 0,
        -std::sin(angle), std::cos(angle), 0,
        0, 0, 1
    ).finished();
}

isaac::MatrixXd Kinematics::robotVelocities(isaac::MatrixXd wheel_velocities) {
    return _orthogonalRotationMatrixInverse * _wheelConstraintsInverse * _wheelRadii * wheel_velocities;
}

isaac::MatrixXd Kinematics::rpmsToAngularVelocities(isaac::MatrixXd wheel_rpms) {
    return wheel_rpms * _rpmsToAngularVelocitiesFactor;
}

void Kinematics::setConfiguration(KinematicsConfiguration _configuration) {
    configuration = _configuration;

    _angularVelocitiesToRpmsFactor = 30 / M_PI;
    _rpmsToAngularVelocitiesFactor = M_PI / 30;
    _orthogonalRotationMatrix = orthogonalRotationMatrix(configuration.orthogonalRotationAngle);
    _orthogonalRotationMatrixInverse = orthogonalRotationMatrix(0).inverse();
    _wheelConstraints = wheelConstraints();
    _wheelConstraintsInverse = wheelConstraints().inverse();
    _wheelRadii = wheelRadii();
    _wheelRadiiInverse = wheelRadii().inverse();
}

isaac::Matrix3d Kinematics::wheelConstraints() {
    return (isaac::MatrixXd(3, 3) <<
        std::sin(configuration.wheel1Angle), -std::cos(configuration.wheel1Angle), -configuration.wheelBaseLength,
        std::sin(configuration.wheel2Angle), -std::cos(configuration.wheel2Angle), -configuration.wheelBaseLength,
        std::sin(configuration.wheel3Angle), -std::cos(configuration.wheel3Angle), -configuration.wheelBaseLength
    ).finished();
}

isaac::Matrix3d Kinematics::wheelRadii() {
    return (isaac::MatrixXd(3, 3) <<
        configuration.wheelRadius, 0, 0,
        0, configuration.wheelRadius, 0,
        0, 0, configuration.wheelRadius
    ).finished();
}

isaac::MatrixXd Kinematics::wheelVelocities(isaac::MatrixXd robot_velocities) {
    return _orthogonalRotationMatrix * _wheelConstraints * _wheelRadiiInverse * robot_velocities;
}

} // namespace Kaya
} // namespace isaac
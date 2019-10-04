#pragma once

#include "engine/core/math/types.hpp"
#include <cmath>

namespace isaac {
namespace Kaya {

struct KinematicsConfiguration {
    double maxAngularSpeed;
    double maxSafeSpeed;
    double orthogonalRotationAngle;
    double wheelBaseLength;
    double wheel1Angle;
    double wheel2Angle;
    double wheel3Angle;
    double wheelRadius;
};

class Kinematics {
    private:
        KinematicsConfiguration configuration;

        double _angularVelocitiesToRpmsFactor;
        isaac::Matrix3d _orthogonalRotationMatrix;
        isaac::Matrix3d _orthogonalRotationMatrixInverse;
        double _rpmsToAngularVelocitiesFactor;
        isaac::Matrix3d _wheelConstraints;
        isaac::Matrix3d _wheelConstraintsInverse;
        isaac::Matrix3d _wheelRadii;
        isaac::Matrix3d _wheelRadiiInverse;

    public:
        isaac::MatrixXd angularVelocitiesToRpms(isaac::MatrixXd wheel_velocities);
        isaac::Matrix3d orthogonalRotationMatrix(double angle);
        isaac::MatrixXd robotVelocities(isaac::MatrixXd wheel_velocities);
        isaac::MatrixXd rpmsToAngularVelocities(isaac::MatrixXd wheel_rpms);
        void setConfiguration(KinematicsConfiguration _configuration);
        isaac::Matrix3d wheelConstraints();
        isaac::Matrix3d wheelRadii();
        isaac::MatrixXd wheelVelocities(isaac::MatrixXd robot_velocities);
};

} // namespace Kaya
} // namespace isaac
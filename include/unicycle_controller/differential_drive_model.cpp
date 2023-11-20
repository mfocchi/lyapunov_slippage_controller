#include "differential_drive_model.h"

DifferentialDriveModel::DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox)
{
    this->wheel_distance = wheel_distance;
    this->wheel_radius = wheel_radius;
    this->gearbox = gearbox;
    setUnicycleSpeed(0.0,0.0);
}

void DifferentialDriveModel::setUnicycleSpeed(double lin_speed, double ang_speed)
{
    linear_speed = lin_speed;
    angular_speed = ang_speed;
}

void DifferentialDriveModel::setDifferentialSpeed(double left_wheel_speed, double right_wheel_speed)
{
    linear_speed = 0.5*wheel_radius*(left_wheel_speed + right_wheel_speed);
    angular_speed = wheel_radius*(-left_wheel_speed + right_wheel_speed) / wheel_distance;
}

double DifferentialDriveModel::getLeftWheelRotationalSpeed() const
{
    double wheel_speed = linear_speed - angular_speed * (0.5*wheel_distance);
    return wheel_speed / wheel_radius;
}
double DifferentialDriveModel::getRightWheelRotationalSpeed() const
{
    double wheel_speed = linear_speed + angular_speed * (0.5*wheel_distance);
    return wheel_speed / wheel_radius;
}

double DifferentialDriveModel::getLeftMotorRotationalSpeed() const
{
    return getLeftWheelRotationalSpeed() * gearbox;
}
double DifferentialDriveModel::getRightMotorRotationalSpeed() const
{
    return getRightWheelRotationalSpeed() * gearbox;
}

double DifferentialDriveModel::getLinearSpeed() const
{
    return linear_speed;
}
double DifferentialDriveModel::getAngularSpeed() const
{
    return angular_speed;
}

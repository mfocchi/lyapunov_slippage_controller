#ifndef DIFFERENTIAL_DRIVE_MODEL_H
#define DIFFERENTIAL_DRIVE_MODEL_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

class DifferentialDriveModel
{
private:
    

public:
    double wheel_radius;
    double wheel_distance;
    double linear_speed;
    double angular_speed;
    double gearbox; 
    DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox);

    void setUnicycleSpeed(double lin_speed, double ang_speed);
    void setDifferentialSpeed(double left_wheel_speed, double right_wheel_speed);
    double getLeftWheelRotationalSpeed() const;
    double getRightWheelRotationalSpeed() const;
    double getLeftMotorRotationalSpeed() const;
    double getRightMotorRotationalSpeed() const;
    double getLinearSpeed() const;
    double getAngularSpeed() const;
};


#endif

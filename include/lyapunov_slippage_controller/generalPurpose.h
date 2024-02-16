#ifndef GENERAL_PURPOSE_H
#define GENERAL_PURPOSE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <cmath>
#define GRAVITY 9.81
#define POW2GRAVITY 96.24

struct Measurement1D {
    double data;
    double time;
};

enum Verbosity {
    ABSENT,
    MINIMAL,
    DEBUG,
};

void computeQuaternion(double roll, double pitch, double yaw, std::vector<double> *q);
Eigen::MatrixXd computeRFdotVecDerivative(const Eigen::VectorXd& q, const Eigen::Vector3d& vec);
Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d& vec);
Eigen::Matrix3d computeRF(const Eigen::VectorXd& q);
Eigen::Matrix2d compute2DRotation(double angle);
Eigen::MatrixXd fillMatrix(std::vector<double> vec);
Eigen::VectorXd fillVector(std::vector<double> vec);
void repeatMatrixDiagonal(const Eigen::MatrixXd& M, Eigen::MatrixXd* Mout);
int  whereIsStringInVector(const std::string& name, const std::vector<std::string>& vec);
double sinc(double val);
double angleWithinPI(double angle);
double angleWithin2PI(double angle);
double movingAverage(double val, double n);
double computeTurningRadius(double v, double omega);
double applyLimits(int idx, int lower_buond, int upper_bound);
double sign(double x);

#endif
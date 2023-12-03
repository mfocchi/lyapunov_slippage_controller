#ifndef GENERAL_PURPOSE_EKF_H
#define GENERAL_PURPOSE_EKF_H

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
void computeRFdotVecDerivative(const Eigen::VectorXd& q, const Eigen::Vector3d& vec, Eigen::MatrixXd* Dq_out);
void computeSkewSymmetric(const Eigen::Vector3d& vec, Eigen::Matrix3d* Skw);
void computeRF(const Eigen::VectorXd& q, Eigen::Matrix3d* RF_out);
void compute2DRotation(double angle, Eigen::Matrix2d* R_out);
void fillMatrix(Eigen::MatrixXd* M, std::vector<double> vec);
void fillVector(Eigen::VectorXd* V, std::vector<double> vec);
void repeatMatrixDiagonal(const Eigen::MatrixXd& M, Eigen::MatrixXd* Mout);
int  whereIsStringInVector(const std::string& name, const std::vector<std::string>& vec);
double sinc(double val);
double angleWithinPI(double angle);
double angleWithin2PI(double angle);
double movingAverage(double val, double n);
double computeTurningRadius(double v, double omega);
#endif
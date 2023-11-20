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

typedef double data_t;
typedef Eigen::Matrix<data_t, Eigen::Dynamic, 1> VectorX_t;
typedef Eigen::Matrix<data_t, 2, 1> Vector2_t;
typedef Eigen::Matrix<data_t, Eigen::Dynamic, Eigen::Dynamic> MatrixX_t;
typedef Eigen::Matrix<data_t, 3, 1> Vector3_t;
typedef Eigen::Matrix<data_t, 3, 3> Matrix3_t;
typedef Eigen::Matrix<data_t, 2, 2> Matrix2_t;


void computeQuaternion(data_t roll, data_t pitch, data_t yaw, std::vector<data_t> *q);
void computeRFdotVecDerivative(const VectorX_t& q, const Vector3_t& vec, MatrixX_t* Dq_out);
void computeSkewSymmetric(const Vector3_t& vec, Matrix3_t* Skw);
void computeRF(const VectorX_t& q, Matrix3_t* RF_out);
void compute2DRotation(data_t angle, Matrix2_t* R_out);
void fillMatrix(MatrixX_t* M, std::vector<data_t> vec);
void fillVector(VectorX_t* V, std::vector<data_t> vec);
void repeatMatrixDiagonal(const MatrixX_t& M, MatrixX_t* Mout);
int  whereIsStringInVector(const std::string& name, const std::vector<std::string>& vec);
data_t sinc(data_t val);
data_t angleWithinPI(data_t angle);
data_t angleWithin2PI(data_t angle);
data_t movingAverage(data_t val, data_t n);
#endif
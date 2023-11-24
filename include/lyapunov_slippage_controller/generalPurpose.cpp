#include "generalPurpose.h"

void computeQuaternion(double roll, double pitch, double yaw, std::vector<double> *q)
{
    double cr = cos(roll  * 0.5);
    double sr = sin(roll  * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw   * 0.5);
    double sy = sin(yaw   * 0.5);

    q->at(0) = sr * cp * cy - cr * sp * sy;
    q->at(1) = cr * sp * cy - sr * cp * sy;
    q->at(2) = cr * cp * sy - sr * sp * cy;
    q->at(3) = cr * cp * cy - sr * sp * sy;
}

void fillMatrix(Eigen::MatrixXd* M, std::vector<double> vec)
{
    for(int i = 0; i < int(vec.size()); i++)
    {
        (*M)(i) = vec.at(i);
    }
}

void fillVector(Eigen::VectorXd* V, std::vector<double> vec)
{
    for(int i = 0; i < int(vec.size()); i++)
    {
        (*V)(i) = vec.at(i);
    }
}

void compute2DRotation(double angle, Eigen::Matrix2d* R_out)
{
    *R_out << 
        cos(angle), -sin(angle),
        sin(angle),  cos(angle);
        
}

void computeRF(const Eigen::VectorXd& q, Eigen::Matrix3d* RF_out)
{
    /*
    Compute the transformation from moving to world reference frame*/
    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);
    // squares
    double qxs = qx*qx;
    double qys = qy*qy;
    double qzs = qz*qz;
    // double qws = qw*qw;

    *RF_out << 
        1-2*qys-2*qzs,    2*(qx*qy-qw*qz),  2*(qx*qz+qw*qy),
        2*(qx*qy+qw*qz),  1-2*qxs-2*qzs,    2*(qy*qz-qw*qx),
        2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),  1-2*qxs-2*qys;
}

void computeRFdotVecDerivative(const Eigen::VectorXd& q, const Eigen::Vector3d& vec, Eigen::MatrixXd* Dq_out)
{
    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);

    double X = vec(0);
    double Y = vec(1);
    double Z = vec(2);
    *Dq_out <<
        2*(Z*qy - Y*qz), 2*(Y*qy + Z*qz), 2*(Y*qx + Z*qw - 2*X*qy), 2*(Z*qy * Y*qw - 2*X*qz),
        2*(X*qz - Z*qx), 2*(X*qy - Z*qw - 2*Y*qx), 2*(X*qx + Z*qz), 2*(X*qw + Z*qy - 2*Y*qz),
        2*(Y*qx - X*qy), 2*(X*qz + Y*qw - 2*Z*qx), 2*(Y*qz - X*qw - 2*Z*qy), 2*(X*qx + Y*qy);
}

void computeSkewSymmetric(const Eigen::Vector3d& vec, Eigen::Matrix3d* Skw)
{
    /* compute skew symmetrix matrix given a vector. 
    Skw(x)*y is equivalent to x cross product y */
    double x = vec(0);
    double y = vec(1);
    double z = vec(2);
    *Skw << 
         0,-z, y,
         z, 0,-x,
        -y, x, 0;
}

void repeatMatrixDiagonal(const Eigen::MatrixXd& M, Eigen::MatrixXd* Mout)
{
    /*
    R matrix must be defined for a single observation array. This function
    repeate on the diagonal copies of the same matrix R*/
    assert(Mout->size() > 0);
    assert(M.size() < Mout->size());
    assert(Mout->size() %  M.size() == 0);
    Mout->setZero();
    int n = Mout->rows() / M.rows();
    for(int i = 0; i < n; i++)
    {
        Mout->block(i*M.rows(), i*M.cols(), M.rows(), M.cols()) << M;
    }
}

int whereIsStringInVector(const std::string& name, const std::vector<std::string>& vec)
{
    int res = -1;
    for(int i = 0; i < int(vec.size()) && res == -1; i++)
    {
        if(name.compare(vec[i]) == 0)
        {
            res = i;
        }
    }
    return res;
}  

double sinc(double val)
{
    if(val == 0.0)
        return 1.0;
    else
        return sin(val) / val;
}

/*
wrap the angle within -pi and pi*/
double angleWithinPI(double angle)
{
    double angle_out = std::fmod(angle, 2.0 * M_PI);  // Wrap angle to range -2π to 2π

    if (angle_out <= -M_PI) {
        angle_out += 2.0 * M_PI;  // Adjust angle if it's less than -π
    } else if (angle_out > M_PI) {
        angle_out -= 2.0 * M_PI;  // Adjust angle if it's greater than π
    }
    return angle_out;
}

double angleWithin2PI(double angle)
{
    double angle_out = std::fmod(angle, 2.0 * M_PI);
    if (angle_out < 0) {
        angle_out += 2*M_PI;  // Adjust angle if it's less than 0
    }
    return angle_out;
}

double computeTurningRadius(double v, double omega)
{
    if(omega == 0.0)
        return 0.0;
    else
    {
        return v / omega;
    }
}

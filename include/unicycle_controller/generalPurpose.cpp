#include "generalPurpose.h"

void computeQuaternion(data_t roll, data_t pitch, data_t yaw, std::vector<data_t> *q)
{
    data_t cr = cos(roll  * 0.5);
    data_t sr = sin(roll  * 0.5);
    data_t cp = cos(pitch * 0.5);
    data_t sp = sin(pitch * 0.5);
    data_t cy = cos(yaw   * 0.5);
    data_t sy = sin(yaw   * 0.5);

    q->at(0) = sr * cp * cy - cr * sp * sy;
    q->at(1) = cr * sp * cy - sr * cp * sy;
    q->at(2) = cr * cp * sy - sr * sp * cy;
    q->at(3) = cr * cp * cy - sr * sp * sy;
}

void fillMatrix(MatrixX_t* M, std::vector<data_t> vec)
{
    for(int i = 0; i < int(vec.size()); i++)
    {
        (*M)(i) = vec.at(i);
    }
}

void fillVector(VectorX_t* V, std::vector<data_t> vec)
{
    for(int i = 0; i < int(vec.size()); i++)
    {
        (*V)(i) = vec.at(i);
    }
}

void compute2DRotation(data_t angle, Matrix2_t* R_out)
{
    *R_out << 
        cos(angle), -sin(angle),
        sin(angle),  cos(angle);
        
}

void computeRF(const VectorX_t& q, Matrix3_t* RF_out)
{
    /*
    Compute the transformation from moving to world reference frame*/
    data_t qw = q(0);
    data_t qx = q(1);
    data_t qy = q(2);
    data_t qz = q(3);
    // squares
    data_t qxs = qx*qx;
    data_t qys = qy*qy;
    data_t qzs = qz*qz;
    // data_t qws = qw*qw;

    *RF_out << 
        1-2*qys-2*qzs,    2*(qx*qy-qw*qz),  2*(qx*qz+qw*qy),
        2*(qx*qy+qw*qz),  1-2*qxs-2*qzs,    2*(qy*qz-qw*qx),
        2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),  1-2*qxs-2*qys;
}

void computeRFdotVecDerivative(const VectorX_t& q, const Vector3_t& vec, MatrixX_t* Dq_out)
{
    data_t qw = q(0);
    data_t qx = q(1);
    data_t qy = q(2);
    data_t qz = q(3);

    data_t X = vec(0);
    data_t Y = vec(1);
    data_t Z = vec(2);
    *Dq_out <<
        2*(Z*qy - Y*qz), 2*(Y*qy + Z*qz), 2*(Y*qx + Z*qw - 2*X*qy), 2*(Z*qy * Y*qw - 2*X*qz),
        2*(X*qz - Z*qx), 2*(X*qy - Z*qw - 2*Y*qx), 2*(X*qx + Z*qz), 2*(X*qw + Z*qy - 2*Y*qz),
        2*(Y*qx - X*qy), 2*(X*qz + Y*qw - 2*Z*qx), 2*(Y*qz - X*qw - 2*Z*qy), 2*(X*qx + Y*qy);
}

void computeSkewSymmetric(const Vector3_t& vec, Matrix3_t* Skw)
{
    /* compute skew symmetrix matrix given a vector. 
    Skw(x)*y is equivalent to x cross product y */
    data_t x = vec(0);
    data_t y = vec(1);
    data_t z = vec(2);
    *Skw << 
         0,-z, y,
         z, 0,-x,
        -y, x, 0;
}

void repeatMatrixDiagonal(const MatrixX_t& M, MatrixX_t* Mout)
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

data_t sinc(data_t val)
{
    if(val == 0.0)
        return 1.0;
    else
        return sin(val) / val;
}

/*
wrap the angle within -pi and pi*/
data_t angleWithinPI(data_t angle)
{
    data_t angle_out = std::fmod(angle, 2.0 * M_PI);  // Wrap angle to range -2π to 2π

    if (angle_out <= -M_PI) {
        angle_out += 2.0 * M_PI;  // Adjust angle if it's less than -π
    } else if (angle_out > M_PI) {
        angle_out -= 2.0 * M_PI;  // Adjust angle if it's greater than π
    }
    return angle_out;
}

data_t angleWithin2PI(data_t angle)
{
    data_t angle_out = std::fmod(angle, 2.0 * M_PI);
    if (angle_out < 0) {
        angle_out += 2*M_PI;  // Adjust angle if it's less than 0
    }
    return angle_out;
}


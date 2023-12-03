#include "motionModels.h"

MotionModel::MotionModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params)
{
    assert(params.at("dt") > 0.0);
    this->integrationType = euler;
    this->dt = params.at("dt");
    this->x = xInit;
    this->u_prev = uInit;
    this->u = uInit;
}

// Assignment operator
MotionModel& MotionModel::operator=(const MotionModel& other) {
    if (this != &other) {
        integrationType = other.integrationType;
        x = other.x;
        u = other.u;
        u_prev = other.u_prev;
        dt = other.dt;
    }
    return *this;
}

MotionModel& MotionModel::operator=(MotionModel&& other) noexcept {
        if (this != &other) {
            integrationType = std::move(other.integrationType);
            x = std::move(other.x);
            u = std::move(other.u);
            u_prev = std::move(other.u_prev);
            dt = other.dt;

            other.dt = 0.0; // Mark the moved object as invalid or empty
        }
        return *this;
    }

void MotionModel::integrate() 
{
    switch (integrationType)
    {
    case euler:
        integrateEuler();
        break;
    case trapezoid:
        integrateTrapezoid();
        break;
    default:
        break;
    }
}

void MotionModel::integrateTrapezoid() 
{
    /*Integration via trapezoid rule*/
    int nx = getStateSize();
    Eigen::VectorXd fu_prev = compute_fu(u_prev);
    Eigen::VectorXd fu_next = compute_fu(u);
    Eigen::VectorXd x_dot(nx);

    x_dot << 0.5 * (fu_next + fu_prev);
    
    x.segment(0,nx) << (x.segment(0,nx) + dt * x_dot).eval(); 
    u_prev = u; // save most recent value for the control input
}

void MotionModel::integrateEuler() 
{
    /*Integration via trapezoid rule*/
    int nx = getStateSize();
    Eigen::VectorXd x_dot = compute_fu(u);
    
    x.segment(0,nx) << (x.segment(0,nx) + dt * x_dot).eval(); 
    u_prev = u; // save most recent value for the control input
}


int MotionModel::getControlInputSize() const 
{
    return this->u.size();
}                    

int MotionModel::getStateSize() const 
{
    return this->x.size();
}   

void MotionModel::setPrevControlInput(const Eigen::VectorXd& u)
{
    this->u_prev = u;
}

void MotionModel::setControlInput(const Eigen::VectorXd& u)
{
    this->u = u;
}

// UNICYCLE -------------------------------------------------
/*
f(x, u) =
x = x + v*cos(theta)*dt
y = y + v*sin(theta)*dt
theta = w*dt
*/
UnicycleModel::UnicycleModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params)
: MotionModel(xInit, uInit, params) {}



Eigen::VectorXd UnicycleModel::compute_fu(const Eigen::VectorXd& uk) const
{
    assert(x.size() >= 3);
    int nx = getStateSize();
    Eigen::VectorXd fu(nx);
    double theta = x(2);
    fu << uk(0) * std::cos(theta), uk(0) * std::sin(theta), uk(1);
    return fu;
}

Eigen::MatrixXd UnicycleModel::computeJacobian_Fx() const
{
    assert(x.size() >= 3);
    int nx = getStateSize();
    Eigen::MatrixXd Fx(nx,nx); 
    double theta = x(2);
    Fx << 
        1, 0, -u(0)*sin(theta)*dt,
        0, 1,  u(0)*cos(theta)*dt,
        0, 0,  1;
    return Fx;
}

Eigen::MatrixXd UnicycleModel::computeJacobian_Fu() const
{
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    assert(x.size() >= 3);
    int nx = getStateSize();
    int nu = getControlInputSize();
    Eigen::MatrixXd Fu(nx,nu);
    double theta = x(2);
    Fu << 
        cos(theta)*dt, 0, 
        sin(theta)*dt, 0,
        0,             dt;
    return Fu;
}

// DRONE ----------------------------------------------------

DroneModel::DroneModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params)
    : MotionModel(xInit, uInit, params) {}

Eigen::VectorXd DroneModel::compute_fu(const Eigen::VectorXd& uk) const
{
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    Eigen::VectorXd q_next(4);
    Eigen::Vector3d a_world; // drone acceleration in the world reference frame
    Eigen::Vector3d a_body;
    Eigen::Vector3d g_world(0,0,GRAVITY);
    // Eigen::Vector3d g_body;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4);
    
    Eigen::Matrix3d RF = computeRF(x.segment(6,4));
    Eigen::Matrix4d SomegaQ = computeSomegaQuaternion();
    Eigen::Matrix3d SomegaE = computeSomegaEuler();
    a_body << uk.segment(0,3) - SomegaE * x.segment(3,3) - RF.transpose() * g_world;
    // a_body << uk.segment(0,3) - RF.transpose() * g_world;
    a_world << RF * a_body;
    
    Eigen::VectorXd fu(getStateSize());
    fu.segment(3,3) <<
        a_body;
    fu.segment(0,3) <<
        RF * x.segment(3,3) + 0.5 * dt * a_world;
    fu.segment(6,4) << SomegaQ * 0.5 * x.segment(6,4);
    return fu;
}


Eigen::MatrixXd DroneModel::computeJacobian_Fx() const
{
    Eigen::Vector3d vecTmp;
    Eigen::Matrix3d RF = computeRF(x.segment(6,4));
    Eigen::Matrix3d SomegaE = computeSomegaEuler();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
    Eigen::Vector3d g_world(0,0,GRAVITY);
    

    double omegaX = this->u(3);
    double omegaY = this->u(4);
    double omegaZ = this->u(5);
    int nx = getStateSize();
    Eigen::MatrixXd Fx(nx, nx);
    Fx.setZero();
    Fx.block(0,0,3,3) << // d position / d[x,y,z]
        I;
    Fx.block(0,3,3,3) << // d position / d[u,v,w]
        RF*dt*(I - 0.5*dt*SomegaE);
    vecTmp << 
        dt*(x.segment(3,3) + 0.5*dt*(u.segment(0,3) - SomegaE*x.segment(3,3)));
    Eigen::MatrixXd RdotVec = computeRFdotVecDerivative(x.segment(6,4), vecTmp); // derivative wrt to q of R(q)*V

    Fx.block(0,6,3,4) << // d position / d[q]
        RdotVec;

    Fx.block(3,3,3,3) << // d vel / d[u,v,z]
        I - dt*SomegaE;
    vecTmp << 
        0,0,GRAVITY;
    Fx.block(3,6,3,4) << // d vel / d[q]
        -dt*RdotVec;
    Fx.block(6,6,4,4) << // d rot / d[q]
        1,-dt*omegaX*0.5, -dt*omegaY*0.5, -dt*omegaZ*0.5,
        dt*omegaX*0.5, 1,  dt*omegaZ*0.5, -dt*omegaY*0.5,
        dt*omegaY*0.5, -dt*omegaZ*0.5  ,1, dt*omegaX*0.5,
        dt*omegaZ*0.5,  dt*omegaY*0.5  ,-dt*omegaX*0.5,1;
    return Fx;
}

Eigen::MatrixXd DroneModel::computeJacobian_Fu() const
{
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
    double qw = x(6);
    double qx = x(7);
    double qy = x(8);
    double qz = x(9);
    double dts = pow(dt,2);

    Eigen::Matrix3d RF = computeRF(x.segment(6,4));
    Eigen::Matrix3d SomegaDotVel = computeSkewSymmetric(-x.segment(3,3));

    Eigen::MatrixXd Fu(getStateSize(), getControlInputSize());
    Fu.setZero();
    Fu.block(0,0,3,3) << //d[x]/d[a]
        dts * 0.5 * RF;
    // fun fact: the derivative of the skew symmetric SomegaE*vector
    //           in the omega, is the skew symmetric matrix of the 
    //           negative vector
    Fu.block(0,3,3,3) << //d[x]/d[omega]
        -dts*0.5*RF*SomegaDotVel;

    Fu.block(3,0,3,3) << //d[vel]/d[a]
        I*dt; 
    Fu.block(3,3,3,3) << //d[vel]/d[omega]
        -dt*SomegaDotVel;

    Fu.block(6,3,4,3) <<
        -qx*dt*0.5, -qy*dt*0.5, -qz*dt*0.5,
         qw*dt*0.5, -qz*dt*0.5,  qy*dt*0.5,
         qz*dt*0.5,  qw*dt*0.5, -qx*dt*0.5,
        -qy*dt*0.5,  qx*dt*0.5,  qw*dt*0.5;   
    return Fu;
}

Eigen::Vector3d DroneModel::computeGbody() const
{
    double qw = x(6);
    double qx = x(7);
    double qy = x(8);
    double qz = x(9);
    // squares
    double qxs = qx*qx;
    double qys = qy*qy;
    Eigen::Vector3d g;
    g << 
        2*(qx*qz+qw*qy) * (-GRAVITY),
        2*(qy*qz-qw*qx) * (-GRAVITY),
        1-2*qxs-2*qys   * (-GRAVITY);
    return g;
}

Eigen::Matrix4d DroneModel::computeSomegaQuaternion() const
{
    Eigen::Matrix4d Somega;
    Somega << 
        0   ,-u(3),-u(4),-u(5),
        u(3), 0   , u(5),-u(4),
        u(4),-u(5), 0   , u(3),
        u(5), u(4),-u(3), 0;
    return Somega;
}

Eigen::Matrix3d DroneModel::computeSomegaEuler() const
{
    return computeSkewSymmetric(u.segment(3,3));
}

//*******************************************************************
// KinematicTrackedVehicleModel
//*******************************************************************

/*
Consider the slips as control inputs, even though their value should be estimated
x = [x,y,theta]^w*/

KinematicTrackedVehicleModel::KinematicTrackedVehicleModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params)
    : MotionModel(xInit, uInit, params) 
{
    this->r          = params.at("sproket_radius");
    this->B = params.at("distance_between_tracks");
}

Eigen::VectorXd KinematicTrackedVehicleModel::compute_fu(const Eigen::VectorXd& u) const 
{
    double i_l = x(3);
    double i_r = x(4);
    double omega_l = u(0);
    double omega_r = u(1);
    double theta = x(2);
    double v = 0.5 * r * (omega_l*(1 - i_l) + omega_r*(1 - i_r));
    double omega = r * (omega_l*(1 - i_l) - omega_r*(1 - i_r)) / B;
    Eigen::VectorXd fu(getStateSize());
    fu << v*cos(theta), v*sin(theta), omega;
    return fu;
}

Eigen::MatrixXd KinematicTrackedVehicleModel::computeJacobian_Fx() const 
{
    Eigen::MatrixXd Fx(getStateSize(), getStateSize());
    Fx.setZero();
    return Fx;
}

Eigen::MatrixXd KinematicTrackedVehicleModel::computeJacobian_Fu() const 
{
    Eigen::MatrixXd Fu(getStateSize(), getControlInputSize());
    Fu.setZero();
    return Fu;
}

#include "motionModels.h"

MotionModel::MotionModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params)
{
    assert(params.at("dt") > 0.0);
    this->integrationType = euler;
    this->dt = params.at("dt");
    this->x = xInit;
    this->u_prev = uInit;
    this->u = uInit;
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
    VectorX_t fu_prev(nx);
    VectorX_t fu_next(nx);
    VectorX_t x_dot(nx);
    compute_fu(u_prev, &fu_prev);
    compute_fu(u,      &fu_next);

    x_dot << 0.5 * (fu_next + fu_prev);
    
    x.segment(0,nx) << (x.segment(0,nx) + dt * x_dot).eval(); 
    u_prev = u; // save most recent value for the control input
}

void MotionModel::integrateEuler() 
{
    /*Integration via trapezoid rule*/
    int nx = getStateSize();
    VectorX_t x_dot(nx);
    compute_fu(u, &x_dot);
    
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

void MotionModel::getState(VectorX_t *x_out) const
{
    *x_out = this->x;
}

void MotionModel::setPrevControlInput(const VectorX_t& u)
{
    this->u_prev = u;
}

void MotionModel::setControlInput(const VectorX_t& u)
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
UnicycleModel::UnicycleModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params)
: MotionModel(xInit, uInit, params) {}



void UnicycleModel::compute_fu(const VectorX_t& uk, VectorX_t* fu) const
{
    assert(x.size() >= 3);
    assert(fu->size() == 3);

    data_t theta = x(2);
    *fu << uk(0) * std::cos(theta), uk(0) * std::sin(theta), uk(1);
}

void UnicycleModel::computeJacobian_Fx(MatrixX_t* Fx) const
{
    assert(x.size() >= 3);
    assert(Fx->size() > 0); // has to be pre-initialized
    data_t theta = x(2);
    (*Fx) << 
        1, 0, -u(0)*sin(theta)*dt,
        0, 1,  u(0)*cos(theta)*dt,
        0, 0,  1;
}

void UnicycleModel::computeJacobian_Fu(MatrixX_t* Fu) const
{
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    assert(x.size() >= 3);
    assert(Fu->size() > 0); // has to be pre-initialized
    data_t theta = x(2);
    (*Fu) << 
        cos(theta)*dt, 0, 
        sin(theta)*dt, 0,
        0,             dt;
}

// DRONE ----------------------------------------------------

DroneModel::DroneModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params)
    : MotionModel(xInit, uInit, params) {}

void DroneModel::compute_fu(const VectorX_t& uk, VectorX_t* fu) const
{
    assert(x.size() >= 10);
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    VectorX_t q_next(4);
    MatrixX_t SomegaQ(4,4);
    Matrix3_t SomegaE;
    Matrix3_t RF;
    Vector3_t a_world; // drone acceleration in the world reference frame
    Vector3_t a_body;
    Vector3_t g_world(0,0,GRAVITY);
    // Vector3_t g_body;
    MatrixX_t I = MatrixX_t::Identity(4,4);
    
    computeRF(x.segment(6,4), &RF);
    computeSomegaQuaternion(&SomegaQ);
    computeSomegaEuler(&SomegaE);
    a_body << uk.segment(0,3) - SomegaE * x.segment(3,3) - RF.transpose() * g_world;
    // a_body << uk.segment(0,3) - RF.transpose() * g_world;
    a_world << RF * a_body;
    
    fu->segment(3,3) <<
        a_body;
    fu->segment(0,3) <<
        RF * x.segment(3,3) + 0.5 * dt * a_world;
    fu->segment(6,4) << SomegaQ * 0.5 * x.segment(6,4);
}


void DroneModel::computeJacobian_Fx(MatrixX_t* Fx) const
{
    assert(x.size() >= 10);
    assert(Fx->size() > 0);
    Vector3_t vecTmp;
    Matrix3_t RF(3,3);
    Matrix3_t SomegaE;
    MatrixX_t I = MatrixX_t::Identity(3,3);
    MatrixX_t RdotVec(3,4); // derivative wrt to q of R(q)*V
    Vector3_t g_world(0,0,GRAVITY);
    computeSomegaEuler(&SomegaE);
    computeRF(x.segment(6,4), &RF);

    data_t omegaX = this->u(3);
    data_t omegaY = this->u(4);
    data_t omegaZ = this->u(5);
    
    Fx->setZero();
    Fx->block(0,0,3,3) << // d position / d[x,y,z]
        I;
    Fx->block(0,3,3,3) << // d position / d[u,v,w]
        RF*dt*(I - 0.5*dt*SomegaE);
    vecTmp << 
        dt*(x.segment(3,3) + 0.5*dt*(u.segment(0,3) - SomegaE*x.segment(3,3)));
    computeRFdotVecDerivative(x.segment(6,4), vecTmp, &RdotVec);
    Fx->block(0,6,3,4) << // d position / d[q]
        RdotVec;

    Fx->block(3,3,3,3) << // d vel / d[u,v,z]
        I - dt*SomegaE;
    vecTmp << 
        0,0,GRAVITY;
    computeRFdotVecDerivative(x.segment(6,4), vecTmp, &RdotVec);
    Fx->block(3,6,3,4) << // d vel / d[q]
        -dt*RdotVec;
    Fx->block(6,6,4,4) << // d rot / d[q]
        1,-dt*omegaX*0.5, -dt*omegaY*0.5, -dt*omegaZ*0.5,
        dt*omegaX*0.5, 1,  dt*omegaZ*0.5, -dt*omegaY*0.5,
        dt*omegaY*0.5, -dt*omegaZ*0.5  ,1, dt*omegaX*0.5,
        dt*omegaZ*0.5,  dt*omegaY*0.5  ,-dt*omegaX*0.5,1;
}

void DroneModel::computeJacobian_Fu(MatrixX_t* Fu) const
{
    assert(x.size() >= 10);
    assert(Fu->size() > 0);
    Matrix3_t RF(3,3);
    Matrix3_t SomegaDotVel(3,3);
    MatrixX_t I = MatrixX_t::Identity(3,3);
    data_t qw = x(6);
    data_t qx = x(7);
    data_t qy = x(8);
    data_t qz = x(9);
    data_t dts = pow(dt,2);

    computeRF(x.segment(6,4), &RF);
    computeSkewSymmetric(-x.segment(3,3), &SomegaDotVel);


    Fu->setZero();
    Fu->block(0,0,3,3) << //d[x]/d[a]
        dts * 0.5 * RF;
    // fun fact: the derivative of the skew symmetric SomegaE*vector
    //           in the omega, is the skew symmetric matrix of the 
    //           negative vector
    Fu->block(0,3,3,3) << //d[x]/d[omega]
        -dts*0.5*RF*SomegaDotVel;

    Fu->block(3,0,3,3) << //d[vel]/d[a]
        I*dt; 
    Fu->block(3,3,3,3) << //d[vel]/d[omega]
        -dt*SomegaDotVel;

    Fu->block(6,3,4,3) <<
        -qx*dt*0.5, -qy*dt*0.5, -qz*dt*0.5,
         qw*dt*0.5, -qz*dt*0.5,  qy*dt*0.5,
         qz*dt*0.5,  qw*dt*0.5, -qx*dt*0.5,
        -qy*dt*0.5,  qx*dt*0.5,  qw*dt*0.5;    
}

void DroneModel::computeGbody(Vector3_t* g) const
{
    data_t qw = x(6);
    data_t qx = x(7);
    data_t qy = x(8);
    data_t qz = x(9);
    // squares
    data_t qxs = qx*qx;
    data_t qys = qy*qy;
    *g << 
        2*(qx*qz+qw*qy) * (-GRAVITY),
        2*(qy*qz-qw*qx) * (-GRAVITY),
        1-2*qxs-2*qys   * (-GRAVITY);
}

void DroneModel::computeSomegaQuaternion(MatrixX_t* Somega) const
{
    *Somega << 
        0   ,-u(3),-u(4),-u(5),
        u(3), 0   , u(5),-u(4),
        u(4),-u(5), 0   , u(3),
        u(5), u(4),-u(3), 0;
}

void DroneModel::computeSomegaEuler(Matrix3_t* Somega) const
{
    computeSkewSymmetric(u.segment(3,3), Somega);
}

//*******************************************************************
// KinematicTrackedVehicleModel
//*******************************************************************

/*
Consider the slips as control inputs, even though their value should be estimated
x = [x,y,theta]^w*/

KinematicTrackedVehicleModel::KinematicTrackedVehicleModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params)
    : MotionModel(xInit, uInit, params) 
{
    this->r          = params.at("sproket_radius");
    this->B = params.at("distance_between_tracks");
}

void KinematicTrackedVehicleModel::compute_fu(const VectorX_t& u, VectorX_t* fu) const 
{
    data_t i_l = x(3);
    data_t i_r = x(4);
    data_t omega_l = u(0);
    data_t omega_r = u(1);
    data_t theta = x(2);
    data_t v = 0.5 * r * (omega_l*(1 - i_l) + omega_r*(1 - i_r));
    data_t omega = r * (omega_l*(1 - i_l) - omega_r*(1 - i_r)) / B;
    (*fu) << v*cos(theta), v*sin(theta), omega;
}

void KinematicTrackedVehicleModel::computeJacobian_Fx(MatrixX_t* Fx) const 
{
    Fx->setZero();
}

void KinematicTrackedVehicleModel::computeJacobian_Fu(MatrixX_t* Fu) const 
{
    Fu->setZero();
}

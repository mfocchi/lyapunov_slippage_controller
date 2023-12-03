#ifndef MOTION_MODELS_H
#define MOTION_MODELS_H

#include <assert.h>  // debugging tool
#include "generalPurpose.h"
#include <map>

class MotionModel;
class UnicycleModel;
class DroneModel;

typedef std::shared_ptr<MotionModel> MotionModelPtr;
typedef std::shared_ptr<UnicycleModel> UnicycleModelPtr;
typedef std::shared_ptr<DroneModel> DroneModelPtr;

enum Integration {euler, trapezoid};
// *************************************************************************************************
// CLASS: MOTION MODEL
// *************************************************************************************************

class MotionModel
{
protected:
    Integration integrationType;
    Eigen::VectorXd x; // system state
    Eigen::VectorXd u; // control input
    Eigen::VectorXd u_prev; // previous control input
    double dt;
    
    void setPrevControlInput(const Eigen::VectorXd& u);
    void integrateEuler();
    void integrateTrapezoid(); 
    virtual Eigen::VectorXd compute_fu(const Eigen::VectorXd& uk) const = 0;
    
public:
    MotionModel(){}
    MotionModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    ~MotionModel(){}
    // Copy constructor
    MotionModel(const MotionModel& other)
        : integrationType(other.integrationType), x(other.x), u(other.u), u_prev(other.u_prev), dt(other.dt) {}

    // Move constructor
    MotionModel(MotionModel&& other) noexcept
        : integrationType(std::move(other.integrationType)), x(std::move(other.x)),
          u(std::move(other.u)), u_prev(std::move(other.u_prev)), dt(other.dt)
    {
        other.dt = 0.0; // Mark the moved object as invalid or empty
    }

    // Assignment operator
    MotionModel& operator=(const MotionModel& other);

    // Move assignment operator
    MotionModel& operator=(MotionModel&& other) noexcept;
    
    // define virtual void methods
    void integrate();
    virtual Eigen::MatrixXd computeJacobian_Fx() const = 0;
    virtual Eigen::MatrixXd computeJacobian_Fu() const = 0;
    //getters
    Eigen::VectorXd getState() const {return this->x;}
    double getStepTime() const {return this->dt;}
    int getControlInputSize() const;                    
    int getStateSize() const;                    
    //setters
    void setControlInput(const Eigen::VectorXd& u);
    void setStepTime(double dt){this->dt = dt;}
    void changeIntegrationtype(Integration intType) {integrationType = intType;}
    void resetState(const Eigen::VectorXd& xInit) {x = xInit;}
};



class UnicycleModel : public MotionModel
{
protected:
    Eigen::VectorXd compute_fu(const Eigen::VectorXd& uk) const override;
public:
    // u=[v,w]
    UnicycleModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    
    Eigen::MatrixXd computeJacobian_Fx() const override;
    Eigen::MatrixXd computeJacobian_Fu() const override;
};


class DroneModel : public MotionModel
{
protected:
    Eigen::Vector3d computeGbody() const;  
    Eigen::Matrix4d computeSomegaQuaternion() const;
    Eigen::Matrix3d computeSomegaEuler() const;
    Eigen::VectorXd compute_fu(const Eigen::VectorXd& uk) const override;

public:
    // u=[ax,ay,az,omegax,omegay,omegaz]
    DroneModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    Eigen::MatrixXd computeJacobian_Fx() const override;
    Eigen::MatrixXd computeJacobian_Fu() const override;
};


class KinematicTrackedVehicleModel : public MotionModel
{
protected:
    double r; // [m] sproket_radius
    double B; // [m] distance_between_tracks
    Eigen::VectorXd compute_fu(const Eigen::VectorXd& uk) const override;
    // void extract();
public:
    // u=[omega_left, omega_right, slip_long_left, slip_long_right]
    KinematicTrackedVehicleModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    Eigen::MatrixXd computeJacobian_Fx() const override;
    Eigen::MatrixXd computeJacobian_Fu() const override;
};

#endif
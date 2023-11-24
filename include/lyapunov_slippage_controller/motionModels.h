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
    virtual void compute_fu( const Eigen::VectorXd& uk, Eigen::VectorXd* fu) const = 0;
    
public:
    MotionModel(){}
    MotionModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    ~MotionModel(){}
    // define virtual void methods
    void integrate();
    virtual void computeJacobian_Fx(Eigen::MatrixXd* Fx) const = 0;
    virtual void computeJacobian_Fu(Eigen::MatrixXd* Fu) const = 0;
    //getters
    void getState(Eigen::VectorXd *x_out) const;
    double getStepTime() const {return dt;}
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
    void compute_fu( const Eigen::VectorXd& u, Eigen::VectorXd* fu) const override;
public:
    // u=[v,w]
    UnicycleModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    
    void computeJacobian_Fx(Eigen::MatrixXd* Fx) const override;
    void computeJacobian_Fu(Eigen::MatrixXd* Fu) const override;
};


class DroneModel : public MotionModel
{
protected:
    void computeGbody(Eigen::Vector3d* g) const;  
    void computeSomegaQuaternion(Eigen::MatrixXd* Somega) const;
    void computeSomegaEuler(Eigen::Matrix3d* Somega) const;
    void compute_fu(const Eigen::VectorXd& u, Eigen::VectorXd* fu) const override;

public:
    // u=[ax,ay,az,omegax,omegay,omegaz]
    DroneModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    void computeJacobian_Fx(Eigen::MatrixXd* Fx) const override;
    void computeJacobian_Fu(Eigen::MatrixXd* Fu) const override;
};


class KinematicTrackedVehicleModel : public MotionModel
{
protected:
    double r; // [m] sproket_radius
    double B; // [m] distance_between_tracks
    void compute_fu(const Eigen::VectorXd& u, Eigen::VectorXd* fu) const override;
    // void extract();
public:
    // u=[omega_left, omega_right, slip_long_left, slip_long_right]
    KinematicTrackedVehicleModel(const Eigen::VectorXd& xInit, const Eigen::VectorXd& uInit, const std::map<std::string, double>& params);
    void computeJacobian_Fx(Eigen::MatrixXd* Fx) const override;
    void computeJacobian_Fu(Eigen::MatrixXd* Fu) const override;
};

#endif
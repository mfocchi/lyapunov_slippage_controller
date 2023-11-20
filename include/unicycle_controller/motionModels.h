#ifndef MOTION_MODELS_H
#define MOTION_MODELS_H

#include <assert.h>  // debugging tool
// #include "unicycle_controller/generalPurposeEKF.h"
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
    VectorX_t x; // system state
    VectorX_t u; // control input
    VectorX_t u_prev; // previous control input
    data_t dt;
    
    void setPrevControlInput(const VectorX_t& u);
    void integrateEuler();
    void integrateTrapezoid(); 
    virtual void compute_fu( const VectorX_t& uk, VectorX_t* fu) const = 0;
    
public:
    MotionModel(){}
    MotionModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    ~MotionModel(){}
    // define virtual void methods
    void integrate();
    virtual void computeJacobian_Fx(MatrixX_t* Fx) const = 0;
    virtual void computeJacobian_Fu(MatrixX_t* Fu) const = 0;
    //getters
    void getState(VectorX_t *x_out) const;
    data_t getStepTime() const {return dt;}
    int getControlInputSize() const;                    
    int getStateSize() const;                    
    //setters
    void setControlInput(const VectorX_t& u);
    void setStepTime(data_t dt){this->dt = dt;}
    void changeIntegrationtype(Integration intType) {integrationType = intType;}
    void resetState(const VectorX_t& xInit) {x = xInit;}
};



class UnicycleModel : public MotionModel
{
protected:
    void compute_fu( const VectorX_t& u, VectorX_t* fu) const override;
public:
    // u=[v,w]
    UnicycleModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    
    void computeJacobian_Fx(MatrixX_t* Fx) const override;
    void computeJacobian_Fu(MatrixX_t* Fu) const override;
};


class DroneModel : public MotionModel
{
protected:
    void computeGbody(Vector3_t* g) const;  
    void computeSomegaQuaternion(MatrixX_t* Somega) const;
    void computeSomegaEuler(Matrix3_t* Somega) const;
    void compute_fu(const VectorX_t& u, VectorX_t* fu) const override;

public:
    // u=[ax,ay,az,omegax,omegay,omegaz]
    DroneModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    void computeJacobian_Fx(MatrixX_t* Fx) const override;
    void computeJacobian_Fu(MatrixX_t* Fu) const override;
};


class KinematicTrackedVehicleModel : public MotionModel
{
protected:
    data_t r; // [m] sproket_radius
    data_t B; // [m] distance_between_tracks
    void compute_fu(const VectorX_t& u, VectorX_t* fu) const override;
    // void extract();
public:
    // u=[omega_left, omega_right, slip_long_left, slip_long_right]
    KinematicTrackedVehicleModel(const VectorX_t& xInit, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    void computeJacobian_Fx(MatrixX_t* Fx) const override;
    void computeJacobian_Fu(MatrixX_t* Fu) const override;
};

#endif
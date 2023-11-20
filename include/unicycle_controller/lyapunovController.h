#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <sstream>
#include "../../../robot_model/include/robot_model/motionModels.h"
#include "error_codes.h"

class LyapController;
typedef std::shared_ptr<LyapController> LyapControllerPtr;

#define MAX_ITER_TO_PRINT 200
#define DECIMATION_PRINT 5
class LyapController 
{
private:
    data_t Kp;      //feedback proportional constant for longitudinal velocity
    data_t Ktheta;  //feedback proportional constant for angular velocity
    data_t time_end;
    UnicycleModelPtr RobotModel;      // model of the robot for the trajectory generation
    std::vector<Vector2_t> u_desired; // control inputs desired
    std::vector<Vector3_t> pose_desired; // pose desired obtained via integration of the u_desired
    Vector3_t pose_offset;
    data_t e_x;
    data_t e_y;
    data_t e_theta; 

    data_t current_time;
    bool finished;

    int getIndexIntegerBasedOnTime(data_t t) const;
    data_t getIndexFractionBasedOnTime(data_t t) const;
    bool endReached();
    void computeLaw(const Vector3_t& pose, Vector2_t* vel_out);
    void updateTrackingErrors(const Vector3_t& pose_ref, const Vector3_t& pose);
    data_t computeMaxTime() const;
public:
    LyapController(data_t Kp, data_t Ktheta, data_t dt);
    ~LyapController(){u_desired.clear(); pose_desired.clear();}

    void step(const Vector3_t& pose, Vector2_t* vel_out);

    data_t getTimeInterval() const {return RobotModel->getStepTime();}
    data_t getGainKp() const {return Kp;}
    data_t getGainKtheta() const {return Ktheta;}
    data_t getErrorX() const {return e_x;}
    data_t getErrorY() const {return e_y;}
    data_t getErrorTheta() const {return e_theta;}
    data_t getErrorDistance() const {return sqrt(pow(e_x, 2) + pow(e_y, 2));}
    
    void setGainKp(data_t Kp)           {this->Kp = Kp;}
    void setGainKtheta(data_t Ktheta)   {this->Ktheta = Ktheta;}

    void copyTrajectory(const std::vector<data_t>& v, const std::vector<data_t>& omega, const std::vector<data_t>& x, const std::vector<data_t>& y, const std::vector<data_t>& theta);
    void generateTrajectory();
    void integrateState(const Vector2_t& u, Vector3_t* pose_next);
    void addStateDesired(const Vector3_t& pose_des);
    void addStateDesiredOffset(const Vector3_t& offset)

    void addToInputDesired(data_t v, data_t omega);
    void resetTrajectory() {u_desired.clear(); pose_desired.clear();}
    void setCurrentTime(data_t t) {this->current_time = t;}

    void getControlInputDesiredOnTime(data_t t, Vector2_t* u_desired_out) const;
    void getPoseDesiredOnTime(data_t t, Vector3_t* pose_desired_out) const;
    std::string stringSetupInfo() const;

};      

data_t sinc(data_t val);

#endif
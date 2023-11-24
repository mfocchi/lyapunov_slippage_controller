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
#include "motionModels.h"
#include "error_codes.h"

class LyapController;
typedef std::shared_ptr<LyapController> LyapControllerPtr;

#define MAX_ITER_TO_PRINT 200
#define DECIMATION_PRINT 5
class LyapController 
{
private:
    double Kp;      //feedback proportional constant for longitudinal velocity
    double Ktheta;  //feedback proportional constant for angular velocity
    double time_end;
    UnicycleModelPtr RobotModel;      // model of the robot for the trajectory generation
    std::vector<Eigen::Vector2d> u_desired; // control inputs desired
    std::vector<Eigen::Vector3d> pose_desired; // pose desired obtained via integration of the u_desired
    Eigen::Vector3d pose_offset;
    double e_x;
    double e_y;
    double e_theta; 

    double current_time;
    bool finished;

    int getIndexIntegerBasedOnTime(double t) const;
    double getIndexFractionBasedOnTime(double t) const;
    bool endReached();
    void computeLaw(const Eigen::Vector3d& pose, Eigen::Vector2d* vel_out);
    void updateTrackingErrors(const Eigen::Vector3d& pose_ref, const Eigen::Vector3d& pose);
    double computeMaxTime() const;
public:
    LyapController(double Kp, double Ktheta, double dt);
    ~LyapController(){u_desired.clear(); pose_desired.clear();}

    void step(const Eigen::Vector3d& pose, Eigen::Vector2d* vel_out);

    double getTimeInterval() const {return RobotModel->getStepTime();}
    double getGainKp() const {return Kp;}
    double getGainKtheta() const {return Ktheta;}
    double getErrorX() const {return e_x;}
    double getErrorY() const {return e_y;}
    double getErrorTheta() const {return e_theta;}
    double getErrorDistance() const {return sqrt(pow(e_x, 2) + pow(e_y, 2));}
    
    void setGainKp(double Kp)           {this->Kp = Kp;}
    void setGainKtheta(double Ktheta)   {this->Ktheta = Ktheta;}

    void copyTrajectory(const std::vector<double>& v, const std::vector<double>& omega, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& theta);
    void generateTrajectory();
    void integrateState(const Eigen::Vector2d& u, Eigen::Vector3d* pose_next);
    void addStateDesired(const Eigen::Vector3d& pose_des);

    void addToInputDesired(double v, double omega);
    void resetTrajectory() {u_desired.clear(); pose_desired.clear();}
    void setCurrentTime(double t) {this->current_time = t;}
    void setStateOffset(double x, double y, double theta) {this->pose_offset << x,y,theta;}

    void getControlInputDesiredOnTime(double t, Eigen::Vector2d* u_desired_out) const;
    void getPoseDesiredOnTime(double t, Eigen::Vector3d* pose_desired_out) const;
    std::string stringSetupInfo() const;

};      

double sinc(double val);

#endif
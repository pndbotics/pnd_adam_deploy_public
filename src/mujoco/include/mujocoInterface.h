
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <mujoco/mujoco.h>
#include <unistd.h>
#include "robot_common.hpp"

#ifndef PI
#define PI 3.141592654
#endif  // PI

#ifndef TIME_STEP
#define TIME_STEP (2.5)
#endif  // TIME_STEP

#ifndef SAMPLE_TIME
#define SAMPLE_TIME (0.0025f)
#endif  // SAMPLE_TIME

#ifndef LEFTFOOT
#define LEFTFOOT 0
#endif  // LEFTFOOT

#ifndef RIGHTFOOT
#define RIGHTFOOT 1
#endif  // RIGHTFOOT

/**
 * @brief The Derivative class
 * Derivative in S-Domain:
 *            s
 * D(s) = -----------
 *          cs + 1
 *
 * e.g. c = 1e-4
 *
 * Tusting approximation:
 *                    1 - z^-1
 *  S(z) = alpha * ---------------
 *                    1 + z^-1
 */
  // model and data
extern mjModel* m; // 声明变量，而不是定义
extern mjData* d;
class Derivative {
 public:
  Derivative();
  Derivative(double dT, double c);
  void init(double dT, double c, double initValue);
  double mSig(double sigIn, double dT);

 private:
  double a0, a1, b0, b1;
  double sigInPrev;
  double sigOutPrev;
};

struct mujocoState {
  Eigen::VectorXd jointPosAct = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd jointVelAct = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd jointTorAct = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd imu9DAct = Eigen::VectorXd::Zero(9);
  Eigen::Vector3d waistRpyAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistRpyVelAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistXyzAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistXyzVelAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistXyzAccAct = Eigen::Vector3d::Zero();
  // Eigen::VectorXd footGrfAct = Eigen::VectorXd::Zero(12);
};

/**
 * @brief The MujocoRobot class
 */
class MujocoRobot {
 public:
  MujocoRobot(mjModel *model, mjData *data):mj_model_(model), mj_data_(data),mjRunning_(true){};
  ~MujocoRobot(){mjRunning_ = false;};

  void runMujoco();
  bool readData(double simTime, mujocoState &robotState);
  bool setMotorPos(const Eigen::VectorXd &jointPosTar, RobotData& robot_data, Eigen::VectorXd jointKp, Eigen::VectorXd jointKd);
  bool setMotorTau(const Eigen::VectorXd &jointTauTar);

 private:
  Eigen::VectorXd getMotorPos();
  Eigen::VectorXd getMotorTau();
  Eigen::VectorXd getMotorVel();
  // Eigen::VectorXd getFootForce6D(const int& footFlag);
  // Eigen::VectorXd getFootForce12D();
  Eigen::Vector3d rotm2Rpy(const Eigen::Matrix3d &rotm);
  Eigen::Vector3d rotm2xyz(const Eigen::Matrix3d &R);
  Eigen::Matrix3d rotx(const double theta);
  Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& quat);

  //mujoco
  mjModel *mj_model_;
  mjData *mj_data_;
  std::atomic<bool> mjRunning_;
};

class MujocoSim{
  public:
    MujocoSim(){};
    ~MujocoSim(){};
    int simLoop();
  
};

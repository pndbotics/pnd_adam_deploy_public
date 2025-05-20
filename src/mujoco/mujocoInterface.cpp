#include "mujocoInterface.h"

mjModel *m = nullptr;
mjData *d = nullptr;

void MujocoRobot::runMujoco(){
  while(mjRunning_){
    sleep(2);
  }
}
bool MujocoRobot::readData(double simTime, mujocoState &robotStateSim)
{
  // Get motor pos
  robotStateSim.jointPosAct = getMotorPos();

  // Get mmotor vel
  robotStateSim.jointVelAct = getMotorVel();

  // Motor torque
  robotStateSim.jointTorAct = getMotorTau();

  // IMU Data 9-dof
  Eigen::Vector4d quat(mj_data_->sensordata[kRobotDof*3+12],
          mj_data_->sensordata[kRobotDof*3+13],
          mj_data_->sensordata[kRobotDof*3+14],
          mj_data_->sensordata[kRobotDof*3+15]);
  
  Eigen::Matrix3d rotm = quaternionToRotationMatrix(quat);

  Eigen::Vector3d rpy = rotm2Rpy(rotm);

  robotStateSim.waistRpyAct << rpy(2), rpy(1), rpy(0);
//  robotStateSim.waistRpyAct << mj_data_->sensordata[kRobotDof*3+9],
//                  mj_data_->sensordata[kRobotDof*3+10],
//                  mj_data_->sensordata[kRobotDof*3+11];


  // the first three is linear velocity the second three is angular velocity
  Eigen::Vector3d angularRate;
  angularRate << mj_data_->sensordata[kRobotDof*3+3],
                 mj_data_->sensordata[kRobotDof*3+4],
                 mj_data_->sensordata[kRobotDof*3+5];
  

  robotStateSim.waistXyzAccAct << mj_data_->sensordata[kRobotDof*3],
                                  mj_data_->sensordata[kRobotDof*3+1],
                                  mj_data_->sensordata[kRobotDof*3+2];

  robotStateSim.waistRpyVelAct = angularRate;
  // robotStateSim.waistRpyVelAct = rotm.transpose() * angularRate;

  robotStateSim.imu9DAct << robotStateSim.waistRpyAct, robotStateSim.waistRpyVelAct, robotStateSim.waistXyzAccAct;

  return true;
}

bool MujocoRobot::setMotorPos(const Eigen::VectorXd &jointPosTar, RobotData& robot_data, Eigen::VectorXd jointKp,Eigen::VectorXd jointKd)
{
  for (int i = 0; i < kRobotDof; i++) {
    mj_data_->ctrl[i] = jointKp(i) * (robot_data.q_d_(i + 6) - robot_data.q_a_(i + 6)) + jointKd(i) * (-robot_data.q_dot_a_(i + 6));
  }
  return true;
}

bool MujocoRobot::setMotorTau(const Eigen::VectorXd &jointTauTar)
{
  for (int i = 0; i < kRobotDof; i++)
  {
    if (jointTauTar(i, 0) < 50000)
    {
      mj_data_->ctrl[i] = jointTauTar(i, 0);
    }
  }
  return true;
}

Eigen::VectorXd MujocoRobot::getMotorPos()
{
  Eigen::VectorXd Q = Eigen::VectorXd::Zero(kRobotDof);
  for (int i = 0; i < kRobotDof; i++)
  {
    // std::cout << "pos [" << i << "]" << "is" << mj_data_->sensordata[i] << std::endl;
    Q(i, 0) = mj_data_->sensordata[i];
  }
  return Q;
}

Eigen::VectorXd MujocoRobot::getMotorVel()
{
  Eigen::VectorXd Dq = Eigen::VectorXd::Zero(kRobotDof);
  for (int i = 0; i < kRobotDof; i++)
  {
    Dq(i, 0) = mj_data_->sensordata[kRobotDof + i];
  }
  return Dq;
}

Eigen::VectorXd MujocoRobot::getMotorTau()
{
  Eigen::VectorXd Tau = Eigen::VectorXd::Zero(kRobotDof);
  for (int i = 0; i < kRobotDof; i++)
  {
    // std::cout << "T [" << i << "]" << "is" << mj_data_->sensordata[2 * kRobotDof + i] << std::endl;
    Tau(i, 0) = mj_data_->sensordata[2 * kRobotDof + i];
  }
  return Tau;
}

Eigen::Vector3d MujocoRobot::rotm2Rpy(const Eigen::Matrix3d &rotm)
{
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  rpy(0) = atan2(rotm(2, 1), rotm(2, 2));
  rpy(1) = atan2(-rotm(2, 0), sqrt(rotm(2, 1) * rotm(2, 1) + rotm(2, 2) * rotm(2, 2)));
  rpy(2) = atan2(rotm(1, 0), rotm(0, 0));
  return rpy;
}

Eigen::Vector3d MujocoRobot::rotm2xyz(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();

  euler.setZero();
  // y (-pi/2 pi/2)
  euler(1) = asin(R(0, 2));
  // z [-pi pi]
  double sinz = -R(0, 1) / cos(euler(1));
  double cosz = R(0, 0) / cos(euler(1));
  euler(2) = atan2(sinz, cosz);
  // x [-pi pi]
  double sinx = -R(1, 2) / cos(euler(1));
  double cosx = R(2, 2) / cos(euler(1));
  euler(0) = atan2(sinx, cosx);

  return euler;
}

Eigen::Matrix3d MujocoRobot::rotx(const double theta)
{
  Eigen::Matrix3d matRes;
  matRes << 1., 0., 0., 0., std::cos(theta), -std::sin(theta), 0., std::sin(theta), std::cos(theta);
  return matRes;
}

// **************************************************************************************************************//

Derivative::Derivative(double dT, double c)
{
  double alpha(2. / dT);
  this->a0 = c * alpha + 1.;
  this->a1 = (1. - c * alpha) / this->a0;
  this->b0 = alpha / this->a0;
  this->b1 = -alpha / this->a0;
  this->a0 = 1.;
  this->sigInPrev = 0.;
  this->sigOutPrev = 0.;
}

void Derivative::init(double dT, double c, double initValue)
{
  double alpha(2. / dT);
  this->a0 = c * alpha + 1.;
  this->a1 = (1. - c * alpha) / this->a0;
  this->b0 = alpha / this->a0;
  this->b1 = -alpha / this->a0;
  this->a0 = 1.;
  this->sigInPrev = initValue;
  this->sigOutPrev = initValue;
}

double Derivative::mSig(double sigIn, double dT)
{
  double sigOut(0.);
  if (sigIn - sigInPrev < -PI)
  {
    sigOut = (2. * PI + sigIn - sigInPrev) / dT;
  }
  else if (sigIn - sigInPrev > PI)
  {
    sigOut = (sigIn - 2. * PI - sigInPrev) / dT;
  }
  else
  {
    sigOut = (sigIn - sigInPrev) / dT;
  }
  // sigOut = b0 * sigIn + b1 * sigInPrev - a1 * sigOutPrev;

  sigOutPrev = sigOut;
  sigInPrev = sigIn;
  return sigOut;
}

Eigen::Matrix3d MujocoRobot::quaternionToRotationMatrix(const Eigen::Vector4d& quat) {
    Eigen::Matrix3d rotMat;
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];
    
    rotMat(0, 0) = 1 - 2 * (y * y + z * z);
    rotMat(0, 1) = 2 * (x * y - z * w);
    rotMat(0, 2) = 2 * (x * z + y * w);
    
    rotMat(1, 0) = 2 * (x * y + z * w);
    rotMat(1, 1) = 1 - 2 * (x * x + z * z);
    rotMat(1, 2) = 2 * (y * z - x * w);
    
    rotMat(2, 0) = 2 * (x * z - y * w);
    rotMat(2, 1) = 2 * (y * z + x * w);
    rotMat(2, 2) = 1 - 2 * (x * x + y * y);
    
    return rotMat;
}
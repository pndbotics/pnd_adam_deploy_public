
#include <mujoco/mujoco.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "pconfig.hpp"
#include "robot_common.hpp"
#include "simulate.h"

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
extern mjModel* m;  // 声明变量，而不是定义
extern mjData* d;

struct mujocoState {
  Eigen::VectorXd jointPosAct = Eigen::VectorXd::Zero(kRobotDof + kHandsDof);
  Eigen::VectorXd jointVelAct = Eigen::VectorXd::Zero(kRobotDof + kHandsDof);
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
  MujocoRobot(mjModel* model, mjData* data) : mj_model_(model), mj_data_(data), mjRunning_(true) {};
  ~MujocoRobot() { mjRunning_ = false; };

  bool init() {
    if (!mj_model_ || !mj_data_) {
      std::cerr << "ERROR: Mujoco model or data is nullptr!" << std::endl;
      return false;
    }
    std::map<std::string, int> mj_joint_pos_name_id_;
    std::map<std::string, int> mj_joint_vel_name_id_;
    std::map<std::string, int> mj_joint_force_name_id_;
    for (int i = 0; i < mj_model_->nsensor; i++) {
      // const char* name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
      // if (name) {
      //   std::cout << "Sensor Name: " << name << " ID: " << i << std::endl;
      // }
      int sensor_type = mj_model_->sensor_type[i];
      int sensor_adr = mj_model_->sensor_adr[i];
      // int sensor_dim = mj_model_->sensor_dim[i];
      int objtype = mj_model_->sensor_objtype[i];
      int objid = mj_model_->sensor_objid[i];
      const char* objname = mj_id2name(mj_model_, objtype, objid);
      if (objtype == mjOBJ_JOINT) {
        if (sensor_type == mjSENS_JOINTPOS) {
          mj_joint_pos_name_id_[std::string(objname)] = sensor_adr;
        } else if (sensor_type == mjSENS_JOINTVEL) {
          mj_joint_vel_name_id_[std::string(objname)] = sensor_adr;
        } else if (sensor_type == mjSENS_JOINTACTFRC) {
          mj_joint_force_name_id_[std::string(objname)] = sensor_adr;
        }
      } else if (sensor_type == mjSENS_ACCELEROMETER) {
        imu_acc_adr_ = sensor_adr;
      } else if (sensor_type == mjSENS_GYRO) {
        imu_gyro_adr_ = sensor_adr;
      } else if (sensor_type == mjSENS_FRAMEQUAT) {
        imu_quat_adr_ = sensor_adr;
      }
    }

    auto joint_names = PConfig::getInst().jointNames();
    auto hand_joint_names = PConfig::getInst().fingerJointNames();
    auto all_joint_names = joint_names;
    all_joint_names.insert(all_joint_names.end(), hand_joint_names.begin(), hand_joint_names.end());
    if (all_joint_names.size() != mj_joint_pos_name_id_.size() ||
        all_joint_names.size() != mj_joint_vel_name_id_.size() ||
        all_joint_names.size() != mj_joint_force_name_id_.size()) {
      std::cerr << "ERROR: Mujoco model sensor number not match config joint number." << std::endl;
      return false;
    }

    for (size_t i = 0; i < joint_names.size(); i++) {
      if (mj_joint_pos_name_id_.find(joint_names[i]) == mj_joint_pos_name_id_.end() ||
          mj_joint_vel_name_id_.find(joint_names[i]) == mj_joint_vel_name_id_.end() ||
          mj_joint_force_name_id_.find(joint_names[i]) == mj_joint_force_name_id_.end()) {
        std::cerr << "ERROR: Joint name " << joint_names[i] << " not found in Mujoco sensor!" << std::endl;
        return false;
      }
      pnd_mj_pos_idx_.push_back(mj_joint_pos_name_id_[joint_names[i]]);
      pnd_mj_vel_idx_.push_back(mj_joint_vel_name_id_[joint_names[i]]);
      pnd_mj_force_idx_.push_back(mj_joint_force_name_id_[joint_names[i]]);
    }
    for (size_t i = 0; i < hand_joint_names.size(); i++) {
      if (mj_joint_pos_name_id_.find(hand_joint_names[i]) == mj_joint_pos_name_id_.end() ||
          mj_joint_vel_name_id_.find(hand_joint_names[i]) == mj_joint_vel_name_id_.end()) {
        std::cerr << "ERROR: Hand Joint name " << hand_joint_names[i] << " not found in Mujoco sensor!" << std::endl;
        return false;
      }
      pnd_hands_mj_pos_idx_.push_back(mj_joint_pos_name_id_[hand_joint_names[i]]);
      pnd_hands_mj_vel_idx_.push_back(mj_joint_vel_name_id_[hand_joint_names[i]]);
    }
    if (imu_acc_adr_ == -1 || imu_gyro_adr_ == -1 || imu_quat_adr_ == -1) {
      std::cerr << "ERROR: IMU sensor not found in Mujoco model!" << std::endl;
      return false;
    }

    return true;
  }

  void runMujoco();
  bool readData(double simTime, mujocoState& robotState);
  bool setMotorPos(RobotData& robot_data, Eigen::VectorXd jointKp, Eigen::VectorXd jointKd);
  bool setMotorTau(const Eigen::VectorXd& jointTauTar);

 private:
  Eigen::VectorXd getMotorPos();
  Eigen::VectorXd getMotorTau();
  Eigen::VectorXd getMotorVel();
  // Eigen::VectorXd getFootForce6D(const int& footFlag);
  // Eigen::VectorXd getFootForce12D();
  Eigen::Vector3d rotm2Rpy(const Eigen::Matrix3d& rotm);
  Eigen::Vector3d rotm2xyz(const Eigen::Matrix3d& R);
  Eigen::Matrix3d rotx(const double theta);
  Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& quat);

  // mujoco
  mjModel* mj_model_;
  mjData* mj_data_;
  std::atomic<bool> mjRunning_;
  std::vector<int> pnd_mj_pos_idx_;        // pnd joint order to mujoco joint index
  std::vector<int> pnd_mj_vel_idx_;        // pnd joint order to mujoco joint index
  std::vector<int> pnd_mj_force_idx_;      // pnd joint order to mujoco joint index
  std::vector<int> pnd_hands_mj_pos_idx_;  // pnd hand joint order to mujoco joint index
  std::vector<int> pnd_hands_mj_vel_idx_;  // pnd hand joint order to mujoco joint index
  int imu_acc_adr_ = -1;
  int imu_gyro_adr_ = -1;
  int imu_quat_adr_ = -1;
};

class MujocoSim {
 public:
  MujocoSim() {};
  ~MujocoSim() {};
  int simLoop();
  void stop() { sim_->exitrequest.store(1); }
  int exitstate() { return sim_->exitrequest.load(); }

 private:
  std::unique_ptr<mujoco::Simulate> sim_;
};

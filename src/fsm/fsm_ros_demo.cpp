#include "fsm/fsm_ros_demo.h"

#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "putil.h"

StateRosDemo::StateRosDemo(RobotData *robot_data) : FSMState(robot_data) { current_state_name_ = FSMStateName::MLP; }

StateRosDemo::~StateRosDemo() {}

void StateRosDemo::onEnter() {
  std::vector<std::string> obs_joint_names = {
      "hipPitch_Left",       "hipRoll_Left",       "hipYaw_Left",       "kneePitch_Left",  "anklePitch_Left",
      "ankleRoll_Left",  //
      "hipPitch_Right",      "hipRoll_Right",      "hipYaw_Right",      "kneePitch_Right", "anklePitch_Right",
      "ankleRoll_Right",                                                               //
      "waistRoll",           "waistPitch",         "waistYaw",                         //
      "shoulderPitch_Left",  "shoulderRoll_Left",  "shoulderYaw_Left",  "elbow_Left",  //
      "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right"};
  obs_joint_ids_ = PConfig::getInst().jointIdFromNames(obs_joint_names);
  for (size_t i = 0; i < obs_joint_ids_.size(); i++) {
    if (obs_joint_ids_[i] == -1) {
      std::cout << "mlp ERROR: joint " << obs_joint_names[i] << " not found" << std::endl;
    }
  }
  def_dof_pos_ = PConfig::getInst().defDofPosFromIds(obs_joint_ids_);
  zero_pos_ = PConfig::getInst().zeroPosFromIds(obs_joint_ids_);

  std::vector<std::string> wrist_yaw_joint_names = {"wristYaw_Left", "wristYaw_Right"};
  wrist_yaw_ids_ = PConfig::getInst().jointIdFromNames(wrist_yaw_joint_names);

  robot_data_->clip_qd_ = false;
}

void StateRosDemo::run() {
  // set des here
  robot_data_->q_d_.tail(kRobotDof);
  robot_data_->q_dot_d_.tail(kRobotDof);
  robot_data_->tau_d_.setZero();

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_);
#endif
}

FSMStateName StateRosDemo::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::MLP;
  }
}

void StateRosDemo::onExit() {}

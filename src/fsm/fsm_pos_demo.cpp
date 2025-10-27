#include "fsm/fsm_pos_demo.h"

#include "pconfig.hpp"
#include "pdata_handler.hpp"

StatePosDemo::StatePosDemo(RobotData* robot_data) : FSMState(robot_data) {
  current_state_name_ = FSMStateName::MLP;
  for (int i = 0; i < 4; ++i) {
    joint_pos_motion_.push_back(Eigen::VectorXd::Zero(kRobotDof + kHandsDof + kHandsLinearActuatorDof));
  }
  loadCfg();
}

StatePosDemo::~StatePosDemo() {}

void StatePosDemo::onEnter() {
  timer_ = 0.;
  robot_data_->clip_qd_ = true;
}

void StatePosDemo::run() {
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(kRobotDof + kHandsDof + kHandsLinearActuatorDof);
  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(kRobotDof + kHandsDof + kHandsLinearActuatorDof);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(kRobotDof + kHandsDof + kHandsLinearActuatorDof);
  Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(kRobotDof + kHandsDof + kHandsLinearActuatorDof);

  // parameters for demo motion
  double time_interval = 2.0;
  int motion_num = joint_pos_motion_.size();

  if (timer_ < (motion_num - 1) * time_interval) {
    int i = (int)(timer_ / time_interval);
    fifthPoly(joint_pos_motion_[i], zero, zero, joint_pos_motion_[i + 1], zero, zero, time_interval,
              timer_ - i * time_interval, joint_pos, joint_vel, joint_acc);
  } else {
    joint_pos = joint_pos_motion_[motion_num - 1];
    joint_vel.setZero();
  }

  // set des here
  robot_data_->q_d_.tail(kRobotDof) = joint_pos.head(kRobotDof);
  robot_data_->q_dot_d_.tail(kRobotDof) = joint_vel.head(kRobotDof);
  robot_data_->tau_d_.setZero();
  robot_data_->hands_q_d_ = joint_pos.segment(kRobotDof, kHandsDof);
  robot_data_->hands_la_q_d_ = joint_pos.tail(kHandsLinearActuatorDof);

  timer_ += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_);
#endif
}

FSMStateName StatePosDemo::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::MLP;
  }
}

void StatePosDemo::onExit() {}

void StatePosDemo::loadCfg() {
  auto config = PConfig::getInst().config();
  auto joint_names = PConfig::getInst().jointNames();
  auto hands_joint_names = PConfig::getInst().fingerJointNames();
  auto linear_actuator_names = PConfig::getInst().linearActuatorNames();
  joint_names.insert(joint_names.end(), hands_joint_names.begin(), hands_joint_names.end());
  joint_names.insert(joint_names.end(), linear_actuator_names.begin(), linear_actuator_names.end());
  assert(("Error: joint_names size mismatch", joint_names.size() == joint_pos_motion_[0].size()));
  auto motion = config["motion"];
  if (!motion) {
    std::cout << "ERROR: no motion data for StatePosDemo" << std::endl;
    exit(-1);
  }
  int joint_idx = 0;
  for (auto& joint_name : joint_names) {
    if (motion[joint_name]) {
      auto pos_list = motion[joint_name].as<std::vector<double>>();
      for (size_t i = 0; i < joint_pos_motion_.size(); ++i) {
        joint_pos_motion_[i][joint_idx] = pos_list[i];
      }
    } else {
      std::cout << "ERROR: no motion data for " << joint_name << std::endl;
    }
    ++joint_idx;
  }

  // for (size_t i = 0; i < joint_pos_motion_.size(); ++i) {
  //   for (size_t j = 0; j < joint_pos_motion_[i].size(); ++j) {
  //     std::cout << joint_pos_motion_[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
};
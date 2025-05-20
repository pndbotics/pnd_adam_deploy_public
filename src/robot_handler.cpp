#include "robot_handler.h"

#include <sys/resource.h>

#include <fstream>
#include <iostream>

#include "nlohmann/json/json.hpp"
#include "pconfig.hpp"
#include "putil.h"

AdamStatusCode initCpuRelated() {
  // set cpu-affinity
  cpu_set_t mask;
  int cpus = sysconf(_SC_NPROCESSORS_CONF);
  printf("cpus: %d\n", cpus);
  CPU_ZERO(&mask);           // init mask
  CPU_SET(cpus - 1, &mask);  // add last cup core to cpu set

  if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
    printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
    return AdamStatusCode::AdamStatusFailure;
  }
  usleep(1000);
  printf("set CPU affinity success\n");
  // set cpu-affinity

  // set sched-strategy
  struct sched_param sched;
  int max_priority = sched_get_priority_max(SCHED_FIFO);
  sched.sched_priority = max_priority;
  if (sched_setscheduler(getpid(), SCHED_FIFO, &sched) == -1) {
    printf("Set Scheduler Param, ERROR:%s\n", strerror(errno));
    return AdamStatusCode::AdamStatusFailure;
  }
  usleep(1000);
  printf("set scheduler success\n");

  if (setpriority(PRIO_PROCESS, getpid(), -20) == -1) {
    std::cerr << "Failed to set nice value" << std::endl;
    return AdamStatusCode::AdamStatusFailure;
  }
  usleep(1000);

  return AdamStatusCode::AdamStatusSuccess;
}

Framework::Framework(RobotData &robot_data) {
  initCpuRelated();

  auto state_mlp = FSMInit(robot_data);

  state_list_.zero = new StateZero(&robot_data);
  state_list_.mlp = state_mlp;
  state_list_.stop = new StateStop(&robot_data);
  current_state_ = state_list_.stop;
  disableJoints = false;
  first_run_ = true;
  next_state_ = state_list_.stop;
  next_state_name_ = FSMStateName::STOP;

#ifdef WEBOTS
  robot_common_ = std::make_unique<WebotsRobotImpl>();
#elif MUJOCO
  robot_common_ = std::make_unique<MujocoRobotImpl>();
#else
  robot_common_ = std::make_unique<RealRobot>();
#endif
}

Framework::~Framework() {
  delete state_list_.zero;
  delete state_list_.mlp;
  delete state_list_.stop;
}

bool Framework::init() {
  PndStateEstimateInit();
  readJointConfig();
  robot_common_->init();

  std::vector<std::string> names = {"hipPitch_Left",   "hipRoll_Left",    "hipYaw_Left",      "kneePitch_Left",
                                    "anklePitch_Left", "ankleRoll_Left",  "hipPitch_Right",   "hipRoll_Right",
                                    "hipYaw_Right",    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right"};
  leg_joint_ids_ = PConfig::getInst().jointIdFromNames(names);
  for (size_t i = 0; i < leg_joint_ids_.size(); i++) {
    if (leg_joint_ids_[i] == -1) {
      std::cout << "ERROR: joint " << names[i] << " not found" << std::endl;
      return false;
    }
  }
  leg_pos_ = Eigen::VectorXd::Zero(leg_joint_ids_.size());
  leg_vel_ = Eigen::VectorXd::Zero(leg_joint_ids_.size());
  leg_tau_ = Eigen::VectorXd::Zero(leg_joint_ids_.size());
  std::cout << "Init! " << std::endl;
  return true;
}

void Framework::getState(double t, RobotData &robot_data) {
  // update joint pos vel tau
  robot_common_->getState(t, robot_data);
  getInfoFormJointIds(robot_data.q_a_, leg_joint_ids_, leg_pos_);
  getInfoFormJointIds(robot_data.q_dot_a_, leg_joint_ids_, leg_vel_);
  getInfoFormJointIds(robot_data.tau_a_, leg_joint_ids_, leg_tau_);
  PndStateEstimate(t, robot_data, leg_pos_, leg_vel_, leg_tau_);
}

void Framework::setCommand(RobotData &robot_data) { robot_common_->setCommand(robot_data); }

void Framework::disableAllJoints() { robot_common_->disableAllJoints(); }

void Framework::entryStop() {
  if (next_state_name_ != FSMStateName::STOP) error_to_stop_ = true;
}

void Framework::runFSM() {
  if (first_run_) {
    current_state_->onEnter();
    first_run_ = false;
    std::cout << "FSM start!" << std::endl;
  }

  next_state_name_ = current_state_->checkTransition();
  if (error_to_stop_) {
    next_state_name_ = FSMStateName::STOP;
    error_to_stop_ = false;
    std::cout << "joint error goto stop" << std::endl;
  }
  if (next_state_name_ != current_state_->current_state_name_) {
    next_state_ = getNextState(next_state_name_);
    current_state_->onExit();
    current_state_ = next_state_;
    current_state_->onEnter();
  }
  current_state_->run();

  if (JsHum::getInst().disableJoints()) {
    disableJoints = true;
  }
}

FSMState *Framework::getNextState(FSMStateName stateName) const {
  switch (stateName) {
    case FSMStateName::ZERO:
      return state_list_.zero;
    case FSMStateName::MLP:
      return state_list_.mlp;
    case FSMStateName::STOP:
      return state_list_.stop;
    default:
      return state_list_.stop;
  }
}

void Framework::readJointConfig() {
  std::vector<std::string> joint_names = PConfig::getInst().jointNames();
  // read joint pd config json
  std::ifstream joint_pd_config_file(PConfig::getInst().jointConfigPath());
  nlohmann::json data;
  if (!joint_pd_config_file.is_open()) {
    std::cout << PConfig::getInst().jointConfigPath() << " not found" << std::endl;
    exit(1);
  }
  try {
    data = nlohmann::json::parse(joint_pd_config_file);
  } catch (std::exception &e) {
    std::cout << "joint_pd_config.json parse error" << std::endl;
    std::cout << e.what() << std::endl;
  }
  joint_pd_config_file.close();
  for (auto &item : data.items()) {
    auto it = std::find(joint_names.begin(), joint_names.end(), item.key());
    if (it != joint_names.end()) {
      int idx = std::distance(joint_names.begin(), it);
      robot_common_->joint_Kp_(idx) = item.value()["control_config"]["Kp"].get<double>();
      robot_common_->joint_Kd_(idx) = item.value()["control_config"]["Kd"].get<double>();
    } else {
      std::cout << "pd config: joint " << item.key() << " not used" << std::endl;
    }
  }
  // std::cout << "joint_Kp_:" << joint_Kp_.transpose() << std::endl;
  // std::cout << "joint_Kd_:" << joint_Kd_.transpose() << std::endl;
}

FSMStateName Framework::getCurrentState() { return current_state_->current_state_name_; }

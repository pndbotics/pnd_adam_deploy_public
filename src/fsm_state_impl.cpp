
#include "fsm_state_impl.h"

#include <torch/script.h>

#include "fsm/fsm_handle.hpp"
#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "putil.h"

torch::jit::script::Module mlp_model;

FSMState *FSMInit(RobotData &robot_data) {
  JsHum::getInst().init();
  mlp_model = torch::jit::load(PConfig::getInst().modelPb(), torch::kCPU);

  // torch warm up
  std::vector<torch::jit::IValue> inputs;
  auto input_data = torch::zeros({PConfig::getInst().obsNum()}).toType(torch::kFloat);
  input_data.to(torch::kCPU);
  inputs.emplace_back(input_data);
  for (int i = 0; i < 10; i++) {
    std::cout << "Warm up: " << i << std::endl;
    torch::Tensor output_data = mlp_model.forward(inputs).toTensor();
  }
  std::cout << "Warm up ready!" << std::endl;

  FSMState *state_mlp;
  state_mlp = FSMFactory::createState(PConfig::getInst().stateName(), &robot_data);
  if (state_mlp == nullptr) {
    std::cout << "state not found" << std::endl;
    exit(1);
  }
  state_mlp->mlp_model_ = &mlp_model;
  return state_mlp;
}

StateZero::StateZero(RobotData *robot_data) : FSMState(robot_data) { current_state_name_ = FSMStateName::ZERO; }

void StateZero::onEnter() {
  timer_ = 0.;
  init_joint_pos = robot_data_->q_a_.tail(kRobotDof);
}

void StateZero::run() {
  // keep warm
  if ((int)(timer_ / kDt) % freq_ == 0) {
    std::vector<torch::jit::IValue> inputs;
    auto input_data = torch::zeros({PConfig::getInst().obsNum()}).toType(torch::kFloat);
    input_data.to(torch::kCPU);
    inputs.emplace_back(input_data);
    torch::Tensor output_data = mlp_model.forward(inputs).toTensor();
    std::cout << robot_data_->q_a_.segment(3, 3).transpose() << std::endl;
  }

  if (timer_ < total_time) {
    fifthPoly(init_joint_pos, zero_, zero_, PConfig::getInst().zeroPos(), zero_, zero_, total_time, timer_, joint_pos_,
              joint_vel_, joint_acc_);
  } else {
    zero_finish_flag = true;
    joint_pos_ = PConfig::getInst().zeroPos();
    joint_vel_.setZero();
  }
  robot_data_->q_d_.tail(kRobotDof) = joint_pos_;
  robot_data_->q_dot_d_.tail(kRobotDof) = joint_vel_;
  robot_data_->tau_d_.setZero();
  robot_data_->pos_mode_ = true;

  timer_ += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_);
#endif
}

FSMStateName StateZero::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoMLP" && zero_finish_flag) {
    std::cout << "Zero2MLP" << std::endl;
    return FSMStateName::MLP;
  } else if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "Zero2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::ZERO;
  }
}

void StateZero::onExit() { zero_finish_flag = false; }

StateStop::StateStop(RobotData *robot_data) : FSMState(robot_data) { current_state_name_ = FSMStateName::STOP; }

void StateStop::onEnter() {
  timer_ = 0.;
  init_joint_pos = robot_data_->q_a_.tail(kRobotDof);
}

void StateStop::run() {
  // keep warm
  if ((int)(timer_ / kDt) % freq_ == 0) {
    std::vector<torch::jit::IValue> inputs;
    auto input_data = torch::zeros({PConfig::getInst().obsNum()}).toType(torch::kFloat);
    input_data.to(torch::kCPU);
    inputs.emplace_back(input_data);
    torch::Tensor output_data = mlp_model.forward(inputs).toTensor();
  }
  // damping stop
  robot_data_->q_d_.tail(kRobotDof) = (1 - 0.01) * robot_data_->q_a_.tail(kRobotDof) + 0.01 * init_joint_pos;
  robot_data_->q_dot_d_.tail(kRobotDof) = -3.0 * robot_data_->q_dot_a_.tail(kRobotDof);

  robot_data_->tau_d_.setZero();
  robot_data_->pos_mode_ = true;

  timer_ += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_);
#endif
}

FSMStateName StateStop::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoZero") {
    std::cout << "Stop2Zero" << std::endl;
    return FSMStateName::ZERO;
  } else {
    return FSMStateName::STOP;
  }
}

void StateStop::onExit() {}

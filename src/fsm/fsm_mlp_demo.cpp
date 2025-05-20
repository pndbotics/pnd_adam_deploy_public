#include "fsm/fsm_mlp_demo.h"

#include <torch/script.h>

#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "putil.h"

StateMlpDemo::StateMlpDemo(RobotData *robot_data) : FSMState(robot_data) {
  current_state_name_ = FSMStateName::MLP;
  input_data_mlp = Eigen::VectorXd::Zero(PConfig::getInst().obsNum());
  input_array = new float[PConfig::getInst().obsNum()];
}

StateMlpDemo::~StateMlpDemo() { delete[] input_array; }

void StateMlpDemo::onEnter() {
  timer_ = 0.;
  robot_data_->clip_qd_ = true;
}

void StateMlpDemo::run() {
  // link input_data_mlp and inputs
  std::vector<torch::jit::IValue> inputs;
  auto input_data = torch::zeros({PConfig::getInst().obsNum()}).toType(torch::kFloat);
  for (int i = 0; i < PConfig::getInst().obsNum(); i++) {
    input_array[i] = input_data_mlp(i);
  }
  input_data = torch::from_blob(input_array, PConfig::getInst().obsNum());
  input_data.to(torch::kCPU);
  inputs.emplace_back(input_data);

  if ((int)(timer_ / kDt) % freq_ == 0) {
    torch::Tensor output_data = mlp_model_->forward(inputs).toTensor();
    std::vector<float> out(output_data.data_ptr<float>(), output_data.data_ptr<float>() + output_data.numel());
    for (int i = 0; i < kObsDof; i++) {
      output_data_mlp(i) = out[i];
    }
  }

  Eigen::VectorXd zero = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_pos = PConfig::getInst().zeroPos();
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(kRobotDof);

  // set des here
  robot_data_->q_d_.tail(kRobotDof) = joint_pos;
  robot_data_->q_dot_d_.tail(kRobotDof) = joint_vel;
  robot_data_->tau_d_.setZero();

  timer_ += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_, output_data_mlp, input_data_mlp, 0);
#endif
}

FSMStateName StateMlpDemo::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::MLP;
  }
}

void StateMlpDemo::onExit() {}

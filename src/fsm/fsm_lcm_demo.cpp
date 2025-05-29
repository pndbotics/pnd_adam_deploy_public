#include "fsm/fsm_lcm_demo.h"

#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "putil.h"


StateLcmDemo::StateLcmDemo(RobotData *robot_data) : FSMState(robot_data) { current_state_name_ = FSMStateName::MLP; }

StateLcmDemo::~StateLcmDemo() {}

void StateLcmDemo::onEnter()
{
  robot_data_->q_dot_d_.setZero();
  robot_data_->tau_d_.setZero();
  robot_data_->clip_qd_ = false;
}

void StateLcmDemo::run()
{
  #ifdef DATALOG
    DataHandler::getInstance().cacheData(robot_data_, timer_);
  #endif
}

FSMStateName StateLcmDemo::checkTransition()
{
  if (JsHum::getInst().getStateChange() == "gotoStop")
  {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  }
  else
  {
    return FSMStateName::MLP;
  }
}

void StateLcmDemo::onExit() {}

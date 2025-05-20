#include "robot_common.hpp"
#include "mujocoInterface.h"
class MujocoRobotImpl : public RobotCommon {
 public:
 MujocoRobotImpl();
  ~MujocoRobotImpl() override;
  AdamStatusCode init() override;
  AdamStatusCode getState(double t, RobotData &robot_data) override;
  AdamStatusCode setCommand(RobotData &robot_data) override;
  AdamStatusCode disableAllJoints() override;

 private:
  MujocoRobot humanoid_;
  mujocoState robot_state_sim_;
  double sim_time_ = 0;
};
#pragma once

#include <functional>
#include <map>

#include "fsm/fsm_ankle_test_demo.h"
#include "fsm/fsm_lcm_demo.h"
#include "fsm/fsm_mlp_baseline_demo.h"
#include "fsm/fsm_mlp_demo.h"
#include "fsm/fsm_pos_demo.h"
#include "fsm/fsm_ros_demo.h"

std::map<std::string, std::function<FSMState *(RobotData *)>> FSMFactory::state_map_ = {
    {"StatePosDemo", [](RobotData *robot_data) { return new StatePosDemo(robot_data); }},
    {"StateAnkleTestDemo", [](RobotData *robot_data) { return new StateAnkleTestDemo(robot_data); }},
    {"StateMlpDemo", [](RobotData *robot_data) { return new StateMlpDemo(robot_data); }},
    {"StateMLP", [](RobotData *robot_data) { return new StateMLP(robot_data); }},
    {"StateRosDemo", [](RobotData *robot_data) { return new StateRosDemo(robot_data); }},
    {"StateLcmDemo", [](RobotData *robot_data) { return new StateLcmDemo(robot_data); }},
};
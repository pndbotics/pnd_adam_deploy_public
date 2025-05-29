#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <torch/torch.h>

#include <lcm/lcm-cpp.hpp>

#include "msg/pnd_lcm_types/pnd_imu_lcmt.hpp"
#include "msg/pnd_lcm_types/pnd_joint_cmd_lcmt.hpp"
#include "msg/pnd_lcm_types/pnd_robot_state_lcmt.hpp"
#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "robot_handler.h"

class LcmRobotPublisher {
 public:
  LcmRobotPublisher() {
    if (!lcm_.good()) {
      std::cerr << "LCM init failed" << std::endl;
      exit(1);
    }

    // subscribe command channel
    lcm_.subscribe("pnd_joint_cmd", &LcmRobotPublisher::joint_cmd_handler, this);

    // init command message
    joint_cmd_.num_joints = kRobotDof;
    joint_cmd_.dpos.resize(kRobotDof);
    joint_cmd_.dvel.resize(kRobotDof);
    joint_cmd_.dtau.resize(kRobotDof);
  }

  void joint_cmd_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const pnd_lcm::pnd_joint_cmd_t* msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    joint_cmd_ = *msg;
    has_new_command_ = true;
  }

  void publish_robot_state(const RobotData& robot_data) {
    state_msg_.num_joints = kRobotDof;
    state_msg_.apos.resize(kRobotDof);
    state_msg_.avel.resize(kRobotDof);
    state_msg_.atau.resize(kRobotDof);

    for (int i = 0; i < kRobotDof; i++) {
      state_msg_.apos[i] = robot_data.q_a_[i + kBaseNum];
      state_msg_.avel[i] = robot_data.q_dot_a_[i + kBaseNum];
      state_msg_.atau[i] = robot_data.tau_a_[i + kBaseNum];
    }

    lcm_.publish("pnd_robot_state", &state_msg_);
  }

  void publish_imu(const RobotData& robot_data) {
    imu_msg_.roll = robot_data.imu_data_[0];
    imu_msg_.pitch = robot_data.imu_data_[1];
    imu_msg_.yaw = robot_data.imu_data_[2];

    for (int i = 0; i < 3; i++) {
      imu_msg_.angular_velocity[i] = robot_data.imu_data_[i + 3];
      imu_msg_.linear_acceleration[i] = robot_data.imu_data_[i + 6];
    }

    lcm_.publish("pnd_imu", &imu_msg_);
  }

  void get_latest_command(RobotData& robot_data) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (has_new_command_) {
      for (int i = 0; i < kRobotDof; i++) {
        robot_data.q_d_[i + kBaseNum] = joint_cmd_.dpos[i];
        robot_data.q_dot_d_[i + kBaseNum] = joint_cmd_.dvel[i];
        robot_data.tau_d_[i + kBaseNum] = joint_cmd_.dtau[i];
      }
      has_new_command_ = false;
    }
  }

  void lcm_handle() {
    lcm_.handleTimeout(1);
  }

 private:
  lcm::LCM lcm_;
  pnd_lcm::pnd_joint_cmd_t joint_cmd_;
  pnd_lcm::pnd_robot_state_t state_msg_;
  pnd_lcm::pnd_imu_t imu_msg_;
  std::mutex cmd_mutex_;
  bool has_new_command_ = false;
};

int main(int argc, char** argv) {
#ifdef MUJOCO
  MujocoSim mujocoSim;
  std::thread mujocoThread(&MujocoSim::simLoop, &mujocoSim);
  sleep(3);
#endif
  at::set_num_threads(1);          // Disables the intraop thread pool.
  at::set_num_interop_threads(1);  // Disables the interop thread pool.

  // load config
  PConfig::getInst().loadConfig();
  if (PConfig::getInst().stateName() != "StateLcmDemo") {
    std::cout << "Error: StateName is not StateLcmDemo, please check config.yaml" << std::endl;
    return -1;
  }

  // robot_data init
  RobotData robot_data;

  // framework init
  Framework framework(robot_data);
  bool res = framework.init();
  if (!res) {
    std::cout << "framework init failed" << std::endl;
    return -1;
  }

  DataHandler::getInstance().init();

  auto robot_publisher = std::make_shared<LcmRobotPublisher>();
  std::thread lcm_thread([&]() {
    while (!framework.disableJoints) {
      robot_publisher->lcm_handle();
    }
  });

  broccoli::core::Time start_time;
  broccoli::core::Time total_time;
  broccoli::core::Time period(0, 2500000);
  broccoli::core::Time sleep2time;
  broccoli::core::Time timer;
  timespec sleep2time_spec;
  double time_fsm = 0.0;

  broccoli::core::Time get_state_time;
  broccoli::core::Time fsm_time;
  broccoli::core::Time cmd_time;

  // 2.5 ms timing loop
  while (true) {
    total_time = timer.currentTime() - start_time;  // total time
    start_time = timer.currentTime();

    framework.getState(time_fsm, robot_data);  // get state (position, velocity, current) from robot

    robot_publisher->publish_robot_state(robot_data);
    robot_publisher->publish_imu(robot_data);

    get_state_time = timer.currentTime() - start_time;  // get state execution time
    if (JsHum::getInst().getStateChange() == "gotoMLP") {
      robot_publisher->get_latest_command(robot_data);
    }
    framework.runFSM();                // run fsm (The calculation time cannot exceed 1.5ms)
    framework.setCommand(robot_data);  // send commands to joints

    fsm_time = timer.currentTime() - start_time - get_state_time;  // Finite state machine execution time

    cmd_time = timer.currentTime() - start_time - fsm_time - get_state_time;  // send command execution time

    time_fsm += kDt;

#ifdef DATALOG
    // write data
    DataHandler::getInstance().cacheData(robot_data, time_fsm, get_state_time, fsm_time, cmd_time, start_time,
                                         total_time, timer, framework.getCurrentState());
    DataHandler::getInstance().writeData(robot_data);
#endif

    if (framework.disableJoints) {
      break;
    }
    if (robot_data.error_state_) {
      framework.entryStop();
    }

    sleep2time = start_time + period;
    sleep2time_spec = sleep2time.toTimeSpec();
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(sleep2time_spec), NULL);
  }

  framework.disableAllJoints();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  lcm_thread.join();
  return 0;
}

#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <torch/torch.h>

#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "robot_handler.h"

#include "rclcpp/rclcpp.hpp"
#include "pndbotics_publisher.hpp" 

int main(int argc, char **argv) {
#ifdef MUJOCO
  MujocoSim mujocoSim;
  std::thread mujocoThread(&MujocoSim::simLoop, &mujocoSim);
  sleep(3);
#endif
  rclcpp::init(argc, argv); // 初始化ROS2
    
  // 创建ROS2节点
  auto robot_publisher = std::make_shared<RobotPublisher>();
  std::thread ros_spin_thread([&]() {
    rclcpp::spin(robot_publisher);
  });

  at::set_num_threads(1);          // Disables the intraop thread pool.
  at::set_num_interop_threads(1);  // Disables the interop thread pool.

  // load config
  PConfig::getInst().loadConfig();

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

        // 发布实际状态和IMU数据
    robot_publisher->publish_robot_state(robot_data);
    robot_publisher->publish_imu(robot_data);
    
    get_state_time = timer.currentTime() - start_time;  // get state execution time
    if(JsHum::getInst().getStateChange() == "gotoMLP"){
      robot_publisher->get_latest_command(robot_data);
    }
      // framework.setCommand(robot_data);  // send commands to joints
    framework.runFSM(); // run fsm (The calculation time cannot exceed 1.5ms)
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
  rclcpp::shutdown();
  ros_spin_thread.join();
  return 0;
}
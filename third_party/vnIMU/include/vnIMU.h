#ifndef VNIMU_h
#define VNIMU_h

#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

class vnIMU {
 private:
  const std::string SensorPort = "/dev/ttyUSB0";
  const uint32_t SensorBaudrate = 115200;
  const uint32_t SensorBaudrate2 = 921600;
  VnSensor vs;
 public:
  bool initIMU();
  bool closeIMU();
  ~vnIMU();
  static void asciiOrBinaryAsyncMessageReceived(void *userData, Packet &p, size_t index);
  static Eigen::VectorXd imuData;
};

#endif
#pragma once
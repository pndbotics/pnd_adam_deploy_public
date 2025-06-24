
#pragma once

#include <Eigen/Dense>
#include <memory>

#include "hi14r5/hipnuc.h"
#include "vnIMU.h"

enum class ImuType { VN100_IMU, CH_IMU, UNKNOWN };

ImuType checkImuType();

class ImuInterface {
 public:
  virtual ~ImuInterface() = default;
  virtual bool initialize() = 0;
  virtual const Eigen::VectorXd& getImuData() = 0;
  virtual bool close() { return true; }
};

class Vn100Imu : public ImuInterface {
 public:
  bool initialize() override { return vn_imu_.initIMU(); }
  const Eigen::VectorXd& getImuData() override { return vnIMU::imuData; }
  // bool close() override { return vn_imu_.closeIMU(); }

 private:
  vnIMU vn_imu_;
};

class ChImu : public ImuInterface {
 public:
  bool initialize() override {
    R_hi14r5.row(0) << 0, -1, 0;
    R_hi14r5.row(1) << -1, 0, 0;
    R_hi14r5.row(2) << 0, 0, -1;
    return hipnuc_init() == 0;
  }
  const Eigen::VectorXd& getImuData() override {
    hipnuc_read(imu_data);
    hi_newIMUData[0] = imu_data[0] / 180.0 * M_PI;   // negative yaw
    hi_newIMUData[1] = -imu_data[1] / 180.0 * M_PI;  // negative pitch
    hi_newIMUData[2] = imu_data[2] > 0.0 ? (imu_data[2] / 180.0 - 1.0) * M_PI
                                         : (imu_data[2] / 180.0 + 1.0) * M_PI;  // roll + pi, modify to be in [-pi,pi]
    hi_newIMUData[1] = -hi_newIMUData[1];
    hi_newIMUData[2] = -hi_newIMUData[2];
    hi_newIMUData.tail(6) << imu_data[3] / 180.0 * M_PI, imu_data[4] / 180.0 * M_PI, imu_data[5] / 180.0 * M_PI,
        imu_data[6] * 9.81, imu_data[7] * 9.81, imu_data[8] * 9.81;
    hi_newIMUData.block(3, 0, 3, 1) = R_hi14r5 * hi_newIMUData.block(3, 0, 3, 1);
    hi_newIMUData.tail(3) = R_hi14r5 * hi_newIMUData.tail(3);
    return hi_newIMUData;
  }
  bool close() override { return hipnuc_close() == 0; }

 private:
  Eigen::Matrix3d R_hi14r5 = Eigen::Matrix3d::Zero();
  Eigen::VectorXd hi_newIMUData = Eigen::VectorXd::Zero(9);
  float imu_data[9] = {0.0f};
};

class ImuHandler {
 public:
  ImuHandler() = default;
  ~ImuHandler() = default;
  bool initialize() {
    ImuType imu_type = checkImuType();
    if (imu_type == ImuType::VN100_IMU) {
      imu_interface = std::make_shared<Vn100Imu>();
      return imu_interface->initialize();
    } else if (imu_type == ImuType::CH_IMU) {
      imu_interface = std::make_shared<ChImu>();
      return imu_interface->initialize();
    } else {
      return false;
    }
  }
  const Eigen::VectorXd& getImuData() const { return imu_interface->getImuData(); }
  bool close() {
    if (imu_interface) {
      return imu_interface->close();
    }
    return true;
  }

 private:
  std::shared_ptr<ImuInterface> imu_interface = nullptr;
};
